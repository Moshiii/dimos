#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""H.264 into modules: the input mixin, and the standalone decoder.

``H264InputMixin`` adds a ``video`` In (H.264 ``CompressedVideo``) to a module
that has an ``In[Image]``, decodes internally, and feeds the frames into that
image input's own transport — downstream code cannot tell them from wire
traffic. Ports declared in a mixin are collected like any other (annotations
merge across the MRO), so the whole adaptation is::

    class VideoMarkerDetectionModule(H264InputMixin, MarkerDetectionStreamModule):
        config: VideoMarkerDetectionModuleConfig

Trade-off vs the standalone :class:`H264DecoderModule` below: the mixin is one
module less in the graph, but each video-capable module owns its own decoder —
two of them watching the same stream decode it twice. Share the standalone
module when pixels have several consumers. Don't wire both ``video`` and the
image topic: the input would see both streams.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any, ClassVar

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from av.video.codeccontext import VideoCodecContext

    # The mixin only ever runs inside a Module, so type it as one — `start`,
    # `register_disposable` and `config` are the host's. At runtime it stays a
    # plain object, or it would be collected as a module in its own right.
    _MixinHost = Module
else:
    _MixinHost = object

logger = setup_logger()


class H264InputConfig(ModuleConfig):
    # Decode everything (H.264 reference frames don't survive skipping), feed
    # the image input at most this often. 0 = every decoded frame.
    decode_hz: float = 5.0


class H264InputMixin(_MixinHost):
    """Mixin: an H.264 ``video`` In decoded into the host's image In."""

    video: In[CompressedVideo]

    # Decode is CPU-bound and runs in the host's process, so sharing a worker
    # stalls every module on it. The mixin leads the MRO, so a host that wants
    # otherwise still wins by setting this on itself.
    dedicated_worker: ClassVar[bool] = True

    # Name of the host module's In[Image] to feed.
    image_port: ClassVar[str] = "color_image"

    # Untyped here on purpose: class annotations are resolved at runtime to
    # collect ports, so an `av` type would make this optional dep a hard
    # import. It is narrowed at the one place it is used.
    _decoder: Any = None
    _last_fed: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        import av

        self._decoder = av.codec.CodecContext.create("h264", "r")
        self.register_disposable(self.video.observable().subscribe(self._decode_into_image))

    def _decode_into_image(self, msg: CompressedVideo) -> None:
        import av

        decoder: VideoCodecContext | None = self._decoder
        if decoder is None:
            return  # packet raced ahead of start()
        try:
            frames = decoder.decode(av.packet.Packet(msg.data.tobytes()))
        except av.error.FFmpegError:
            return  # P-frame with no reference yet (joined mid-GOP)
        except Exception:
            logger.exception("video mixin decode failed, skipping packet")
            return
        if not frames:
            return
        # The host owns the config; only H264InputConfig subclasses carry the
        # knob, so read it off the instance rather than narrowing config here.
        decode_hz: float = getattr(self.config, "decode_hz", 5.0)
        now = time.monotonic()
        if decode_hz > 0 and now - self._last_fed < 1.0 / decode_hz:
            return
        self._last_fed = now
        bgr = frames[-1].to_ndarray(format="bgr24")
        image = Image.from_numpy(bgr, ImageFormat.BGR, msg.frame_id, msg.ts)
        # The image port may be an In (inject into its transport — downstream
        # subscribers see wire traffic) or an Out (plain publish): the mixin
        # serves both the retrofit case and a standalone decoder module.
        port = getattr(self, self.image_port)
        publish = getattr(port, "publish", None) or port.transport.publish
        publish(image)


class H264DecoderModule(H264InputMixin, Module):
    """``video`` (H.264 CompressedVideo) in, throttled BGR ``color_image`` out.

    The mixin publishing on an Out: one shared decode for graphs where
    several consumers want pixels. A single consumer can take the mixin
    directly and skip this hop.
    """

    config: H264InputConfig

    color_image: Out[Image]
