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

Trade-off vs a standalone :class:`~dimos.perception.video.h264_decoder_module.
H264DecoderModule`: the mixin is one module less in the graph, but each
video-capable module owns its own decoder — two of them watching the same
stream decode it twice. Share the standalone module when pixels have several
consumers. Don't wire both ``video`` and the image topic: the input would
see both streams.
"""

from __future__ import annotations

import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class H264InputConfig(ModuleConfig):
    # Decode everything (H.264 reference frames don't survive skipping), feed
    # the image input at most this often. 0 = every decoded frame.
    decode_hz: float = 5.0


class H264InputMixin:
    """Mixin: an H.264 ``video`` In decoded into the host's image In."""

    video: In[CompressedVideo]

    # Name of the host module's In[Image] to feed.
    image_port: str = "color_image"

    _decoder: Any = None
    _last_fed: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()  # type: ignore[misc]
        import av

        self._decoder = av.codec.CodecContext.create("h264", "r")
        self.register_disposable(  # type: ignore[attr-defined]
            self.video.observable().subscribe(self._decode_into_image)  # type: ignore[no-untyped-call]
        )

    def _decode_into_image(self, msg: CompressedVideo) -> None:
        import av

        try:
            frames = self._decoder.decode(av.packet.Packet(msg.data.tobytes()))
        except av.error.FFmpegError:
            return  # P-frame with no reference yet (joined mid-GOP)
        except Exception:
            logger.exception("video mixin decode failed, skipping packet")
            return
        if not frames:
            return
        decode_hz = getattr(self.config, "decode_hz", 5.0)  # type: ignore[attr-defined]
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

    dedicated_worker = True
    config: H264InputConfig

    color_image: Out[Image]
