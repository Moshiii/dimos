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

"""H.264 as a reactive operator, plus the two ways to attach it to a module.

:func:`h264_decode` is the whole implementation — packets in, BGR frames out,
one decoder per subscription. The rest is wiring:

* :class:`H264InputMixin` decodes into the host's *own* ``color_image`` In, so
  a consumer takes compressed video off the wire and never sees raw frames
  crossing a transport::

      class VideoMarkerDetectionModule(H264InputMixin, MarkerDetectionStreamModule):
          config: VideoMarkerDetectionModuleConfig

* :class:`H264DecoderModule` publishes on an Out instead, for graphs where
  several consumers should share one decode.

Longer term this belongs beside ``jpeg_lcm``/``jpeg_shm`` as a transport codec
(:mod:`dimos.protocol.pubsub.encoders`), so consumers keep a plain ``In[Image]``
and nothing in the graph knows the wire was compressed. That needs
per-subscription decoder state, which the per-message encoder contract lacks.
:class:`~dimos.robot.unitree.go2.dds.video.H264Decoder` is a third copy of the
same decode, as a memory2 transform.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any, ClassVar

import reactivex as rx
from reactivex import operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from reactivex import Observable

    # The mixin only ever runs inside a Module, so type it as one — `start`,
    # `register_disposable` and `config` are the host's. At runtime it stays a
    # plain object, or it would be collected as a module in its own right.
    _MixinHost = Module
else:
    _MixinHost = object

logger = setup_logger()


def h264_decode() -> Callable[[Observable[CompressedVideo]], Observable[Image]]:
    """Decode an ordered H.264 packet stream into BGR frames, latest frame per packet.

    State lives per subscription: reference frames only make sense against the
    stream that produced them. Packets that resolve to nothing — P-frames before
    the first keyframe — are dropped until the decoder re-syncs.
    """

    def _operator(source: Observable[CompressedVideo]) -> Observable[Image]:
        def _subscribe(observer: Any, scheduler: Any = None) -> Any:
            import av

            decoder = av.codec.CodecContext.create("h264", "r")

            def on_next(msg: CompressedVideo) -> None:
                try:
                    frames = decoder.decode(av.packet.Packet(msg.data.tobytes()))
                except av.error.FFmpegError:
                    return  # no reference frame yet (joined mid-GOP)
                except Exception:
                    logger.exception("h264 decode failed, skipping packet")
                    return
                if not frames:
                    return
                bgr = frames[-1].to_ndarray(format="bgr24")
                observer.on_next(Image.from_numpy(bgr, ImageFormat.BGR, msg.frame_id, msg.ts))

            return source.subscribe(
                on_next,
                observer.on_error,
                observer.on_completed,
                scheduler=scheduler,
            )

        return rx.create(_subscribe)

    return _operator


def _decoded(video: In[CompressedVideo], publish_hz: float) -> Observable[Image]:
    """``video`` decoded, thinned to ``publish_hz``.

    ``pure_observable`` on purpose: the default is latest-wins backpressured, and
    a dropped packet costs every frame until the next keyframe. Decode every
    packet, throttle the frames that come out.
    """
    stages: list[Any] = [h264_decode()]
    if publish_hz > 0:
        stages.append(ops.throttle_first(1.0 / publish_hz))
    return video.pure_observable().pipe(*stages)


class H264InputConfig(ModuleConfig):
    # Every packet is decoded — reference frames don't survive skipping — but
    # frames reach the consumer at most this often. 0 = every frame.
    publish_hz: float = 5.0


class H264InputMixin(_MixinHost):
    """Mixin: an H.264 ``video`` In decoded into the host's own ``color_image`` In."""

    video: In[CompressedVideo]
    # Merged with the host's identically-named port: annotations collect across
    # the MRO, so this is the consumer's own input, fed from inside.
    color_image: In[Image]

    # Only H264InputConfig subclasses carry the knob; the host owns config.
    _default_publish_hz: ClassVar[float] = 5.0

    @rpc
    def start(self) -> None:
        super().start()
        hz: float = getattr(self.config, "publish_hz", self._default_publish_hz)
        self.register_disposable(
            _decoded(self.video, hz).subscribe(self.color_image.transport.publish)
        )


class H264DecoderModule(Module):
    """``video`` (H.264 CompressedVideo) in, throttled BGR ``color_image`` out.

    One decode shared by every subscriber, for graphs with several consumers.
    """

    config: H264InputConfig

    video: In[CompressedVideo]
    color_image: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            _decoded(self.video, self.config.publish_hz).subscribe(self.color_image.publish)
        )
