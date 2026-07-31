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
          pass

* :class:`H264DecoderModule` publishes on an Out instead, for graphs where
  several consumers should share one decode.

Every decoded frame is emitted. Rate is the consumer's business — subscribe
through ``observable()`` for latest-wins backpressure, or thin the stream with
a memory2 transform. An Image over a decoded frame costs 0.6 us and shares the
buffer, so the frames a consumer ignores are close to free.

Longer term this belongs beside ``jpeg_lcm``/``jpeg_shm`` as a transport codec
(:mod:`dimos.protocol.pubsub.encoders`), so consumers keep a plain ``In[Image]``
and nothing in the graph knows the wire was compressed. That needs
per-subscription decoder state, which the per-message encoder contract lacks.
:class:`~dimos.robot.unitree.go2.dds.video.H264Decoder` is a third copy of the
same decode, as a memory2 transform.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING, Any

import reactivex as rx

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from reactivex import Observable

    # The mixin only ever runs inside a Module, so type it as one — `start` and
    # `register_disposable` are the host's. At runtime it stays a plain object,
    # or it would be collected as a module in its own right.
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


def _decoded(video: In[CompressedVideo]) -> Observable[Image]:
    """``video`` decoded.

    ``pure_observable`` on purpose: the default is latest-wins backpressured, and
    a dropped packet costs every frame until the next keyframe. Whatever thins
    the stream has to sit downstream of the decoder, never in front of it.
    """
    return video.pure_observable().pipe(h264_decode())


class H264InputMixin(_MixinHost):
    """Mixin: an H.264 ``video`` In decoded into the host's image In."""

    video: In[CompressedVideo]

    if TYPE_CHECKING:
        # The host's port, not one the mixin contributes — declared for type
        # checking only, so a host that names its image In otherwise doesn't
        # inherit a stray `color_image`. Runtime never sees this annotation.
        color_image: In[Image]

    @property
    def image_in(self) -> In[Image]:
        """The In decoded frames feed. Override to point at a different port."""
        return self.color_image

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(_decoded(self.video).subscribe(self.image_in.transport.publish))


class H264DecoderModule(Module):
    """``video`` (H.264 CompressedVideo) in, BGR ``color_image`` out.

    One decode shared by every subscriber, for graphs with several consumers.
    """

    video: In[CompressedVideo]
    color_image: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(_decoded(self.video).subscribe(self.color_image.publish))
