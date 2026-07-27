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

"""Live H.264 -> Image decoding, for consumers that need pixels.

The viewer decodes ``CompressedVideo`` itself, but perception modules
(fiducial detection, tracking) want BGR frames. Every packet is fed to the
decoder — H.264 reference frames don't survive skipping — while emission is
throttled to ``emit_hz``: detection rarely wants 40 frames a second.

The live twin of :class:`dimos.robot.unitree.go2.dds.video.H264Decoder`
(the memory2 pipeline transformer).
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


class H264DecoderModuleConfig(ModuleConfig):
    # Decode everything, emit at most this often (0 = every decoded frame).
    emit_hz: float = 5.0


class H264DecoderModule(Module):
    """``video`` (H.264 CompressedVideo) in, throttled BGR ``color_image`` out."""

    dedicated_worker = True
    config: H264DecoderModuleConfig

    video: In[CompressedVideo]
    color_image: Out[Image]

    _decoder: Any = None
    _last_emit: float = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        import av

        self._decoder = av.codec.CodecContext.create("h264", "r")
        self.register_disposable(self.video.observable().subscribe(self._on_packet))  # type: ignore[no-untyped-call]

    def _on_packet(self, msg: CompressedVideo) -> None:
        import av

        try:
            frames = self._decoder.decode(av.packet.Packet(msg.data.tobytes()))
        except av.error.FFmpegError:
            return  # P-frame with no reference yet (joined mid-GOP)
        except Exception:
            logger.exception("h264 decode failed, skipping packet")
            return
        if not frames:
            return
        now = time.monotonic()
        if self.config.emit_hz > 0 and now - self._last_emit < 1.0 / self.config.emit_hz:
            return
        self._last_emit = now
        bgr = frames[-1].to_ndarray(format="bgr24")
        self.color_image.publish(Image.from_numpy(bgr, ImageFormat.BGR, msg.frame_id, msg.ts))
