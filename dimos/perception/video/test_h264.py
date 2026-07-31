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

from __future__ import annotations

from typing import Any

import numpy as np

from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.fiducial.marker_detection_stream_module import VideoMarkerDetectionModule
from dimos.perception.video.h264 import H264DecoderModule

PACKET = CompressedVideo(data=b"\x00\x00\x00\x01\x65payload", format="h264", frame_id="cam", ts=1.0)


class _Frame:
    """Stands in for an ``av`` frame; only ``to_ndarray`` is reached."""

    def __init__(self, bgr: np.ndarray) -> None:
        self._bgr = bgr

    def to_ndarray(self, format: str) -> np.ndarray:
        assert format == "bgr24"
        return self._bgr


class _Decoder:
    def __init__(self, *frames: Any) -> None:
        self._frames = list(frames)

    def decode(self, packet: Any) -> list[Any]:
        return self._frames


def _bgr(value: int = 0) -> np.ndarray:
    return np.full((4, 4, 3), value, dtype=np.uint8)


def test_mixin_port_merges_into_the_host_module() -> None:
    """The whole point: a video In appears on the host without touching it."""
    module = H264DecoderModule()
    try:
        assert "video" in module.inputs
        assert set(module.outputs) == {"color_image"}
    finally:
        module.stop()


def test_mixin_port_merges_across_the_mro_onto_an_existing_consumer() -> None:
    module = VideoMarkerDetectionModule(marker_length_m=0.18)
    try:
        assert {"video", "color_image"} <= set(module.inputs)
        assert set(module.outputs) == {"detections"}
    finally:
        module.stop()


def test_decoded_frame_is_published_on_the_image_port() -> None:
    module = H264DecoderModule()
    try:
        seen: list[Image] = []
        module.color_image.subscribe(seen.append)
        module._decoder = _Decoder(_Frame(_bgr(7)))

        module._decode_into_image(PACKET)

        assert len(seen) == 1
        assert seen[0].format is ImageFormat.BGR
        assert seen[0].frame_id == "cam"
        assert seen[0].ts == PACKET.ts
    finally:
        module.stop()


def test_only_the_latest_frame_of_a_packet_is_fed() -> None:
    """Decoding never skips, but only the newest picture reaches the consumer."""
    module = H264DecoderModule()
    try:
        seen: list[Image] = []
        module.color_image.subscribe(seen.append)
        module._decoder = _Decoder(_Frame(_bgr(1)), _Frame(_bgr(9)))

        module._decode_into_image(PACKET)

        assert len(seen) == 1
        assert seen[0].as_numpy()[0, 0, 0] == 9
    finally:
        module.stop()


def test_decode_hz_throttles_the_feed() -> None:
    module = H264DecoderModule(decode_hz=1.0)
    try:
        seen: list[Image] = []
        module.color_image.subscribe(seen.append)
        module._decoder = _Decoder(_Frame(_bgr()))

        module._decode_into_image(PACKET)
        module._decode_into_image(PACKET)

        assert len(seen) == 1
    finally:
        module.stop()


def test_decode_hz_zero_feeds_every_decoded_frame() -> None:
    module = H264DecoderModule(decode_hz=0.0)
    try:
        seen: list[Image] = []
        module.color_image.subscribe(seen.append)
        module._decoder = _Decoder(_Frame(_bgr()))

        module._decode_into_image(PACKET)
        module._decode_into_image(PACKET)

        assert len(seen) == 2
    finally:
        module.stop()


def test_a_packet_that_decodes_to_nothing_is_not_published() -> None:
    """Joining mid-GOP: P-frames arrive before any reference picture."""
    module = H264DecoderModule()
    try:
        seen: list[Image] = []
        module.color_image.subscribe(seen.append)
        module._decoder = _Decoder()

        module._decode_into_image(PACKET)

        assert seen == []
    finally:
        module.stop()
