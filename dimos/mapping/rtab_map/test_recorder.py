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

import asyncio
from collections.abc import Iterator
from pathlib import Path
from typing import Any, cast

import numpy as np
import pytest

from dimos.mapping.rtab_map.recorder import RtabmapRecorder
from dimos.memory2.backend import Backend
from dimos.memory2.codecs.base import codec_id
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def recorder(tmp_path: Path) -> Iterator[RtabmapRecorder]:
    module = RtabmapRecorder(db_path=str(tmp_path / "rtabmap.db"))
    module._prepare_streams()
    yield module
    module.store.stop()
    module.stop()


def _codec_of(recorder: RtabmapRecorder, stream: str, payload: type[Any]) -> str:
    backend = cast("Backend[Any]", recorder.store.stream(stream, payload)._source)
    return codec_id(backend.codec)


def test_depth_is_lossless_and_color_is_jpeg(recorder: RtabmapRecorder) -> None:
    """Depth carries millimetres, not a picture: JPEG would go through RGB."""
    assert _codec_of(recorder, "depth_image", Image) == "lz4+lcm"
    assert _codec_of(recorder, "color_image", Image) == "jpeg"


def test_cloud_map_is_lossless(recorder: RtabmapRecorder) -> None:
    assert _codec_of(recorder, "cloud_map", PointCloud2) == "lz4+lcm"


def test_depth_round_trips_exactly(recorder: RtabmapRecorder) -> None:
    depth = np.arange(16 * 16, dtype=np.uint16).reshape(16, 16) * 41
    stream = recorder.store.stream("depth_image", Image)
    stream.append(Image(data=depth, format=ImageFormat.DEPTH16, frame_id="camera", ts=1.0), ts=1.0)

    stored = next(iter(stream)).data
    np.testing.assert_array_equal(stored.data, depth)


def test_image_streams_take_their_pose_from_odometry(recorder: RtabmapRecorder) -> None:
    setters = recorder._collect_pose_setters()
    assert set(setters) == {"rtabmap_odometry", "color_image", "depth_image", "cloud_map"}


def test_cloud_map_is_anchored_at_the_origin(recorder: RtabmapRecorder) -> None:
    """It is already in map coordinates; a robot pose would double-apply."""
    pose = asyncio.run(recorder._cloud_pose(PointCloud2()))
    assert pose is not None
    assert (pose.position.x, pose.position.y, pose.position.z) == (0.0, 0.0, 0.0)
