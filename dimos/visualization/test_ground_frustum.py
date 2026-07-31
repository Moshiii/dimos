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

import math
import pickle

import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.visualization.ground_frustum import (
    Config,
    ground_frustum_override,
    ground_range,
)

# Optical convention: +Z forward, +X right, +Y down. Nadir points optical +Z
# at world -Z, which is a 180 deg roll about X off the identity-up camera.
NADIR = Quaternion(1.0, 0.0, 0.0, 0.0)


def _pose(height, tilt_deg=0.0, x=0.0, y=0.0):
    q = NADIR
    if tilt_deg:
        half = math.radians(tilt_deg) / 2
        # tilt about optical X, applied after the nadir flip
        q = q * Quaternion(math.sin(half), 0.0, 0.0, math.cos(half))
    return Transform(translation=Vector3(x, y, height), rotation=q)


def _distance(pose, **config):
    return ground_range(pose, Config(**config))


def test_nadir_reaches_the_ground():
    assert _distance(_pose(120.0)) == 120.0


def test_ground_z_is_the_plane_it_reaches():
    assert _distance(_pose(120.0), ground_z=20.0) == 100.0


def test_tilt_lengthens_the_slant_range():
    assert _distance(_pose(100.0, tilt_deg=20.0)) == pytest.approx(
        100.0 / math.cos(math.radians(20.0))
    )


def test_looking_sideways_falls_back_to_the_short_frustum():
    assert _distance(_pose(100.0, tilt_deg=60.0)) == 1.0


def test_on_the_ground_falls_back_to_the_short_frustum():
    assert _distance(_pose(0.0)) == 1.0


def test_clamped_to_max_distance():
    assert _distance(_pose(9000.0), max_distance_m=500.0) == 500.0


def test_ground_point_stops_at_the_point():
    assert _distance(_pose(120.0), ground_point=(0.0, 0.0, 20.0)) == 100.0


def test_ground_point_holds_while_the_drone_drifts_off_it():
    """Straight down, the plane sits at the point's height wherever the drone is."""
    assert _distance(_pose(120.0, x=50.0), ground_point=(0.0, 0.0, 0.0)) == 120.0


def test_ground_point_shortens_where_the_plane_would_lengthen():
    """Tilted, the point is nearer along the axis than the ground plane is."""
    assert _distance(_pose(100.0, tilt_deg=20.0), ground_point=(0.0, 0.0, 0.0)) == pytest.approx(
        100.0 * math.cos(math.radians(20.0))
    )


def test_ground_point_behind_the_camera_falls_back():
    assert _distance(_pose(100.0), ground_point=(0.0, 0.0, 200.0)) == 1.0


def test_override_renders_a_pinhole_on_the_image_topic():
    info = CameraInfo.from_intrinsics(1000.0, 1000.0, 960.0, 540.0, 1920, 1080, "drone/cam")
    override = ground_frustum_override(info, image_topic="world/video")
    convert = override["world/ground_frustum"]

    ((path, pinhole),) = convert(PointStamped(0.0, 0.0, 120.0, frame_id="drone/cam"))

    assert path == "world/video"
    assert pinhole.image_plane_distance.as_arrow_array().to_pylist() == [120.0]


def test_override_survives_pickling_to_the_bridge_worker():
    """Overrides travel to the worker by pickle — a closure would not arrive."""
    info = CameraInfo.from_intrinsics(1000.0, 1000.0, 960.0, 540.0, 1920, 1080, "drone/cam")

    override = pickle.loads(pickle.dumps(ground_frustum_override(info)))

    ((_, pinhole),) = override["world/ground_frustum"](PointStamped(0.0, 0.0, 77.0))
    assert pinhole.image_plane_distance.as_arrow_array().to_pylist() == [77.0]


def test_the_point_survives_the_typed_transport():
    """The bridge decodes by lcm_type; a topic it cannot decode is dropped."""
    point = PointStamped(0.0, 0.0, 42.0, ts=1.5, frame_id="drone/camera_optical")

    decoded = PointStamped.lcm_decode(point.lcm_encode())

    assert decoded.z == 42.0
    assert decoded.frame_id == "drone/camera_optical"
