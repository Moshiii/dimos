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

"""Offline tests for the ENU snap: one (fix, pose) pair, identity yaw."""

from __future__ import annotations

import pytest

from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.visualization.citymesh.enu_tf import snap_transform
from dimos.visualization.citymesh.frame import EnuFrame, snap_origin

ATHENS = (37.9838, 23.7275)
OFFSET = (10.0, -5.0, 2.0)  # where tf says the robot is, relative to ENU


def _fix(lat=ATHENS[0], lon=ATHENS[1], alt=30.0):
    return NavSatFix(latitude=lat, longitude=lon, altitude=alt, status=NavSatFix.STATUS_FIX)


def _position_for(fix: NavSatFix) -> tuple[float, float, float]:
    lat0, lon0 = snap_origin(*ATHENS)
    frame = EnuFrame.at(lat0, lon0, 0.0, datum="msl", undulation=0.0)
    e, n, u = frame.geodetic_to_enu(fix.latitude, fix.longitude, fix.altitude, datum="msl")[0]
    return (float(e) + OFFSET[0], float(n) + OFFSET[1], float(u) + OFFSET[2])


def test_no_fix_yields_nothing():
    assert snap_transform(NavSatFix(status=NavSatFix.STATUS_NO_FIX), (0.0, 0.0, 0.0)) is None


def test_transform_is_the_offset_with_identity_yaw():
    fix = _fix()
    transform = snap_transform(fix, _position_for(fix))
    assert transform is not None
    assert transform.frame_id == "world"
    assert transform.child_frame_id == "enu"
    t = transform.translation
    assert (t.x, t.y, t.z) == pytest.approx(OFFSET, abs=1e-6)
    q = transform.rotation
    assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)


def test_frames_agree_across_nearby_fixes():
    """Any fix in the snap cell must pin the same ENU origin CityMeshModule uses."""
    a = snap_transform(_fix(), _position_for(_fix()))
    shifted = _fix(lat=ATHENS[0] + 1e-4, lon=ATHENS[1] - 1e-4)
    b = snap_transform(shifted, _position_for(shifted))
    assert a is not None and b is not None
    assert (a.translation.x, a.translation.y) == pytest.approx(
        (b.translation.x, b.translation.y), abs=1e-6
    )


def test_nan_altitude_measures_z_against_sea_level():
    transform = snap_transform(_fix(alt=float("nan")), (0.0, 0.0, 1.5))
    assert transform is not None
    # ~2 cm of tangent-plane curvature at 500 m from the origin is fine.
    assert transform.translation.z == pytest.approx(1.5, abs=0.05)
