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

"""Offline tests for the ENU snap: identity yaw, translation from the median."""

from __future__ import annotations

import pytest

from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.visualization.citymesh.enu_tf import EnuSnap
from dimos.visualization.citymesh.frame import EnuFrame, snap_origin

ATHENS = (37.9838, 23.7275)
OFFSET = (10.0, -5.0, 2.0)  # where tf says the robot is, relative to ENU


def _fix(lat=ATHENS[0], lon=ATHENS[1], alt=30.0):
    return NavSatFix(latitude=lat, longitude=lon, altitude=alt, status=NavSatFix.STATUS_FIX)


def _frame() -> EnuFrame:
    lat0, lon0 = snap_origin(*ATHENS)
    return EnuFrame.at(lat0, lon0, 0.0, datum="msl", undulation=0.0)


def _position_for(fix: NavSatFix, frame: EnuFrame) -> tuple[float, float, float]:
    e, n, u = frame.geodetic_to_enu(fix.latitude, fix.longitude, fix.altitude, datum="msl")[0]
    return (float(e) + OFFSET[0], float(n) + OFFSET[1], float(u) + OFFSET[2])


def test_no_fix_yields_nothing():
    snap = EnuSnap()
    assert snap.add_fix(NavSatFix(status=NavSatFix.STATUS_NO_FIX), (0.0, 0.0, 0.0)) is None


def test_transform_is_the_offset_with_identity_yaw():
    snap = EnuSnap(min_samples=3)
    frame = _frame()
    transform = None
    for i in range(3):
        fix = _fix(lat=ATHENS[0] + i * 1e-5)
        transform = snap.add_fix(fix, _position_for(fix, frame))

    assert transform is not None
    assert transform.frame_id == "world"
    assert transform.child_frame_id == "enu"
    t = transform.translation
    assert (t.x, t.y, t.z) == pytest.approx(OFFSET, abs=1e-6)
    q = transform.rotation
    assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)


def test_needs_min_samples_before_speaking():
    snap = EnuSnap(min_samples=5)
    frame = _frame()
    fix = _fix()
    position = _position_for(fix, frame)
    for _ in range(4):
        assert snap.add_fix(fix, position) is None
    assert snap.add_fix(fix, position) is not None


def test_median_shrugs_off_an_outlier_fix():
    snap = EnuSnap(min_samples=3)
    frame = _frame()
    fix = _fix()
    position = _position_for(fix, frame)
    for _ in range(5):
        snap.add_fix(fix, position)
    # One wild fix, same robot position: a mean would drag the origin metres away.
    transform = snap.add_fix(_fix(lat=ATHENS[0] + 3e-4), position)
    assert transform is not None
    assert (transform.translation.x, transform.translation.y) == pytest.approx(OFFSET[:2], abs=1e-6)


def test_frame_anchors_at_the_snapped_origin():
    snap = EnuSnap()
    snap.add_fix(_fix(), (0.0, 0.0, 0.0))
    assert snap.frame is not None
    assert (snap.frame.lat0, snap.frame.lon0) == snap_origin(*ATHENS)


def test_nan_altitude_measures_z_against_sea_level():
    snap = EnuSnap(min_samples=1)
    transform = snap.add_fix(_fix(alt=float("nan")), (0.0, 0.0, 1.5))
    assert transform is not None
    # ~2 cm of tangent-plane curvature at 500 m from the origin is fine.
    assert transform.translation.z == pytest.approx(1.5, abs=0.05)
