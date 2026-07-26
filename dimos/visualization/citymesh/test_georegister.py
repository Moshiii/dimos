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

"""Offline tests for the odom-to-ENU track alignment."""

import math

import numpy as np
import pytest

from dimos.visualization.citymesh.georegister import (
    Registration,
    TrackAligner,
    rigid_align_2d,
)

YAW = math.radians(137.0)
T = np.array([250.0, -80.0])


def _enu_of(odom_xy: np.ndarray, yaw: float = YAW, t: np.ndarray = T) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s], [s, c]])
    return (r @ odom_xy.T).T + t


def _square_track(side_m: float = 10.0, step_m: float = 0.5) -> np.ndarray:
    n = int(side_m / step_m)
    up = np.linspace(0, side_m, n)
    pts = (
        [(x, 0.0) for x in up]
        + [(side_m, y) for y in up]
        + [(x, side_m) for x in up[::-1]]
        + [(0.0, y) for y in up[::-1]]
    )
    return np.array(pts)


def test_exact_recovery_on_a_clean_track():
    odom = _square_track()
    yaw, t, rms = rigid_align_2d(odom, _enu_of(odom))
    assert yaw == pytest.approx(YAW, abs=1e-9)
    assert t == pytest.approx(T, abs=1e-6)
    assert rms < 1e-9


def test_recovery_under_gps_noise():
    rng = np.random.default_rng(7)
    odom = _square_track(side_m=15.0)
    enu = _enu_of(odom) + rng.normal(0.0, 2.0, size=odom.shape)
    yaw, t, rms = rigid_align_2d(odom, enu)
    assert abs(math.degrees(yaw - YAW)) < 5.0, "2 m noise on a 15 m track"
    assert np.linalg.norm(t - T) < 2.0
    assert 1.0 < rms < 3.0, "residual should reflect the injected noise"


def test_aligner_pairs_by_interpolation_and_solves():
    aligner = TrackAligner(min_extent_m=8.0, min_pairs=8)
    # 10 Hz odometry along a diagonal, 1 Hz fixes landing between samples.
    for i in range(200):
        ts = i * 0.1
        aligner.add_odom(ts, ts * 1.0, ts * 0.5)
    for ts in np.arange(0.55, 19.0, 1.0):
        e, n = _enu_of(np.array([[ts * 1.0, ts * 0.5]]))[0]
        aligner.add_fix(float(ts), float(e), float(n))
    reg = aligner.solve()
    assert reg is not None
    assert reg.yaw == pytest.approx(YAW, abs=1e-6)
    assert reg.rms_m < 1e-6, "interpolated pairs on a straight line are exact"


def test_underdetermined_tracks_return_none():
    aligner = TrackAligner(min_extent_m=8.0, min_pairs=8)
    # Plenty of pairs, no extent: a parked robot must not produce a yaw.
    for i in range(50):
        aligner.add_odom(i * 0.1, 0.01 * (i % 3), 0.0)
    for i in range(5, 45, 2):
        aligner.add_fix(i * 0.1, 100.0, 200.0)
    assert aligner.solve() is None


def test_fix_without_bracketing_odometry_is_dropped():
    aligner = TrackAligner()
    aligner.add_odom(10.0, 0.0, 0.0)
    aligner.add_odom(11.0, 1.0, 0.0)
    aligner.add_fix(9.0, 0.0, 0.0)  # before the odom window
    aligner.add_fix(12.0, 0.0, 0.0)  # after it
    assert len(aligner._pairs) == 0


def test_odom_from_enu_inverts_the_registration():
    reg = Registration(yaw=YAW, t_e=float(T[0]), t_n=float(T[1]), rms_m=0.0, n_pairs=10)
    yaw_inv, (te, tn) = reg.odom_from_enu()
    # A point through forward then inverse lands where it started.
    p_odom = np.array([3.0, -4.0])
    p_enu = _enu_of(p_odom[None, :])[0]
    c, s = math.cos(yaw_inv), math.sin(yaw_inv)
    back = np.array([[c, -s], [s, c]]) @ p_enu + np.array([te, tn])
    assert back == pytest.approx(p_odom, abs=1e-9)
