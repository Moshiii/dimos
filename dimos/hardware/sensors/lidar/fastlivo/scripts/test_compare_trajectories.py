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

import numpy as np
import pytest

from dimos.hardware.sensors.lidar.fastlivo.scripts.compare_trajectories import (
    associate,
    find_time_offset,
    umeyama_align,
)


def _rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@pytest.fixture
def wavy_path() -> np.ndarray:
    t = np.linspace(0.0, 20.0, 400)
    return np.column_stack([t, np.sin(t), 0.1 * t])


def test_umeyama_recovers_known_transform(wavy_path: np.ndarray) -> None:
    rot_true = _rot_z(0.7)
    t_true = np.array([5.0, -2.0, 1.0])
    dst = wavy_path @ rot_true.T + t_true

    rot, t = umeyama_align(wavy_path, dst)

    np.testing.assert_allclose(rot, rot_true, atol=1e-9)
    np.testing.assert_allclose(t, t_true, atol=1e-9)


def test_associate_pairs_nearest_within_tolerance() -> None:
    ref_ts = np.arange(0.0, 10.0, 0.1)
    est_ts = np.array([0.02, 5.03, 7.49, 20.0])  # last has no match

    ref_idx, est_idx = associate(ref_ts, est_ts, max_dt=0.06)

    assert list(est_idx) == [0, 1, 2]
    np.testing.assert_allclose(ref_ts[ref_idx], [0.0, 5.0, 7.5])


def test_find_time_offset_recovers_shift(wavy_path: np.ndarray) -> None:
    ts = np.linspace(0.0, 20.0, 400)
    # Same motion, stamped 100s later, in a rotated/translated frame (offset
    # recovery must not depend on a common spatial frame).
    shift = 100.0
    est_p = wavy_path @ _rot_z(1.1).T + np.array([3.0, 4.0, 5.0])

    offset = find_time_offset(ts, wavy_path, ts + shift, est_p)

    assert offset == pytest.approx(-shift, abs=0.2)
