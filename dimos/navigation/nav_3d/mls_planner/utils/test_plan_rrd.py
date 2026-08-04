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

"""Picking the tf stream out of a recording without disturbing it."""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.nav_3d.mls_planner.utils.plan_rrd import _tf_over

if TYPE_CHECKING:
    from pathlib import Path


def _store(tmp_path: Path, tf_stamps: list[float] | None, lidar_stamps: list[float]) -> SqliteStore:
    store = SqliteStore(path=str(tmp_path / "rec.db"))
    lidar = store.stream("pointlio_lidar", str)
    for ts in lidar_stamps:
        lidar.append("cloud", ts=ts)
    if tf_stamps is not None:
        tf = store.stream("tf", TFMessage)
        for ts in tf_stamps:
            tf.append(
                TFMessage(Transform(frame_id="odom", child_frame_id="base_link", ts=ts)), ts=ts
            )
    return store


@pytest.mark.skipif_macos
@pytest.mark.skipif_aarch64
def test_missing_tf_is_not_created_by_looking_for_it(tmp_path: Path) -> None:
    """Probing with ``store.stream`` would register tf and write its tables."""
    with _store(tmp_path, tf_stamps=None, lidar_stamps=[1.0, 2.0]) as store:
        assert _tf_over(store, store.stream("pointlio_lidar", str)) is None
        assert "tf" not in store.list_streams()


@pytest.mark.skipif_macos
@pytest.mark.skipif_aarch64
def test_tf_window_follows_the_lidar_window(tmp_path: Path) -> None:
    """tf starts 10s before the lidar here, so a relative window would be shifted."""
    with _store(
        tmp_path, tf_stamps=[90.0, 100.0, 105.0, 110.0, 115.0], lidar_stamps=[100.0, 110.0]
    ) as store:
        tf = _tf_over(store, store.stream("pointlio_lidar", str))

        assert [obs.ts for obs in tf] == [100.0, 105.0, 110.0]


@pytest.mark.skipif_macos
@pytest.mark.skipif_aarch64
def test_empty_lidar_window_has_no_tf(tmp_path: Path) -> None:
    with _store(tmp_path, tf_stamps=[1.0], lidar_stamps=[1.0]) as store:
        empty = store.stream("pointlio_lidar", str).after(500.0)

        assert _tf_over(store, empty) is None
