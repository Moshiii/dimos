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

"""Accumulation, decay, and grasp carve-out behaviour of the planning snapshot."""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from dimos.manipulation.planning.monitor.planning_collision_snapshot import (
    PlanningCollisionSnapshot,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

RESOLUTION = 0.05


class FakeWorldMonitor:
    """Record obstacle lifecycle calls without a planning backend."""

    def __init__(self) -> None:
        self.obstacles: dict[str, Any] = {}
        self.added = 0
        self.updated = 0
        self.removed = 0

    def add_obstacle(self, obstacle: Any) -> str:
        self.added += 1
        obstacle_id = obstacle.name
        self.obstacles[obstacle_id] = obstacle
        return obstacle_id

    def update_obstacle(self, obstacle_id: str, obstacle: Any) -> bool:
        if obstacle_id not in self.obstacles:
            return False
        self.updated += 1
        self.obstacles[obstacle_id] = obstacle
        return True

    def remove_obstacle(self, obstacle_id: str) -> bool:
        self.removed += 1
        return self.obstacles.pop(obstacle_id, None) is not None


def cloud(points: Any, ts: float) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray(points, dtype=np.float64).reshape((-1, 3)), frame_id="world", timestamp=ts
    )


def committed_points(monitor: FakeWorldMonitor) -> Any:
    return np.asarray(monitor.obstacles["planning-collision"].points, dtype=np.float64)


def test_accumulation_keeps_views_the_camera_turned_away_from() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=30.0)

    snapshot.stage(cloud([[0.4, 0.2, 0.3]], 100.0))
    snapshot.synchronize(monitor)
    snapshot.stage(cloud([[0.4, -0.2, 0.3]], 101.0))
    snapshot.synchronize(monitor)

    points = committed_points(monitor)
    assert len(points) == 2
    assert points[:, 1].max() > 0.0
    assert points[:, 1].min() < 0.0
    # One stable obstacle identity, replaced rather than re-registered.
    assert monitor.added == 1
    assert monitor.updated == 1


def test_latest_wins_when_decay_disabled() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=None)

    snapshot.stage(cloud([[0.4, 0.2, 0.3]], 100.0))
    snapshot.synchronize(monitor)
    snapshot.stage(cloud([[0.4, -0.2, 0.3]], 101.0))
    snapshot.synchronize(monitor)

    points = committed_points(monitor)
    assert len(points) == 1
    assert points[0][1] == pytest.approx(-0.2)


def test_stale_voxels_expire_after_decay() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=10.0)

    snapshot.stage(cloud([[0.4, 0.2, 0.3]], 100.0))
    snapshot.synchronize(monitor)
    snapshot.stage(cloud([[0.4, -0.2, 0.3]], 200.0))
    snapshot.synchronize(monitor)

    points = committed_points(monitor)
    assert len(points) == 1
    assert points[0][1] == pytest.approx(-0.2)


def test_repeated_observation_refreshes_a_voxel() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=10.0)

    snapshot.stage(cloud([[0.4, 0.2, 0.3]], 100.0))
    snapshot.synchronize(monitor)
    # Same voxel, seen again later: it must survive its original expiry.
    snapshot.stage(cloud([[0.4, 0.2, 0.3]], 105.0))
    snapshot.synchronize(monitor)
    snapshot.stage(cloud([[0.4, -0.2, 0.3]], 112.0))
    snapshot.synchronize(monitor)

    assert len(committed_points(monitor)) == 2


def test_accumulated_points_are_voxel_centers() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=30.0)

    snapshot.stage(cloud([[0.401, 0.0, 0.0], [0.2, -0.2, 0.0]], 100.0))
    snapshot.synchronize(monitor)

    # The octree centers a box of edge RESOLUTION on each point, so voxels must
    # report centers, not corners. Points already on a boundary (0.2 / 0.05 is
    # 4.000000000000001 in floating point) must not shift a whole cell.
    points = sorted(committed_points(monitor).tolist())
    assert points[0] == pytest.approx([0.2, -0.2, 0.0])
    assert points[1] == pytest.approx([0.4, 0.0, 0.0])


def test_carveout_empties_a_sphere_and_is_reversible() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=30.0)
    grid = [[0.4, y, 0.3] for y in np.arange(-0.2, 0.2, RESOLUTION)]
    snapshot.stage(cloud(grid, 100.0))
    snapshot.synchronize(monitor)
    full = len(committed_points(monitor))

    snapshot.set_grasp_carveout("target", (0.4, 0.0, 0.3), 0.1)
    snapshot.synchronize(monitor)
    carved = committed_points(monitor)
    assert len(carved) < full
    assert np.linalg.norm(carved - np.array([0.4, 0.0, 0.3]), axis=1).min() > 0.1

    snapshot.clear_grasp_carveout("target")
    snapshot.synchronize(monitor)
    assert len(committed_points(monitor)) == full


def test_carveout_applies_to_an_already_staged_cloud() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=30.0)
    snapshot.stage(cloud([[0.4, 0.0, 0.3], [0.4, 0.3, 0.3]], 100.0))
    snapshot.synchronize(monitor)

    # Set after staging and after a commit: the next synchronize must still
    # rebuild, otherwise a grasp request never clears its own target.
    snapshot.set_grasp_carveout("target", (0.4, 0.0, 0.3), 0.1)
    snapshot.synchronize(monitor)

    assert len(committed_points(monitor)) == 1


def test_carveout_emptying_the_map_removes_the_obstacle() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=30.0)
    snapshot.stage(cloud([[0.4, 0.0, 0.3]], 100.0))
    snapshot.synchronize(monitor)

    snapshot.set_grasp_carveout("target", (0.4, 0.0, 0.3), 0.5)
    snapshot.synchronize(monitor)

    assert "planning-collision" not in monitor.obstacles
    assert monitor.removed == 1


def test_wrong_frame_is_rejected() -> None:
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world")
    with pytest.raises(ValueError, match="does not match"):
        snapshot.stage(
            PointCloud2.from_numpy(
                np.zeros((1, 3), dtype=np.float64), frame_id="camera", timestamp=1.0
            )
        )


def test_non_finite_points_are_dropped() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=30.0)
    snapshot.stage(cloud([[0.4, 0.0, 0.3], [np.nan, 0.0, 0.0], [np.inf, 0.0, 0.0]], 100.0))
    snapshot.synchronize(monitor)

    points = committed_points(monitor)
    assert len(points) == 1
    assert np.all(np.isfinite(points))


def test_reset_accumulated_clears_the_map() -> None:
    monitor = FakeWorldMonitor()
    snapshot = PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=30.0)
    snapshot.stage(cloud([[0.4, 0.0, 0.3]], 100.0))
    snapshot.synchronize(monitor)

    snapshot.reset_accumulated()
    snapshot.stage(cloud([[0.4, 0.3, 0.3]], 101.0))
    snapshot.synchronize(monitor)

    assert len(committed_points(monitor)) == 1


def test_invalid_decay_rejected() -> None:
    with pytest.raises(ValueError, match="decay_s"):
        PlanningCollisionSnapshot(RESOLUTION, "world", decay_s=0.0)
