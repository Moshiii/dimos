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

"""Staging and obstacle-lifecycle synchronization for planning voxel snapshots."""

from __future__ import annotations

from collections.abc import Sequence
import threading
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_PACK_BITS = 21
"""Bits per axis in a packed voxel key: +-1e6 cells, i.e. +-52km at 5cm."""

_PACK_OFFSET = 1 << (_PACK_BITS - 1)
_PACK_MASK = (1 << _PACK_BITS) - 1


def _pack(keys: NDArray[np.int64]) -> NDArray[np.int64]:
    """Pack integer xyz voxel keys into one int64 each.

    Deduplicating on a 1-D key is far cheaper than np.unique(axis=0), which
    lexsorts the full Nx3 array on every incoming cloud.
    """
    shifted = np.clip(keys + _PACK_OFFSET, 0, _PACK_MASK)
    return shifted[:, 0] | (shifted[:, 1] << _PACK_BITS) | (shifted[:, 2] << (2 * _PACK_BITS))


def _unpack(packed: NDArray[np.int64]) -> NDArray[np.int64]:
    """Invert :func:`_pack`."""
    return (
        np.stack(
            [
                packed & _PACK_MASK,
                (packed >> _PACK_BITS) & _PACK_MASK,
                (packed >> (2 * _PACK_BITS)) & _PACK_MASK,
            ],
            axis=1,
        )
        - _PACK_OFFSET
    )


class PlanningCollisionSnapshot:
    """Keep the latest planning cloud and commit it through a WorldMonitor."""

    def __init__(
        self,
        resolution: float = 0.05,
        planning_frame: str = "world",
        decay_s: float | None = None,
    ) -> None:
        if resolution <= 0:
            raise ValueError("Planning collision snapshot resolution must be positive")
        if not planning_frame:
            raise ValueError("Planning collision snapshot planning_frame must not be empty")
        if decay_s is not None and decay_s <= 0:
            raise ValueError("Planning collision snapshot decay_s must be positive or None")
        self.resolution = resolution
        self.planning_frame = planning_frame
        self.decay_s = decay_s
        # Parallel arrays of packed voxel keys and their last observation
        # time. Only populated when accumulating.
        self._keys: NDArray[np.int64] = np.empty(0, dtype=np.int64)
        self._seen: NDArray[np.float64] = np.empty(0, dtype=np.float64)
        self._lock = threading.RLock()
        self._staged: PointCloud2 | None = None
        self._committed: PointCloud2 | None = None
        self._staged_generation = 0
        self._committed_generation = 0
        self._active_obstacle_id: str | None = None
        self._grasp_carveouts: dict[str, tuple[NDArray[np.float64], float]] = {}

    def stage(self, cloud: PointCloud2) -> None:
        """Validate and stage a complete cloud, replacing older staged input."""
        if cloud.frame_id != self.planning_frame:
            raise ValueError(
                f"Planning collision snapshot frame '{cloud.frame_id}' does not match "
                f"planning frame '{self.planning_frame}'"
            )
        points, _ = cloud.as_numpy()
        points = np.asarray(points, dtype=np.float64)
        points = points[np.all(np.isfinite(points), axis=1)]
        with self._lock:
            if self.decay_s is not None:
                points = self._accumulate(points, cloud.ts)
            staged = PointCloud2.from_numpy(
                points.copy(),
                frame_id=cloud.frame_id,
                timestamp=cloud.ts,
            )
            self._staged = staged
            self._staged_generation += 1

    def _accumulate(self, points: NDArray[np.float64], ts: float | None) -> NDArray[np.float64]:
        """Merge a view into the running occupancy and return the whole map.

        A wrist camera sees only a slice of the scene at a time. Committing just
        the latest view would delete every obstacle the arm has turned away
        from, so views are merged on a voxel grid and expire on a timer instead
        -- without raycast clearing, that timer is the only thing that lets an
        object that has moved stop being an obstacle.
        """
        assert self.decay_s is not None
        now = float(ts if ts is not None else 0.0)
        if len(points):
            # Round rather than floor, so a voxel's representative point is its
            # center. Flooring both keys a point on a cell boundary into
            # whichever cell floating-point error picks (0.2 / 0.05 is
            # 4.000000000000001) and then reports a corner, which offsets the
            # octree box by up to a full voxel from the point that created it.
            fresh = np.unique(_pack(np.rint(points / self.resolution).astype(np.int64)))
            keys = np.concatenate([self._keys, fresh])
            seen = np.concatenate([self._seen, np.full(len(fresh), now)])
        else:
            keys, seen = self._keys, self._seen
        if len(keys):
            # Everything here is vectorized on a packed 1-D key: a per-point
            # Python loop, or even np.unique(axis=0)'s lexsort, runs over every
            # point of every cloud at camera rate and starves the simulator.
            keys, inverse = np.unique(keys, return_inverse=True)
            newest = np.zeros(len(keys), dtype=np.float64)
            np.maximum.at(newest, inverse.ravel(), seen)
            alive = newest >= now - self.decay_s
            keys, seen = keys[alive], newest[alive]
        self._keys, self._seen = keys, seen
        if not len(keys):
            return np.empty((0, 3), dtype=np.float64)
        return _unpack(keys).astype(np.float64) * self.resolution

    def reset_accumulated(self) -> None:
        """Forget accumulated occupancy, e.g. after the scene is rearranged."""
        with self._lock:
            self._keys = np.empty(0, dtype=np.int64)
            self._seen = np.empty(0, dtype=np.float64)
            self._staged_generation += 1

    def set_grasp_carveout(self, key: str, center: Sequence[float], radius: float) -> None:
        """Exclude a sphere of the cloud from planning collision.

        The object we are reaching for is itself part of the occupancy cloud, so
        with the raw cloud committed every grasp pose is in collision and the
        planner can never approach. Carving a sphere around the target restores
        an approach corridor while the rest of the scene stays occupied.

        The carve-out is applied at commit time, so callers can set it after a
        cloud has already been staged and the next ``synchronize`` picks it up.
        """
        if radius <= 0:
            raise ValueError("Grasp carve-out radius must be positive")
        point = np.asarray(center, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(point)):
            raise ValueError("Grasp carve-out center must be finite")
        with self._lock:
            self._grasp_carveouts[key] = (point, float(radius))
            # Force the next synchronize to rebuild geometry even if no new
            # cloud arrived; the committed octree is now stale.
            self._staged_generation += 1

    def clear_grasp_carveout(self, key: str | None = None) -> None:
        """Drop one carve-out, or every carve-out when ``key`` is None."""
        with self._lock:
            if key is None:
                if not self._grasp_carveouts:
                    return
                self._grasp_carveouts.clear()
            elif self._grasp_carveouts.pop(key, None) is None:
                return
            self._staged_generation += 1

    def _apply_grasp_carveouts(self, points: NDArray[np.float64]) -> NDArray[np.float64]:
        """Drop points inside any active carve-out sphere."""
        if not self._grasp_carveouts or len(points) == 0:
            return points
        keep = np.ones(len(points), dtype=bool)
        for center, radius in self._grasp_carveouts.values():
            keep &= np.sum((points - center) ** 2, axis=1) > radius * radius
        return points[keep]

    def committed(self) -> PointCloud2 | None:
        """Return the fully committed cloud, or None before first commit."""
        with self._lock:
            return self._committed

    def staged(self) -> PointCloud2 | None:
        """Return the latest validated input, including uncommitted input."""
        with self._lock:
            return self._staged

    def synchronize(self, world_monitor: WorldMonitor) -> PointCloud2 | None:
        """Commit the latest staged cloud through stable-ID add/update/remove methods."""
        with self._lock:
            if self._staged is None or self._staged_generation == self._committed_generation:
                return self.committed()

            staged = self._staged
            generation = self._staged_generation
            points, _ = staged.as_numpy()
            points = np.asarray(points, dtype=np.float64).copy()
            points = self._apply_grasp_carveouts(points)

            if len(points) == 0:
                if self._active_obstacle_id is not None:
                    removed = world_monitor.remove_obstacle(self._active_obstacle_id)
                    if not removed:
                        raise RuntimeError(
                            "Failed to remove planning collision obstacle "
                            f"'{self._active_obstacle_id}'"
                        )
                self._active_obstacle_id = None
                self._committed = staged
                self._committed_generation = generation
                return self.committed()

            obstacle = Obstacle(
                name="planning-collision",
                obstacle_type=ObstacleType.OCTREE,
                pose=PoseStamped(frame_id=self.planning_frame),
                points=points,
                octree_resolution=self.resolution,
            )
            if self._active_obstacle_id is None:
                active_obstacle_id = world_monitor.add_obstacle(obstacle)
                if not active_obstacle_id:
                    raise RuntimeError("Failed to register planning collision obstacle")
                self._active_obstacle_id = active_obstacle_id
            elif not world_monitor.update_obstacle(self._active_obstacle_id, obstacle):
                raise RuntimeError(
                    f"Planning collision obstacle '{self._active_obstacle_id}' is missing"
                )
            self._committed = staged
            self._committed_generation = generation
            return self.committed()
