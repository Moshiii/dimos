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

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper


@dataclass
class EvalConfig:
    """Harness and gate parameters, sized for the Unitree Go2.

    Algorithm tuning lives in the algorithm packages, not here.
    """

    voxel_size: float = 0.08
    max_range: float = 30.0
    robot_height: float = 0.3

    # Collision-gate body box. Only the ground_margin to body_clearance band
    # is checked, so the legs and the terrain under them never count.
    robot_length: float = 0.7
    robot_width: float = 0.31
    ground_margin: float = 0.25
    body_clearance: float = 0.45
    goal_tolerance: float = 0.5
    align_tol: float = 0.05
    # Ground-support reach. The radius models straddling small scan holes.
    support_radius_m: float = 0.35
    support_depth_m: float = 0.35
    # Climb limits, from the steepest climbs the Go2 demonstrated on stairs.
    max_slope: float = 1.2
    max_step_m: float = 0.2
    kinematic_window_m: float = 0.5
    # How far an endpoint may sit from a standable surface before it is off the map.
    snap_max_m: float = 1.0

    # An improvement must not buy score with compute. p95 over the suite.
    # A plan is timed end to end, so deferred map work is charged here too.
    plan_p95_budget_ms: float = 200.0
    # Per lidar frame, so a pipeline that cannot keep up with the sensor fails
    # regardless of how it scores.
    map_update_p95_budget_ms: float = 100.0

    # Which pipeline is under test, by registry name.
    pipeline: str = "mls"
    # Pipeline constructor overrides, e.g. --set planner.wall_clearance_m=0.0.
    planner: dict[str, float] = field(default_factory=dict)

    def make_mapper(self) -> VoxelRayMapper:
        """The mapper that builds the occupancy every pipeline is graded against."""
        from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper

        return VoxelRayMapper(voxel_size=self.voxel_size, max_range=self.max_range)

    def mapper_fingerprint(self) -> dict[str, float | int]:
        """Cache key parameters for the final map.

        Mapper internals are not fingerprinted, so a mapper change needs
        dimos cache clean rather than a new key here.
        """
        return {"voxel_size": self.voxel_size, "max_range": self.max_range}
