#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Mapping-only blueprint for M20 — no control takeover.

Subscribes to ``slam_aligned_points`` and ``slam_odom``, builds a
global voxel map and costmap.  Does NOT include M20Connection, so the
physical remote controller keeps authority.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.robot.deeprobotics.m20.blueprints.basic import rerun
from dimos.robot.deeprobotics.m20.tf import M20TF

voxel_size = 0.05
m20_width_clearance = 0.45
# SLAM odometry is the robot head pose (~1.3 m above bottom).
m20_height_clearance = 0.8
m20_overhead_safety_margin = 0.2
m20_overhead_clearance = m20_height_clearance + m20_overhead_safety_margin
m20_max_step_height = 0.15
m20_safe_radius_margin = 0.1

_ray_tracer = RayTracingVoxelMap.blueprint(
    executable="target/release/voxel_ray_tracing",
    build_command=None,
    voxel_size=voxel_size,
    max_range=8.0,
    shadow_depth=0.1,
    min_health=-1,
    max_health=10,
    emit_every=2,
    ray_subsample=1,
    global_emit_every=1,
    auto_build=False,
    support_min=0,
    registered_clouds=True,
).remappings(
    [
        (RayTracingVoxelMap, "lidar", "slam_aligned_points"),
        (RayTracingVoxelMap, "odometry", "slam_odom"),
    ]
)

m20_map_only = autoconnect(
    rerun,
    M20TF.blueprint().remappings([(M20TF, "odometry", "slam_odom")]),
    _ray_tracer,
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=voxel_size,
            can_pass_under=m20_overhead_clearance,
            can_climb=m20_max_step_height,
            ignore_noise=0.08,
            smoothing=1.5,
            min_gradient_neighbors=2,
            ignore_overhead_only=True,
        ),
        initial_safe_radius_meters=m20_width_clearance + m20_safe_radius_margin,
        inflation_radius_m=m20_width_clearance + m20_safe_radius_margin,
        gradient_distance_m=1.5,
    ),
).global_config(
    n_workers=5,
    robot_model="m20",
    robot_width=(m20_width_clearance + m20_safe_radius_margin) * 2,
)
