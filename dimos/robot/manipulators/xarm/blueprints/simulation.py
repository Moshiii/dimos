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

"""Simulation xArm perception manipulation blueprints."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.point_cloud_self_filter import PointCloudSelfFilter, SelfFilterRegion
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule

_xarm7_sim_hw = make_xarm7_sim_hardware(XARM7_SIM_PATH)

# One resolution for the map and the planning octree: the octree is built
# straight from these points, so a mismatch either inflates every voxel or
# leaves gaps the planner will happily route through.
XARM_VOXEL_PLANNING_RESOLUTION = 0.05

# The wrist camera sees the gripper it is mounted on. Those points are the
# robot, not the scene -- left in, they become an obstacle rigidly attached to
# the end effector and every plan collides at the start state. Anchored to
# link7 and link_tcp, both of which this robot config publishes TF for
# (tf_extra_links=["link7"], end_effector_link="link_tcp").
_XARM_SELF_FILTER_REGIONS = [
    SelfFilterRegion(
        shape="box", frame_id="link7", size=(0.22, 0.22, 0.30), center=(0.0, 0.0, 0.02)
    ),
    SelfFilterRegion(shape="sphere", frame_id="link_tcp", radius=0.16),
]

xarm_perception_sim = autoconnect(
    PickAndPlaceModule.blueprint(
        robots=[make_xarm7_sim_robot_config()],
        planning_timeout=10.0,
        # Live occupancy from the wrist camera becomes an octree obstacle, so
        # the RRT routes around the table and clutter instead of sampling
        # straight through them.
        planning_world_frame="world",
        planning_voxel_resolution=XARM_VOXEL_PLANNING_RESOLUTION,
        # Sim stack runs on the RoboPlan (pinocchio) backend with its native
        # RRT planner; Drake stays the default for hardware blueprints.
        world_backend="roboplan",
        planner_name="roboplan",
        # Viser replaces the retired DrakeWorld meshcat; reachable over an SSH
        # port-forward to 127.0.0.1:8095 (logged as "Visualization:" at startup).
        # "Scan from here" drives the PickAndPlace scan_objects skill; the
        # ground-truth overlay mirrors the MJCF object positions.
        visualization={
            "backend": "viser",
            "scan_tool": "scan_objects",
            "ground_truth_objects": [
                {"name": "apple", "x": 0.40, "y": 0.08, "z": 0.17},
                {"name": "orange", "x": 0.45, "y": -0.08, "z": 0.175},
                {"name": "cup", "x": 0.50, "y": 0.0, "z": 0.19},
            ],
        },
    ),
    MujocoSimModule.blueprint(
        **make_xarm7_sim_module_kwargs(XARM7_SIM_PATH),
        enable_pointcloud=True,
        pointcloud_fps=5.0,
    ),
    ObjectSceneRegistrationModule.blueprint(target_frame="world"),
    PointCloudSelfFilter.blueprint(
        regions=_XARM_SELF_FILTER_REGIONS,
        tf_tolerance_s=0.25,
        # Keep the unfiltered cloud rather than dropping it when TF is late: a
        # brief stall should degrade the self-filter, not blank the obstacle map.
        drop_cloud_on_missing_tf=False,
    ),
    coordinator(
        hardware=[_xarm7_sim_hw],
        tasks=[trajectory_task(_xarm7_sim_hw)],
    ),
    RerunBridgeModule.blueprint(),
).remappings(
    [
        # ObjectSceneRegistrationModule also publishes a stream named
        # "pointcloud", and autoconnect matches on (name, type). Left on the
        # default topic the self-filter would receive an interleaved mix of raw
        # depth and detection-derived points, so the sim's cloud gets its own.
        (MujocoSimModule, "pointcloud", "camera_pointcloud"),
        (PointCloudSelfFilter, "pointcloud", "camera_pointcloud"),
        (PickAndPlaceModule, "planning_voxel_map", "filtered_pointcloud"),
    ]
)
