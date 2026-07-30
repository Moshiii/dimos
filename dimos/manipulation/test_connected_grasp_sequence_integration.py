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

"""Guarded GraspGenX-to-RoboPlan connected-sequence integration coverage."""

from __future__ import annotations

import importlib.util
import os

import numpy as np
import pytest
import torch

from dimos.manipulation.demo_graspgenx.fixture import load_demo_clouds
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.xarm.config import make_xarm7_sim_robot_config
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config

pytestmark = pytest.mark.self_hosted_large


def _require_real_graspgenx_environment() -> None:
    if os.environ.get("DIMOS_RUN_GRASPGENX_INTEGRATION") != "1":
        pytest.skip("set DIMOS_RUN_GRASPGENX_INTEGRATION=1 to run model integration")
    if importlib.util.find_spec("graspgenx") is None:
        pytest.skip("graspgenx optional dependency is not installed")
    if not torch.cuda.is_available():
        pytest.skip("GraspGenX integration requires CUDA")


@pytest.fixture(scope="module")
def recorded_grasp_proposals() -> tuple[np.ndarray, GraspCandidateArray]:
    """Run the real model once against the repository's recorded object cloud."""
    _require_real_graspgenx_environment()
    try:
        _, object_cloud = load_demo_clouds()
    except FileNotFoundError:
        pytest.skip("recorded graspgenx_ycb_banana_scene data is unavailable")

    config = make_xarm_graspgenx_config()
    module = GraspGenXModule(**config.model_dump(exclude={"rpc_transport", "tf_transport", "g"}))
    try:
        module.start()
        proposals = module.propose_grasps(object_cloud)
    finally:
        module.stop()
    return object_cloud.points_f32(), proposals


def test_real_graspgenx_proposals_preserve_recorded_cloud_contract(
    recorded_grasp_proposals: tuple[np.ndarray, GraspCandidateArray],
) -> None:
    """Smoke-test structural invariants without pinning stochastic poses."""
    points, proposals = recorded_grasp_proposals

    assert points.shape == (3500, 3)
    assert proposals.header.frame_id == "world"
    assert proposals.candidates
    scores = np.asarray([candidate.score for candidate in proposals.candidates])
    assert np.all(np.isfinite(scores))
    assert np.all(scores[:-1] >= scores[1:])
    for candidate in proposals.candidates:
        pose = candidate.pose
        values = np.asarray(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        assert np.all(np.isfinite(values))
        assert np.linalg.norm(values[3:]) == pytest.approx(1.0, abs=1e-5)


def test_real_graspgenx_has_a_connected_sequence_in_open_roboplan_scene(
    recorded_grasp_proposals: tuple[np.ndarray, GraspCandidateArray],
) -> None:
    """Relocate recorded proposals into xArm workspace and require one full path."""
    points, proposals = recorded_grasp_proposals
    robot_config = make_xarm7_sim_robot_config()
    module = PickAndPlaceModule(
        robots=[robot_config],
        planning_timeout=10.0,
        visualization={"backend": "none"},
        floor_z=None,
    )
    module.coordinator_joint_state = None
    module.objects = None
    try:
        module.start()
        home = robot_config.home_joints
        assert home is not None
        module._on_joint_state(
            JointState(
                name=list(robot_config.get_coordinator_joint_names()),
                position=list(home),
            )
        )
        source_center = np.mean(points, axis=0)
        workspace_center = np.asarray([0.45, 0.0, 0.25])
        translation = workspace_center - source_center
        approach = Vector3(0.0, 0.0, -1.0)

        feasible: GraspCandidate | None = None
        for candidate in proposals.candidates[:20]:
            pose = candidate.pose
            relocated = Pose(
                Vector3(
                    pose.position.x + translation[0],
                    pose.position.y + translation[1],
                    pose.position.z + translation[2],
                ),
                Quaternion(pose.orientation),
            )
            pre_grasp = module._compute_pre_grasp_pose(relocated, 0.05, approach)
            retreat = module._compute_pre_grasp_pose(relocated, 0.05, approach)
            failed_index, _ = module._check_connected_pose_sequence(
                (pre_grasp, relocated, retreat),
                "arm",
            )
            if failed_index is None:
                feasible = candidate
                break

        assert feasible is not None
    finally:
        module.stop()
