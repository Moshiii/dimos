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

import math
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from dimos.core.module import ModuleBase
from dimos.manipulation.blueprints import _picknplace_xarm6_model, _xarm_graspgenx
from dimos.manipulation.picknplace import PickNPlaceConfig, PickNPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header
from dimos.robot.manipulators.xarm.grasp_config import XARM_TCP_TO_GRASP_FRAME


def test_picknplace_scans_and_selects_target() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig()
    scene = MagicMock()
    detections = MagicMock()
    module._scene = scene
    obj = MagicMock(
        ts=1.0,
        frame_id="link_base",
        center=Vector3(0.1, 0.2, 0.04),
        confidence=0.9,
    )
    obj.name = "cup"
    obj.size = Vector3(0.4, 0.1, 0.2)
    obj.camera_transform = None
    obj.image = None
    obj.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    scene.scan_scene.side_effect = lambda: (module._on_objects([obj]), detections)[1]

    with patch("dimos.manipulation.picknplace.to_detection3d_array") as to_detection3d_array:
        result = MagicMock()
        to_detection3d_array.return_value = result
        assert module.scan_scene() is result
        to_detection3d_array.assert_called_once_with([obj], frame_id="link_base", ts=1.0)

    assert module.get_scene_info() == [{"number": 1, "name": "cup", "confidence": 0.9}]
    goal = module.get_goal_pose(1)
    assert goal is not None
    assert goal.position == Vector3(0.1, 0.2, 0.100)
    assert goal.orientation == Quaternion.from_euler(Vector3(-3.141592653589793, 0.0, 0.0))
    pre_grasp = module.get_pre_grasp_pose()
    assert pre_grasp is not None
    assert pre_grasp.position == Vector3(0.1, 0.2, 0.200)

    module.scan_scene("water bottle")
    scene.set_prompts.assert_called_once_with(["water bottle"])

    module.config = PickNPlaceConfig(align_grasp_yaw=True)
    yaw_aligned_goal = module.get_goal_pose(1)
    assert yaw_aligned_goal is not None
    expected = Quaternion.from_euler(Vector3(-math.pi, 0.0, 0.0))
    assert yaw_aligned_goal.orientation.angle_to(expected) == pytest.approx(0.0)


def test_picknplace_home_matches_xarm_lifecycle_home() -> None:
    assert _picknplace_xarm6_model.home_joints == [
        0.0,
        math.radians(-40.0),
        math.radians(-50.0),
        0.0,
        math.radians(90.0),
        0.0,
    ]


def test_picknplace_graspgenx_uses_xarm_tcp_calibration() -> None:
    assert _xarm_graspgenx.grasp_frame_to_tcp[2][3] == pytest.approx(0.172)
    assert _xarm_graspgenx.grasp_frame_to_tcp[:2] == ((0.0, -1.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    assert np.allclose(
        np.asarray(_xarm_graspgenx.grasp_frame_to_tcp) @ np.asarray(XARM_TCP_TO_GRASP_FRAME),
        np.eye(4),
    )


def test_picknplace_yaw_alignment_defaults_to_disabled() -> None:
    assert not PickNPlaceConfig().align_grasp_yaw


def test_picknplace_uses_top_graspgenx_candidate() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig(grasp_strategy="graspgenx")
    obj = MagicMock(
        ts=1.0,
        frame_id="link_base",
        center=Vector3(0.1, 0.2, 0.3),
        pointcloud=MagicMock(),
    )
    obj.camera_transform = None
    obj.image = None
    obj.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    module._latest_objects = (obj,)
    candidate = GraspCandidate(
        Pose(
            Vector3(0.4, 0.5, 0.6),
            Quaternion.from_euler(Vector3(0.0, math.pi / 2.0, 0.0)),
        ),
        score=0.9,
    )
    second_candidate = GraspCandidate(
        Pose(Vector3(0.2, 0.3, 0.4), Quaternion()),
        score=0.8,
    )
    module._grasp_generator = MagicMock(
        propose_grasps=MagicMock(
            return_value=GraspCandidateArray(
                Header(2.0, "link_base"), [candidate, second_candidate]
            )
        )
    )
    module.graspgenx_candidates = MagicMock()

    goal = module.get_goal_pose(1)

    assert goal is not None
    assert goal.ts == 2.0
    assert goal.frame_id == "link_base"
    assert goal.position == candidate.pose.position
    assert goal.orientation == candidate.pose.orientation
    assert module.get_grasp_candidates().candidates == [candidate, second_candidate]
    module.graspgenx_candidates.publish.assert_called_once_with(module.get_grasp_candidates())
    pre_grasp = module.get_pre_grasp_pose()
    assert pre_grasp is not None
    assert pre_grasp.position.x == pytest.approx(goal.position.x - 0.1)
    assert pre_grasp.position.z == pytest.approx(goal.position.z)
    selected_goal = module.select_grasp_candidate(1)
    assert selected_goal is not None
    assert selected_goal.position == second_candidate.pose.position
    assert module.get_grasp_candidates().selected_index == 1
    pre_grasp = module.get_pre_grasp_pose()
    assert pre_grasp is not None
    assert pre_grasp.position.x == pytest.approx(selected_goal.position.x)
    assert pre_grasp.position.z == pytest.approx(selected_goal.position.z - 0.1)


def test_picknplace_returns_empty_candidates_for_obb_grasps() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig()

    assert module.get_grasp_candidates().candidates == []
