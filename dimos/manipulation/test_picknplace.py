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

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import _resolve_single_ref
from dimos.core.module import ModuleBase
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.blueprints import (
    _picknplace_xarm6_model,
    _xarm_graspgenx,
    picknplace,
    picknplace_graspgenx,
    picknplace_graspgenx_edgetam,
)
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.grasping.grasp_proposal import GraspProposalInput
from dimos.manipulation.grasping.grasp_selection import FeasibleGrasp, GraspSelectionResult
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.picknplace import PickNPlaceConfig, PickNPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.xarm.grasp_config import XARM_TCP_TO_GRASP_FRAME


@pytest.mark.parametrize(
    ("blueprint", "uses_graspgenx"),
    [
        (picknplace, False),
        (picknplace_graspgenx, True),
        (picknplace_graspgenx_edgetam, True),
    ],
)
def test_picknplace_blueprints_wire_realsense_rgbd_to_scene_registration(
    blueprint: Blueprint, uses_graspgenx: bool
) -> None:
    atoms = blueprint.blueprints
    modules = [atom.module for atom in atoms]

    assert modules.count(RealSenseCamera) == 1
    assert modules.count(ObjectSceneRegistrationModule) == 1
    assert modules.count(ManipulationModule) == 1
    assert modules.count(PickNPlaceModule) == 1
    assert modules.count(GraspGenXModule) == int(uses_graspgenx)

    camera = next(atom for atom in atoms if atom.module is RealSenseCamera)
    scene = next(atom for atom in atoms if atom.module is ObjectSceneRegistrationModule)
    pick = next(atom for atom in atoms if atom.module is PickNPlaceModule)
    manipulation_ref = next(ref for ref in pick.module_refs if ref.name == "_manipulation")
    assert (
        _resolve_single_ref(pick, manipulation_ref, manipulation_ref.spec, blueprint, set())
        == ManipulationModule.name
    )
    assert camera.kwargs["enable_depth"] is True
    assert camera.kwargs["align_depth_to_color"] is True
    assert camera.kwargs["base_frame_id"] == "link6"
    for stream_name in ("color_image", "depth_image", "camera_info"):
        assert any(
            stream.name == stream_name and stream.direction == "out" for stream in camera.streams
        )
        assert any(
            stream.name == stream_name and stream.direction == "in" for stream in scene.streams
        )


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


def test_picknplace_uses_first_motion_feasible_graspgenx_candidate() -> None:
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
    obj.pointcloud.points_f32.return_value = np.asarray([[0.4, 0.5, 0.6]], dtype=np.float32)
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
    pre_grasp = Pose(Vector3(0.2, 0.3, 0.5), second_candidate.pose.orientation)
    retreat = Pose(Vector3(0.2, 0.3, 0.5), second_candidate.pose.orientation)
    module._manipulation = MagicMock()
    module._manipulation.select_first_feasible_grasp.return_value = GraspSelectionResult(
        FeasibleGrasp(second_candidate, 2, pre_grasp, retreat),
        {"pre_grasp_ik_infeasible": 1},
        2,
    )
    module.graspgenx_candidates = MagicMock()
    module._visualization = MagicMock()

    goal = module.get_goal_pose(1)

    assert goal is not None
    assert goal.ts == 2.0
    assert goal.frame_id == "link_base"
    assert goal.position == second_candidate.pose.position
    assert goal.orientation == second_candidate.pose.orientation
    assert module.get_grasp_candidates().candidates == [candidate, second_candidate]
    assert module.get_grasp_candidates().selected_index == 1
    module.graspgenx_candidates.publish.assert_called_once_with(module.get_grasp_candidates())
    proposal_input = module._grasp_generator.propose_grasps.call_args.args[0]
    assert isinstance(proposal_input, GraspProposalInput)
    assert proposal_input.object_pointcloud is obj.pointcloud
    module._manipulation.select_first_feasible_grasp.assert_called_once_with(
        module.get_grasp_candidates(), "arm", 0.1, 0.1
    )
    pre_grasp = module.get_pre_grasp_pose()
    assert pre_grasp is not None
    assert pre_grasp.position == Vector3(0.2, 0.3, 0.5)
    layer = module._visualization.set_visualization_layer.call_args.args[0]
    assert layer.id == "picknplace/selection"
    assert layer.elements[0].points.shape[1] == 3
    assert layer.elements[1].line_width is None


def test_picknplace_returns_empty_candidates_for_obb_grasps() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig()

    assert module.get_grasp_candidates().candidates == []


def test_failed_target_selection_clears_previous_goal() -> None:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PickNPlaceModule()
    module.config = PickNPlaceConfig()
    module._goal_pose = PoseStamped(position=Vector3(0.1, 0.2, 0.3), orientation=Quaternion())
    module._pre_grasp_pose = PoseStamped(position=Vector3(0.1, 0.2, 0.4), orientation=Quaternion())

    assert module.get_goal_pose(1) is None
    assert module.get_pre_grasp_pose() is None
