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

"""Unit tests for PickAndPlaceModule pure logic (no Drake required)."""

from __future__ import annotations

from collections import Counter
from contextlib import nullcontext
import json
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import open3d as o3d
import pytest
from pytest_mock import MockerFixture

from dimos.agents.skill_result import SkillResult
from dimos.core.coordination.blueprints import BlueprintAtom, autoconnect
from dimos.core.coordination.module_coordinator import _resolve_single_ref
from dimos.core.module import ModuleBase
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.grasping.grasp_proposal import GraspProposalInput
from dimos.manipulation.manipulation_module import (
    ConnectedPoseIKResult,
    ConnectedPoseSequenceResult,
)
from dimos.manipulation.pick_and_place_module import (
    GraspVerificationConfig,
    GraspVisualizationConfig,
    PickAndPlaceModule,
    PickAndPlaceModuleConfig,
    PromptedPickAndPlaceModule,
    _FeasibleGrasp,
    _GraspVerification,
    _PickTransaction,
)
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.manipulation.visualization.layers import LineSetElement, VisualizationLayer
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.perception.experimental.object import Object as DetObject
from dimos.perception.experimental.object_scene_registration import (
    ObjectSceneRegistrationModule,
)


def _make_det_object(
    name: str = "cup",
    object_id: str = "abc12345",
    center: tuple[float, float, float] = (0.5, 0.0, 0.3),
    size: tuple[float, float, float] = (0.05, 0.05, 0.10),
) -> DetObject:
    """Create a DetObject with the given attributes and sensible defaults."""
    return DetObject(
        name=name,
        object_id=object_id,
        center=Vector3(x=center[0], y=center[1], z=center[2]),
        size=Vector3(x=size[0], y=size[1], z=size[2]),
        pose=PoseStamped(),
        pointcloud=PointCloud2(o3d.geometry.PointCloud()),
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=0,
        class_id=0,
        confidence=1.0,
        ts=0.0,
        image=Image(),
    )


def _ik_result(
    *positions: float,
    failed_index: int | None = None,
    start: float = 0.0,
) -> ConnectedPoseIKResult:
    return ConnectedPoseIKResult(
        failed_index=failed_index,
        start=JointState(name=["arm/joint1"], position=[start]),
        joint_states=tuple(
            JointState(name=["arm/joint1"], position=[position]) for position in positions
        ),
    )


def _plan_result(*positions: float, failed_index: int | None = None) -> ConnectedPoseSequenceResult:
    path = tuple(JointState(name=["arm/joint1"], position=[position]) for position in positions)
    return ConnectedPoseSequenceResult(
        failed_index=failed_index,
        endpoint=path[-1] if failed_index is None and path else None,
        paths=(path,) if path else (),
    )


@pytest.fixture
def module() -> PickAndPlaceModule:
    """Create a PickAndPlaceModule with heavy base init (RPC, config) patched out."""
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        result = PickAndPlaceModule()
    result.config = PickAndPlaceModuleConfig()
    return result


@pytest.fixture
def prompted_module() -> PromptedPickAndPlaceModule:
    """Create the prompted provider variant without initializing RPC boundaries."""
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        result = PromptedPickAndPlaceModule()
    result.config = PickAndPlaceModuleConfig()
    return result


class TestFindObjectInDetections:
    """Test object lookup logic in detection snapshot."""

    def test_find_by_exact_name(self, module):
        det = _make_det_object(name="cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_partial_name(self, module):
        det = _make_det_object(name="red cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_object_id(self, module):
        det = _make_det_object(object_id="abc12345")
        module._detection_snapshot = [det]

        # Truncated prefix match
        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is det

    def test_find_by_object_id_ambiguous_returns_none(self, module):
        det1 = _make_det_object(object_id="abc12345")
        det2 = _make_det_object(object_id="abc19999")
        module._detection_snapshot = [det1, det2]

        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is None

    def test_find_missing_returns_none(self, module):
        module._detection_snapshot = [_make_det_object(name="bottle")]

        result = module._find_object_in_detections("keyboard")
        assert result is None

    def test_find_by_name_requires_unique_match(self, module):
        module._detection_snapshot = [
            _make_det_object(name="cup", object_id="first"),
            _make_det_object(name="red cup", object_id="second"),
        ]

        assert module._find_object_in_detections("cup") is None

    def test_empty_snapshot_returns_none(self, module):
        module._detection_snapshot = []

        result = module._find_object_in_detections("cup")
        assert result is None


class TestGraspHeuristics:
    """Test the shared grasp/place orientation wrapper."""

    def test_grasp_orientation_near_is_top_down(self):
        q = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        # Near object: pitch = 180° (top-down), tilt = 0, yaw = 0
        # RPY(0, π, 0) → quaternion (x=0, y=1, z=0, w=0)
        assert abs(q.x) < 0.01
        assert abs(q.y - 1.0) < 0.01
        assert abs(q.z) < 0.01
        assert abs(q.w) < 0.01

    def test_grasp_orientation_far_differs_from_near(self):
        q_near = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        q_far = PickAndPlaceModule._grasp_orientation(gx=1.0, gy=0.0, xy_dist=1.0)
        # Far object should have different orientation (tilted)
        assert not (
            abs(q_near.x - q_far.x) < 0.01
            and abs(q_near.y - q_far.y) < 0.01
            and abs(q_near.z - q_far.z) < 0.01
            and abs(q_near.w - q_far.w) < 0.01
        )


class TestPlaceBack:
    """Test place_back guard logic."""

    def test_place_back_no_pick_pose_errors(self, module):
        module._last_pick_pose = None

        result = module.place_back()
        assert not result.is_success()
        assert result.error_code == "NO_PRIOR_POSE"
        assert "pick" in result.message.lower()


def test_grasp_pipeline_error_agent_encoding_is_structured() -> None:
    result = SkillResult[ManipulationSkillError].fail("PICK_BUSY", "pick in progress")

    payload = json.loads(result.agent_encode()[0]["text"])

    assert payload == {
        "success": False,
        "message": "pick in progress",
        "error_code": "PICK_BUSY",
        "duration_ms": 0.0,
    }


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"planning_frame": " "}, "planning_frame"),
        ({"grasp_approach_vector": (0.0, 0.0, 2.0)}, "unit vector"),
        (
            {
                "grasp_verification": {
                    "open_position": 0.85,
                    "closed_position": 0.0,
                    "held_threshold": 0.9,
                }
            },
            "held_threshold",
        ),
    ],
)
def test_grasp_pipeline_config_rejects_invalid_values(
    kwargs: dict[str, object], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        PickAndPlaceModuleConfig(**kwargs)


def test_pick_module_declares_optional_perception_and_required_grasp_spec() -> None:
    atom = BlueprintAtom.create(PickAndPlaceModule, kwargs={})

    refs = {ref.name: ref for ref in atom.module_refs}

    assert refs["_object_scene"].optional is True
    assert refs["_grasp_generator"].optional is False


def test_prompted_pick_module_requires_localizer_spec() -> None:
    atom = BlueprintAtom.create(PromptedPickAndPlaceModule, kwargs={})

    refs = {ref.name: ref for ref in atom.module_refs}

    assert refs["_prompted_localizer"].optional is False
    assert refs["_object_scene"].optional is True


def test_optional_object_scene_resolves_when_absent_or_present() -> None:
    consumer = BlueprintAtom.create(PickAndPlaceModule, kwargs={})
    module_ref = next(ref for ref in consumer.module_refs if ref.name == "_object_scene")

    absent = autoconnect(PickAndPlaceModule.blueprint())
    assert _resolve_single_ref(consumer, module_ref, module_ref.spec, absent, set()) is None

    present = autoconnect(PickAndPlaceModule.blueprint(), ObjectSceneRegistrationModule.blueprint())
    assert (
        _resolve_single_ref(consumer, module_ref, module_ref.spec, present, set())
        == ObjectSceneRegistrationModule.name
    )


def test_required_grasp_provider_rejects_absent_or_ambiguous_and_resolves_one() -> None:
    consumer = BlueprintAtom.create(PickAndPlaceModule, kwargs={})
    module_ref = next(ref for ref in consumer.module_refs if ref.name == "_grasp_generator")
    absent = autoconnect(PickAndPlaceModule.blueprint())
    with pytest.raises(Exception):
        _resolve_single_ref(consumer, module_ref, module_ref.spec, absent, set())

    present = autoconnect(PickAndPlaceModule.blueprint(), GraspGenXModule.blueprint())
    assert (
        _resolve_single_ref(consumer, module_ref, module_ref.spec, present, set())
        == GraspGenXModule.name
    )
    ambiguous = autoconnect(
        PickAndPlaceModule.blueprint(),
        GraspGenXModule.blueprint(instance_name="provider-a"),
        GraspGenXModule.blueprint(instance_name="provider-b"),
    )
    with pytest.raises(Exception, match="Multiple modules met that spec"):
        _resolve_single_ref(consumer, module_ref, module_ref.spec, ambiguous, set())


def _pointcloud(frame_id: str = "world", timestamp: float | None = None) -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray([[0.4, 0.0, 0.2], [0.41, 0.01, 0.2]], dtype=np.float32),
        frame_id=frame_id,
        timestamp=timestamp,
    )


def _candidate(x: float, score: float) -> GraspCandidate:
    return GraspCandidate(
        Pose(Vector3(x, 0.0, 0.2), Quaternion(0.0, 0.0, 0.0, 1.0)),
        score,
    )


def _grasp_visualization_config() -> GraspVisualizationConfig:
    return GraspVisualizationConfig(
        gripper={
            "extents_open": (0.1, 0.05, 0.1),
            "offset_open": (0.0, 0.0, 0.1),
            "extents_half_open": (0.05, 0.05, 0.05),
            "offset_half_open": (0.0, 0.0, 0.05),
            "fingertip_depth": 0.1,
        },
        grasp_frame_to_tcp=(
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
    )


class _LayerRecorder:
    def __init__(self, error: Exception | None = None) -> None:
        self.layers: list[VisualizationLayer] = []
        self.error = error

    def set_layer(self, layer: VisualizationLayer) -> None:
        if self.error is not None:
            raise self.error
        self.layers.append(layer)


class TestProposalSelection:
    def test_provider_receives_real_world_frame_cloud(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        now = 100.0
        cloud = _pointcloud(timestamp=now)
        detection = _make_det_object()
        scene = mocker.Mock()
        scene.get_object_pointcloud_by_object_id.return_value = cloud
        generator = mocker.Mock()
        generator.propose_grasps.return_value = GraspCandidateArray(
            Header(now, "world"), [_candidate(0.4, 0.8)]
        )
        module._object_scene = scene
        module._grasp_generator = generator
        mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=now + 0.1)

        candidates = module._provider_candidates(detection)

        proposal_input = generator.propose_grasps.call_args.args[0]
        assert isinstance(proposal_input, GraspProposalInput)
        assert proposal_input.object_pointcloud is cloud
        assert proposal_input.object_center.as_tuple == detection.center.as_tuple
        assert proposal_input.object_size.as_tuple == detection.size.as_tuple
        assert [(candidate.pose.position.x, candidate.score) for candidate in candidates] == [
            (0.4, 0.8)
        ]

    @pytest.mark.parametrize("cloud_available", [False, True])
    def test_provider_rejects_missing_or_stale_cloud(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
        cloud_available: bool,
    ) -> None:
        scene = mocker.Mock()
        scene.get_object_pointcloud_by_object_id.return_value = (
            _pointcloud(timestamp=1.0) if cloud_available else None
        )
        module._object_scene = scene
        module._grasp_generator = mocker.Mock()
        mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=100.0)

        with pytest.raises(RuntimeError, match="point cloud"):
            module._provider_candidates(_make_det_object())

    @pytest.mark.parametrize(
        ("cloud_frame", "proposal_frame"),
        [("camera", "world"), ("world", "camera")],
    )
    def test_provider_rejects_frame_mismatch(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
        cloud_frame: str,
        proposal_frame: str,
    ) -> None:
        now = 100.0
        scene = mocker.Mock()
        scene.get_object_pointcloud_by_object_id.return_value = _pointcloud(
            cloud_frame, timestamp=now
        )
        generator = mocker.Mock()
        generator.propose_grasps.return_value = GraspCandidateArray(
            Header(now, proposal_frame), [_candidate(0.4, 0.8)]
        )
        module._object_scene = scene
        module._grasp_generator = generator
        mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=now)

        with pytest.raises(RuntimeError, match="frame"):
            module._provider_candidates(_make_det_object())

    def test_provider_preserves_stable_order_for_equal_scores(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        now = 100.0
        scene = mocker.Mock()
        scene.get_object_pointcloud_by_object_id.return_value = _pointcloud(timestamp=now)
        generator = mocker.Mock()
        generator.propose_grasps.return_value = GraspCandidateArray(
            Header(now, "world"),
            [_candidate(0.1, 0.5), _candidate(0.2, 0.7), _candidate(0.3, 0.7)],
        )
        module._object_scene = scene
        module._grasp_generator = generator
        mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=now)

        candidates = module._provider_candidates(_make_det_object())

        assert [candidate.pose.position.x for candidate in candidates] == [0.2, 0.3, 0.1]

    def test_selection_skips_higher_scored_infeasible_candidate(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        solve_ik = mocker.patch.object(
            module,
            "_solve_connected_pose_sequence_ik",
            side_effect=[
                _ik_result(failed_index=0),
                _ik_result(0.1, 0.2, 0.3),
            ],
        )
        plan_sequence = mocker.patch.object(
            module, "_plan_connected_joint_sequence", return_value=_plan_result(0.0, 0.3)
        )
        plan_motion = mocker.patch.object(module, "plan_to_pose")
        command_gripper = mocker.patch.object(module, "_set_gripper_position")
        transaction = SimpleNamespace(rejections=Counter())

        selected = module._select_feasible_grasp(
            [_candidate(0.4, 0.9), _candidate(0.5, 0.8)],
            "arm",
            0.1,
            transaction,
        )

        assert selected.rank == 2
        assert selected.candidate.score == 0.8
        assert solve_ik.call_count == 2
        assert plan_sequence.call_count == 1
        assert transaction.rejections == {"pre_grasp_ik_infeasible": 1}
        plan_motion.assert_not_called()
        command_gripper.assert_not_called()

    @pytest.mark.parametrize(
        ("failed_index", "expected_rejection"),
        [
            (0, "pre_grasp_ik_infeasible"),
            (1, "grasp_ik_infeasible"),
            (2, "retreat_ik_infeasible"),
        ],
    )
    def test_selection_reports_failed_ik_waypoint(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
        failed_index: int,
        expected_rejection: str,
    ) -> None:
        mocker.patch.object(
            module,
            "_solve_connected_pose_sequence_ik",
            return_value=_ik_result(failed_index=failed_index),
        )
        plan_sequence = mocker.patch.object(module, "_plan_connected_joint_sequence")
        transaction = SimpleNamespace(rejections=Counter())

        with pytest.raises(RuntimeError, match="No feasible grasp among 1"):
            module._select_feasible_grasp([_candidate(0.4, 0.9)], "arm", 0.1, transaction)

        assert transaction.rejections == {expected_rejection: 1}
        plan_sequence.assert_not_called()

    @pytest.mark.parametrize(
        ("failed_index", "expected_rejection"),
        [
            (0, "pre_grasp_planning_infeasible"),
            (1, "grasp_planning_infeasible"),
            (2, "retreat_planning_infeasible"),
        ],
    )
    def test_selection_reports_failed_planning_segment(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
        failed_index: int,
        expected_rejection: str,
    ) -> None:
        mocker.patch.object(
            module,
            "_solve_connected_pose_sequence_ik",
            return_value=_ik_result(0.1, 0.2, 0.3),
        )
        mocker.patch.object(
            module,
            "_plan_connected_joint_sequence",
            return_value=_plan_result(failed_index=failed_index),
        )
        transaction = SimpleNamespace(rejections=Counter())

        with pytest.raises(RuntimeError, match="No feasible grasp among 1"):
            module._select_feasible_grasp([_candidate(0.4, 0.9)], "arm", 0.1, transaction)

        assert transaction.rejections == {expected_rejection: 1}

    def test_selection_checks_candidates_beyond_rank_five(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        invalid = [_candidate(0.4 + index * 0.01, 0.9 - index * 0.01) for index in range(5)]
        for candidate in invalid:
            candidate.pose.orientation.w = 0.0
        solve_ik = mocker.patch.object(
            module,
            "_solve_connected_pose_sequence_ik",
            return_value=_ik_result(0.1, 0.2, 0.3),
        )
        plan_sequence = mocker.patch.object(
            module,
            "_plan_connected_joint_sequence",
            return_value=_plan_result(0.0, 0.3),
        )
        transaction = SimpleNamespace(rejections=Counter())

        selected = module._select_feasible_grasp(
            [*invalid, _candidate(0.5, 0.8)],
            "arm",
            0.1,
            transaction,
        )

        assert selected.rank == 6
        solve_ik.assert_called_once()
        plan_sequence.assert_called_once()
        assert transaction.rejections == {"invalid": 5}

    def test_selection_visualization_tracks_current_rejected_and_selected(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        recorder = _LayerRecorder()
        module._world_monitor = SimpleNamespace(visualization=recorder)
        module.config.grasp_visualization = _grasp_visualization_config()
        mocker.patch.object(
            module,
            "_solve_connected_pose_sequence_ik",
            side_effect=[
                _ik_result(failed_index=0),
                _ik_result(0.1, 0.2, 0.3),
            ],
        )
        mocker.patch.object(
            module,
            "_plan_connected_joint_sequence",
            return_value=_plan_result(0.0, 0.3),
        )

        selected = module._select_feasible_grasp(
            [_candidate(0.4, 0.9), _candidate(0.5, 0.8)],
            "arm",
            0.1,
            SimpleNamespace(rejections=Counter()),
        )

        proposal_layers = [layer for layer in recorder.layers if layer.id == "grasp/proposals"]
        assert selected.rank == 2
        assert [element.id for element in proposal_layers[-1].elements] == ["rank-2"]
        selected_element = proposal_layers[-1].elements[0]
        assert isinstance(selected_element, LineSetElement)
        np.testing.assert_array_equal(selected_element.colors, [0, 220, 80])
        rejected_current = next(
            layer.elements
            for layer in proposal_layers
            if np.array_equal(layer.elements[0].colors, [230, 50, 50])
            and np.array_equal(layer.elements[1].colors, [255, 220, 0])
        )
        np.testing.assert_array_equal(rejected_current[0].colors, [230, 50, 50])
        np.testing.assert_array_equal(rejected_current[1].colors, [255, 220, 0])

    def test_selection_visualization_retains_all_rejected_candidates(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        recorder = _LayerRecorder()
        module._world_monitor = SimpleNamespace(visualization=recorder)
        module.config.grasp_visualization = _grasp_visualization_config()
        mocker.patch.object(
            module,
            "_solve_connected_pose_sequence_ik",
            side_effect=[
                _ik_result(failed_index=0),
                _ik_result(failed_index=1),
            ],
        )

        with pytest.raises(RuntimeError, match="No feasible grasp"):
            module._select_feasible_grasp(
                [_candidate(0.4, 0.9), _candidate(0.5, 0.8)],
                "arm",
                0.1,
                SimpleNamespace(rejections=Counter()),
            )

        final = recorder.layers[-1]
        assert [element.id for element in final.elements] == ["rank-1", "rank-2"]
        for element in final.elements:
            assert isinstance(element, LineSetElement)
            np.testing.assert_array_equal(element.colors, [230, 50, 50])

    def test_visualization_failure_does_not_change_selection(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module._world_monitor = SimpleNamespace(
            visualization=_LayerRecorder(RuntimeError("renderer unavailable"))
        )
        module.config.grasp_visualization = _grasp_visualization_config()
        mocker.patch.object(
            module,
            "_solve_connected_pose_sequence_ik",
            return_value=_ik_result(0.1, 0.2, 0.3),
        )
        mocker.patch.object(
            module,
            "_plan_connected_joint_sequence",
            return_value=_plan_result(0.0, 0.3),
        )

        selected = module._select_feasible_grasp(
            [_candidate(0.4, 0.9)],
            "arm",
            0.1,
            SimpleNamespace(rejections=Counter()),
        )

        assert selected.rank == 1

    def test_selection_stops_after_first_fully_feasible_candidate(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        solve_ik = mocker.patch.object(
            module,
            "_solve_connected_pose_sequence_ik",
            return_value=_ik_result(1.0, 2.0, 3.0),
        )
        plan_sequence = mocker.patch.object(
            module,
            "_plan_connected_joint_sequence",
            return_value=_plan_result(0.0, 0.6),
        )

        selected = module._select_feasible_grasp(
            [_candidate(0.4, 0.90), _candidate(0.5, 0.88)],
            "arm",
            0.1,
            SimpleNamespace(rejections=Counter()),
        )

        assert selected.rank == 1
        solve_ik.assert_called_once()
        plan_sequence.assert_called_once()

    def test_selection_tries_next_candidate_after_full_plan_failure(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        mocker.patch.object(
            module,
            "_solve_connected_pose_sequence_ik",
            side_effect=[
                _ik_result(0.1, 0.2, 0.3),
                _ik_result(0.4, 0.5, 0.6),
            ],
        )
        mocker.patch.object(
            module,
            "_plan_connected_joint_sequence",
            side_effect=[
                _plan_result(failed_index=1),
                _plan_result(0.0, 0.1),
            ],
        )
        transaction = SimpleNamespace(rejections=Counter())

        selected = module._select_feasible_grasp(
            [_candidate(0.4, 0.90), _candidate(0.5, 0.80)],
            "arm",
            0.1,
            transaction,
        )

        assert selected.rank == 2
        assert transaction.rejections == {"grasp_planning_infeasible": 1}


class TestPickTransaction:
    def _arrange_success(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> tuple[GraspCandidate, SimpleNamespace]:
        detection = _make_det_object()
        candidate = _candidate(0.4, 0.9)
        selected = _FeasibleGrasp(candidate, 1, Pose(0.4, 0.0, 0.3), Pose(0.4, 0.0, 0.3))
        robot_config = SimpleNamespace(pre_grasp_offset=0.1)
        mocker.patch.object(
            module, "_get_robot", return_value=("arm", "robot-id", robot_config, None)
        )
        mocker.patch.object(module, "_require_pick_object", return_value=detection)
        mocker.patch.object(module, "_provider_candidates", return_value=[candidate])
        mocker.patch.object(module, "_select_feasible_grasp", return_value=selected)
        mocker.patch.object(module, "_safety_lift_pose", return_value=None)
        mocker.patch.object(module, "_lift_if_low", return_value=SkillResult.ok())
        mocker.patch.object(module, "plan_to_pose", return_value=True)
        mocker.patch.object(module, "_preview_execute_wait", return_value=SkillResult.ok())
        mocker.patch.object(module, "_set_gripper_position", return_value=True)
        mocker.patch.object(
            module,
            "_verify_grasp",
            return_value=_GraspVerification(True, 0.1, "verified"),
        )
        suppression = SimpleNamespace(cleanup_error=None)
        world = mocker.Mock()
        world.suppress_object_obstacle.return_value = nullcontext(suppression)
        module._world_monitor = world
        return candidate, suppression

    def test_success_executes_ordered_pick_and_records_metadata(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        candidate, _ = self._arrange_success(module, mocker)

        result = module.pick("cup", object_id="abc12345")

        assert result.is_success()
        assert result.metadata["candidate_rank"] == 1
        assert result.metadata["candidate_score"] == 0.9
        assert module._last_pick_pose is candidate.pose
        assert module._set_gripper_position.call_args_list == [
            mocker.call(0.85, "arm"),
            mocker.call(0.0, "arm"),
        ]
        assert module.plan_to_pose.call_args_list == [
            mocker.call(Pose(0.4, 0.0, 0.3), "arm"),
            mocker.call(candidate.pose, "arm"),
            mocker.call(Pose(0.4, 0.0, 0.3), "arm"),
        ]

    def test_no_safety_lift_validates_candidates_from_current_state(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        self._arrange_success(module, mocker)
        check_sequence = mocker.patch.object(module, "_check_connected_pose_sequence")

        result = module.pick("cup", object_id="abc12345")

        assert result.is_success()
        check_sequence.assert_not_called()
        assert module._select_feasible_grasp.call_args.args[4] is None

    def test_safety_lift_endpoint_is_shared_with_candidate_validation(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        self._arrange_success(module, mocker)
        lift_pose = Pose(0.2, 0.0, 0.1)
        lift_endpoint = JointState(name=["arm/joint1"], position=[0.2])
        module._safety_lift_pose.return_value = lift_pose
        check_sequence = mocker.patch.object(
            module,
            "_check_connected_pose_sequence",
            return_value=(None, lift_endpoint),
        )

        result = module.pick("cup", object_id="abc12345")

        assert result.is_success()
        check_sequence.assert_called_once_with((lift_pose,), "arm")
        assert module._select_feasible_grasp.call_args.args[4] is lift_endpoint

    def test_safety_lift_planning_failure_aborts_prepare_without_candidate_rejection(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        self._arrange_success(module, mocker)
        lift_pose = Pose(0.2, 0.0, 0.1)
        module._safety_lift_pose.return_value = lift_pose
        check_sequence = mocker.patch.object(
            module,
            "_check_connected_pose_sequence",
            return_value=(0, None),
        )

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == "PLANNING_FAILED"
        assert result.metadata["phase"] == "PREPARE"
        assert result.metadata["rejections"] == {}
        check_sequence.assert_called_once_with((lift_pose,), "arm")
        module._select_feasible_grasp.assert_not_called()
        module._lift_if_low.assert_not_called()
        module.plan_to_pose.assert_not_called()
        module._set_gripper_position.assert_not_called()

    def test_retreat_failure_keeps_gripper_closed(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        self._arrange_success(module, mocker)
        module.plan_to_pose.side_effect = [True, True, False]

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == "PLANNING_FAILED"
        assert result.metadata["object_may_be_held"] is True
        assert module._set_gripper_position.call_args_list == [
            mocker.call(0.85, "arm"),
            mocker.call(0.0, "arm"),
        ]

    def test_concurrent_pick_is_rejected_without_robot_access(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
    ) -> None:
        get_robot = mocker.patch.object(module, "_get_robot")
        log = mocker.patch("dimos.agents.annotation.logger.info")
        module._pick_guard.acquire()
        try:
            result = module.pick("cup")
        finally:
            module._pick_guard.release()

        assert result.error_code == "PICK_BUSY"
        get_robot.assert_not_called()
        log.assert_called_once()
        assert log.call_args.args[:3] == (
            "SKILL %s result=%s duration_ms=%.1f",
            "pick",
            "PICK_BUSY",
        )

    def test_cleanup_failure_does_not_hide_primary_failure(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        _, suppression = self._arrange_success(module, mocker)
        suppression.cleanup_error = "restore failed"
        module.plan_to_pose.side_effect = [False]

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == "PLANNING_FAILED"
        assert "cleanup: restore failed" in result.message

    def test_cleanup_failure_turns_success_into_scene_failure(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        _, suppression = self._arrange_success(module, mocker)
        suppression.cleanup_error = "restore failed"

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == "WORLD_MONITOR_UNAVAILABLE"
        assert "restore failed" in result.message

    @pytest.mark.parametrize(
        ("setup", "expected_code", "expected_phase"),
        [
            ("prepare", "EXECUTION_FAILED", "PREPARE"),
            ("open", "GRIPPER_FAILED", "PREPARE"),
            ("approach_planning", "PLANNING_FAILED", "APPROACH"),
            ("approach_execution", "EXECUTION_FAILED", "APPROACH"),
            ("grasp_planning", "PLANNING_FAILED", "GRASP"),
            ("grasp_execution", "EXECUTION_FAILED", "GRASP"),
            ("close", "GRIPPER_FAILED", "CLOSE"),
            ("verification", "GRASP_VERIFICATION_FAILED", "VERIFY"),
            ("retreat_planning", "PLANNING_FAILED", "RETREAT"),
            ("retreat_execution", "EXECUTION_FAILED", "RETREAT"),
        ],
    )
    def test_phase_failures_stop_the_pipeline(
        self,
        module: PickAndPlaceModule,
        mocker: MockerFixture,
        setup: str,
        expected_code: str,
        expected_phase: str,
    ) -> None:
        self._arrange_success(module, mocker)
        if setup == "prepare":
            module._lift_if_low.return_value = SkillResult.fail("EXECUTION_FAILED", "lift failed")
        elif setup == "open":
            module._set_gripper_position.return_value = False
        elif setup == "approach_planning":
            module.plan_to_pose.side_effect = [False]
        elif setup == "approach_execution":
            module._preview_execute_wait.side_effect = [
                SkillResult.fail("EXECUTION_FAILED", "rejected")
            ]
        elif setup == "grasp_planning":
            module.plan_to_pose.side_effect = [True, False]
        elif setup == "grasp_execution":
            module._preview_execute_wait.side_effect = [
                SkillResult.ok(),
                SkillResult.fail("EXECUTION_FAILED", "rejected"),
            ]
        elif setup == "close":
            module._set_gripper_position.side_effect = [True, False]
        elif setup == "verification":
            module._verify_grasp.return_value = _GraspVerification(False, 0.0, "empty close")
        elif setup == "retreat_planning":
            module.plan_to_pose.side_effect = [True, True, False]
        else:
            module._preview_execute_wait.side_effect = [
                SkillResult.ok(),
                SkillResult.ok(),
                SkillResult.fail("EXECUTION_FAILED", "rejected"),
            ]

        result = module.pick("cup", object_id="abc12345")

        assert result.error_code == expected_code
        assert result.metadata["phase"] == expected_phase
        module._select_feasible_grasp.assert_called_once()


def test_full_pick_pipeline_uses_real_messages_and_fake_boundary_providers(
    module: PickAndPlaceModule, mocker: MockerFixture
) -> None:
    now = 100.0
    detection = _make_det_object()
    module._detection_snapshot = [detection]
    scene = mocker.Mock()
    scene.get_object_pointcloud_by_object_id.return_value = _pointcloud(timestamp=now)
    generator = mocker.Mock()
    generator.propose_grasps.return_value = GraspCandidateArray(
        Header(now, "world"),
        [_candidate(0.4, 0.9), _candidate(0.5, 0.8)],
    )
    module._object_scene = scene
    module._grasp_generator = generator
    robot_config = SimpleNamespace(pre_grasp_offset=0.1)
    mocker.patch.object(module, "_get_robot", return_value=("arm", "robot-id", robot_config, None))
    solve_ik = mocker.patch.object(
        module,
        "_solve_connected_pose_sequence_ik",
        side_effect=[
            _ik_result(failed_index=0),
            _ik_result(0.1, 0.2, 0.3),
        ],
    )
    plan_sequence = mocker.patch.object(
        module,
        "_plan_connected_joint_sequence",
        return_value=_plan_result(0.0, 0.3),
    )
    mocker.patch.object(module, "_safety_lift_pose", return_value=None)
    mocker.patch.object(module, "_lift_if_low", return_value=SkillResult.ok())
    plan = mocker.patch.object(module, "plan_to_pose", return_value=True)
    execute = mocker.patch.object(module, "_preview_execute_wait", return_value=SkillResult.ok())
    gripper = mocker.patch.object(module, "_set_gripper_position", return_value=True)
    suppression = SimpleNamespace(cleanup_error=None)
    world = mocker.Mock()
    world.visualization = None
    world.suppress_object_obstacle.return_value = nullcontext(suppression)
    module._world_monitor = world
    mocker.patch("dimos.manipulation.pick_and_place_module.time.time", return_value=now)

    result = module.pick("cup", object_id="abc12345")

    assert result.is_success()
    assert result.metadata["candidate_rank"] == 2
    assert result.metadata["candidate_score"] == 0.8
    assert result.metadata["rejections"] == {"pre_grasp_ik_infeasible": 1}
    scene.get_object_pointcloud_by_object_id.assert_called_once_with("abc12345")
    proposal_input = generator.propose_grasps.call_args.args[0]
    assert isinstance(proposal_input, GraspProposalInput)
    assert proposal_input.object_pointcloud is scene.get_object_pointcloud_by_object_id()
    world.suppress_object_obstacle.assert_called_once_with("abc12345")
    assert world.method_calls == [mocker.call.suppress_object_obstacle("abc12345")]
    assert solve_ik.call_count == 2
    assert plan_sequence.call_count == 1
    assert plan.call_count == 3
    assert execute.call_count == 3
    assert gripper.call_args_list == [mocker.call(0.85, "arm"), mocker.call(0.0, "arm")]


class TestPromptedPickTransaction:
    def test_localized_cloud_is_proposed_without_object_suppression(
        self,
        prompted_module: PromptedPickAndPlaceModule,
        mocker: MockerFixture,
    ) -> None:
        cloud = _pointcloud(timestamp=1.0)
        prompted_module._prompted_localizer = mocker.Mock()
        prompted_module._prompted_localizer.localize.return_value = cloud
        prompted_module._grasp_generator = mocker.Mock()
        candidate = _candidate(0.4, 0.9)
        prompted_module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
            Header(1.0, "world"), [candidate]
        )
        robot_config = SimpleNamespace(pre_grasp_offset=0.1)
        mocker.patch.object(
            prompted_module,
            "_get_robot",
            return_value=("arm", "robot-id", robot_config, None),
        )
        run_pick = mocker.patch.object(
            prompted_module,
            "_run_resolved_pick",
            return_value=SkillResult.ok("planned"),
        )

        result = prompted_module.pick("white and red marker", object_id="ignored")

        assert result.is_success()
        prompted_module._prompted_localizer.localize.assert_called_once_with("white and red marker")
        proposal_input = prompted_module._grasp_generator.propose_grasps.call_args.args[0]
        assert isinstance(proposal_input, GraspProposalInput)
        assert proposal_input.object_pointcloud is cloud
        assert proposal_input.object_center.as_tuple == pytest.approx((0.405, 0.005, 0.2))
        run_pick.assert_called_once()
        assert run_pick.call_args.kwargs == {"suppress_object_id": None}

    @pytest.mark.parametrize("failure", [None, RuntimeError("models unavailable")])
    def test_localization_failure_stops_before_grasp_or_motion(
        self,
        prompted_module: PromptedPickAndPlaceModule,
        mocker: MockerFixture,
        failure: Exception | None,
    ) -> None:
        prompted_module._prompted_localizer = mocker.Mock()
        if failure is None:
            prompted_module._prompted_localizer.localize.return_value = None
        else:
            prompted_module._prompted_localizer.localize.side_effect = failure
        prompted_module._grasp_generator = mocker.Mock()
        robot_config = SimpleNamespace(pre_grasp_offset=0.1)
        mocker.patch.object(
            prompted_module,
            "_get_robot",
            return_value=("arm", "robot-id", robot_config, None),
        )
        run_pick = mocker.patch.object(prompted_module, "_run_resolved_pick")
        plan = mocker.patch.object(prompted_module, "plan_to_pose")
        gripper = mocker.patch.object(prompted_module, "_set_gripper_position")

        result = prompted_module.pick("marker")

        assert result.error_code == "LOCALIZATION_FAILED"
        assert result.metadata["phase"] == "RESOLVE"
        prompted_module._grasp_generator.propose_grasps.assert_not_called()
        run_pick.assert_not_called()
        plan.assert_not_called()
        gripper.assert_not_called()

    def test_resolved_prompted_pick_does_not_call_obstacle_suppression(
        self,
        prompted_module: PromptedPickAndPlaceModule,
        mocker: MockerFixture,
    ) -> None:
        candidate = _candidate(0.4, 0.9)
        selected = _FeasibleGrasp(candidate, 1, candidate.pose, candidate.pose)
        world = mocker.Mock()
        prompted_module._world_monitor = world
        mocker.patch.object(prompted_module, "_safety_lift_pose", return_value=None)
        mocker.patch.object(prompted_module, "_select_feasible_grasp", return_value=selected)
        mocker.patch.object(
            prompted_module,
            "_execute_selected_pick",
            return_value=SkillResult.ok("picked"),
        )

        result = prompted_module._run_resolved_pick(
            SimpleNamespace(
                phase=None,
                selected=None,
                rejections=Counter(),
                object_id="",
                object_name="marker",
                gripper_closed=False,
            ),
            [candidate],
            "arm",
            0.1,
            suppress_object_id=None,
        )

        assert result.is_success()
        world.suppress_object_obstacle.assert_not_called()

    def test_prompted_cloud_uses_existing_visualization_layer(
        self,
        prompted_module: PromptedPickAndPlaceModule,
        mocker: MockerFixture,
    ) -> None:
        cloud = _pointcloud(timestamp=1.0)
        recorder = _LayerRecorder()
        prompted_module._world_monitor = SimpleNamespace(visualization=recorder)
        prompted_module._grasp_generator = mocker.Mock()
        prompted_module._grasp_generator.propose_grasps.return_value = GraspCandidateArray(
            Header(1.0, "world"), [_candidate(0.4, 0.9)]
        )

        prompted_module._pointcloud_candidates(
            GraspProposalInput.from_pointcloud(cloud),
            "marker",
        )

        assert len(recorder.layers) == 1

    def test_plan_only_mode_keeps_full_planning_and_skips_execution(
        self,
        prompted_module: PromptedPickAndPlaceModule,
        mocker: MockerFixture,
    ) -> None:
        prompted_module.config.execute_pick = False
        prompted_module._world_monitor = SimpleNamespace(visualization=None)
        candidate = _candidate(0.4, 0.9)
        transaction = _PickTransaction(object_name="marker")
        solve_ik = mocker.patch.object(
            prompted_module,
            "_solve_connected_pose_sequence_ik",
            return_value=_ik_result(0.1, 0.2, 0.3),
        )
        plan_sequence = mocker.patch.object(
            prompted_module,
            "_plan_connected_joint_sequence",
            return_value=_plan_result(0.0, 0.3),
        )
        mocker.patch.object(prompted_module, "_safety_lift_pose", return_value=None)
        execute_pick = mocker.patch.object(prompted_module, "_execute_selected_pick")
        plan_motion = mocker.patch.object(prompted_module, "plan_to_pose")
        execute_motion = mocker.patch.object(prompted_module, "_preview_execute_wait")
        gripper = mocker.patch.object(prompted_module, "_set_gripper_position")

        result = prompted_module._run_resolved_pick(
            transaction,
            [candidate],
            "arm",
            0.1,
            suppress_object_id=None,
        )

        assert result.is_success()
        assert result.metadata["planning_only"] is True
        assert result.metadata["candidate_rank"] == 1
        assert "no motion executed" in result.message
        assert transaction.phase.value == "DONE"
        assert prompted_module._last_pick_pose is None
        solve_ik.assert_called_once()
        plan_sequence.assert_called_once()
        execute_pick.assert_not_called()
        plan_motion.assert_not_called()
        execute_motion.assert_not_called()
        gripper.assert_not_called()

    def test_pick_and_place_stops_after_plan_only_pick(
        self,
        prompted_module: PromptedPickAndPlaceModule,
        mocker: MockerFixture,
    ) -> None:
        mocker.patch.object(
            prompted_module,
            "pick",
            return_value=SkillResult[ManipulationSkillError].ok(
                "pick planned",
                planning_only=True,
            ),
        )
        place = mocker.patch.object(prompted_module, "place")

        result = prompted_module.pick_and_place("marker", 0.5, 0.0, 0.2)

        assert result.is_success()
        assert result.metadata["planning_only"] is True
        assert result.message == "pick planned; place skipped"
        place.assert_not_called()


class TestGraspVerification:
    def test_empty_close_fails_immediately(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.grasp_verification = GraspVerificationConfig(
            enabled=True,
            timeout=1.0,
            poll_interval=0.1,
            held_threshold=0.02,
        )
        mocker.patch.object(module, "get_gripper", return_value=0.0)
        mocker.patch(
            "dimos.manipulation.pick_and_place_module.time.monotonic",
            side_effect=[0.0, 0.1],
        )

        result = module._verify_grasp("arm")

        assert result == _GraspVerification(False, 0.0, "gripper reached the empty-closed region")

    def test_held_position_succeeds_after_timeout(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.grasp_verification = GraspVerificationConfig(
            enabled=True,
            timeout=1.0,
            poll_interval=0.1,
            held_threshold=0.02,
        )
        mocker.patch.object(module, "get_gripper", return_value=0.1)
        mocker.patch(
            "dimos.manipulation.pick_and_place_module.time.monotonic",
            side_effect=[0.0, 0.1, 1.1],
        )
        sleep = mocker.patch("dimos.manipulation.pick_and_place_module.time.sleep")

        result = module._verify_grasp("arm")

        assert result == _GraspVerification(True, 0.1, "grasp verified by gripper closure feedback")
        sleep.assert_called_once_with(0.1)

    def test_no_gripper_motion_is_not_misclassified_as_a_grasp(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.grasp_verification = GraspVerificationConfig(
            enabled=True,
            timeout=1.0,
            poll_interval=0.1,
            held_threshold=0.02,
        )
        mocker.patch.object(module, "get_gripper", return_value=0.85)
        mocker.patch(
            "dimos.manipulation.pick_and_place_module.time.monotonic",
            side_effect=[0.0, 0.1, 1.1],
        )
        mocker.patch("dimos.manipulation.pick_and_place_module.time.sleep")

        result = module._verify_grasp("arm")

        assert result == _GraspVerification(False, 0.85, "gripper did not leave the open position")

    def test_feedback_timeout_is_reported(
        self, module: PickAndPlaceModule, mocker: MockerFixture
    ) -> None:
        module.config.grasp_verification = GraspVerificationConfig(
            enabled=True,
            timeout=1.0,
            poll_interval=0.1,
            held_threshold=0.02,
        )
        mocker.patch.object(module, "get_gripper", return_value=None)
        mocker.patch(
            "dimos.manipulation.pick_and_place_module.time.monotonic",
            side_effect=[0.0, 0.1, 1.1],
        )
        mocker.patch("dimos.manipulation.pick_and_place_module.time.sleep")

        result = module._verify_grasp("arm")

        assert result == _GraspVerification(False, None, "gripper feedback was unavailable")
