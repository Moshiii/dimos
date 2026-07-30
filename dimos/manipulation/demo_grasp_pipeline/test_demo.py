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

"""Hermetic coverage for the offline proposal-to-motion-planning demo."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.manipulation_module import ConnectedPoseSequenceResult
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config

from . import __main__
from .demo import PipelineDemoResult, run_demo


def _clouds() -> tuple[PointCloud2, PointCloud2]:
    points = np.asarray(
        [
            [1.0, 2.0, 3.0],
            [1.1, 2.0, 3.0],
            [1.0, 2.1, 3.0],
        ],
        dtype=np.float32,
    )
    return (
        PointCloud2.from_numpy(points, frame_id="world", timestamp=42.0),
        PointCloud2.from_numpy(points[:2], frame_id="world", timestamp=42.0),
    )


def _candidates() -> list[GraspCandidate]:
    return [
        GraspCandidate(
            Pose(
                {
                    "position": [0.45, 0.0, 0.25],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                }
            ),
            score,
        )
        for score in (0.9, 0.8)
    ]


def _path(start: float, goal: float) -> tuple[JointState, JointState]:
    return (
        JointState(name=["arm/joint1"], position=[start]),
        JointState(name=["arm/joint1"], position=[goal]),
    )


def _planner(mocker: MockerFixture) -> Any:
    planner = mocker.Mock(spec=PickAndPlaceModule)
    planner.config = SimpleNamespace(grasp_approach_vector=(0.0, 0.0, -1.0))
    planner._get_robot.return_value = ("arm", "robot", SimpleNamespace(pre_grasp_offset=0.05), None)
    planner._safety_lift_pose.return_value = None
    planner._valid_candidate.return_value = True
    planner._compute_pre_grasp_pose.side_effect = lambda pose, _offset, _approach: pose
    return planner


def _proposer(mocker: MockerFixture) -> Any:
    proposer = mocker.Mock()
    proposer.propose_grasps.return_value = GraspCandidateArray(
        Header(42.0, "world"),
        _candidates(),
    )
    return proposer


def test_demo_selects_first_complete_sequence_and_writes_artifacts(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    planner = _planner(mocker)
    proposer = _proposer(mocker)
    planner._plan_connected_pose_sequence.side_effect = [
        ConnectedPoseSequenceResult(1, None, (_path(0.0, 0.1),)),
        ConnectedPoseSequenceResult(
            None,
            JointState(name=["arm/joint1"], position=[0.3]),
            (_path(0.0, 0.1), _path(0.1, 0.2), _path(0.2, 0.3)),
        ),
    ]

    def write_image(path: Path, *_args: Any) -> Path:
        path.write_bytes(b"png")
        return path

    renderer = mocker.Mock(side_effect=write_image)
    output = tmp_path / "pipeline"

    result = run_demo(
        proposer,
        planner,
        output,
        gripper=make_xarm_graspgenx_config().gripper,
        grasp_frame_to_tcp=(
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.1),
            (0.0, 0.0, 0.0, 1.0),
        ),
        cloud_loader=_clouds,
        renderer=renderer,
    )

    assert result.success is True
    assert result.selected_rank == 2
    assert result.selected_score == 0.8
    assert [outcome.rejection for outcome in result.outcomes] == [
        "grasp_infeasible",
        None,
    ]
    summary = json.loads(result.summary_path.read_text(encoding="utf-8"))
    assert summary["selected"]["rank"] == 2
    assert summary["rejection_counts"] == {"grasp_infeasible": 1}
    assert summary["execution_performed"] is False
    plans = json.loads(result.plans_path.read_text(encoding="utf-8"))
    assert [segment["name"] for segment in plans["segments"]] == [
        "pre_grasp",
        "grasp",
        "retreat",
    ]
    assert plans["segments"][-1]["waypoints"][-1]["positions"] == [0.3]
    selected_array = renderer.call_args.args[3]
    assert selected_array.candidates[0].pose.position.z == pytest.approx(0.15)
    assert summary["selected"]["pose"]["position"][2] == pytest.approx(0.25)
    assert renderer.call_args.args[5] == 2
    assert result.image_path is not None
    assert result.image_path.read_bytes() == b"png"
    planner.execute.assert_not_called()
    planner.set_gripper.assert_not_called()


def test_demo_reuses_an_explicit_synthetic_start_for_every_candidate(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    planner = _planner(mocker)
    proposer = _proposer(mocker)
    start = JointState(name=["arm/joint1"], position=[0.0])
    planner._plan_connected_pose_sequence.side_effect = [
        ConnectedPoseSequenceResult(0, None, ()),
        ConnectedPoseSequenceResult(0, None, ()),
    ]

    run_demo(
        proposer,
        planner,
        tmp_path,
        gripper=make_xarm_graspgenx_config().gripper,
        sequence_start=start,
        cloud_loader=_clouds,
        renderer=mocker.Mock(),
    )

    assert all(
        call.args[2] is start for call in planner._plan_connected_pose_sequence.call_args_list
    )


def test_demo_records_shared_lift_and_exhausted_candidates(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    planner = _planner(mocker)
    proposer = _proposer(mocker)
    planner._safety_lift_pose.return_value = _candidates()[0].pose
    lift_endpoint = JointState(name=["arm/joint1"], position=[0.1])
    planner._plan_connected_pose_sequence.side_effect = [
        ConnectedPoseSequenceResult(None, lift_endpoint, (_path(0.0, 0.1),)),
        ConnectedPoseSequenceResult(0, None, ()),
        ConnectedPoseSequenceResult(2, None, (_path(0.1, 0.2), _path(0.2, 0.3))),
    ]
    renderer = mocker.Mock()

    result = run_demo(
        proposer,
        planner,
        tmp_path,
        gripper=make_xarm_graspgenx_config().gripper,
        cloud_loader=_clouds,
        renderer=renderer,
    )

    assert result.success is False
    assert result.failure_reason == "no_complete_candidate"
    assert result.selected_rank is None
    summary = json.loads(result.summary_path.read_text(encoding="utf-8"))
    assert summary["selected"] is None
    assert summary["rejection_counts"] == {
        "pre_grasp_infeasible": 1,
        "retreat_infeasible": 1,
    }
    plans = json.loads(result.plans_path.read_text(encoding="utf-8"))
    assert [segment["name"] for segment in plans["segments"]] == ["safety_lift"]
    candidate_calls = planner._plan_connected_pose_sequence.call_args_list[1:]
    assert all(call.args[2] is lift_endpoint for call in candidate_calls)
    renderer.assert_not_called()
    planner.execute.assert_not_called()
    planner.set_gripper.assert_not_called()


def test_demo_records_shared_lift_failure_without_screening_candidates(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    planner = _planner(mocker)
    proposer = _proposer(mocker)
    planner._safety_lift_pose.return_value = _candidates()[0].pose
    planner._plan_connected_pose_sequence.return_value = ConnectedPoseSequenceResult(
        0,
        None,
        (),
    )
    renderer = mocker.Mock()

    result = run_demo(
        proposer,
        planner,
        tmp_path,
        gripper=make_xarm_graspgenx_config().gripper,
        cloud_loader=_clouds,
        renderer=renderer,
    )

    assert result.success is False
    assert result.failure_reason == "safety_lift_infeasible"
    summary = json.loads(result.summary_path.read_text(encoding="utf-8"))
    assert summary["failure_reason"] == "safety_lift_infeasible"
    assert summary["checked_count"] == 0
    assert json.loads(result.plans_path.read_text(encoding="utf-8"))["segments"] == []
    assert planner._plan_connected_pose_sequence.call_count == 1
    renderer.assert_not_called()
    planner.execute.assert_not_called()
    planner.set_gripper.assert_not_called()


def test_demo_import_does_not_load_optional_graspgenx_runtime() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; "
                "import dimos.manipulation.demo_grasp_pipeline.demo; "
                "assert 'dimos.manipulation.grasping.grasp_gen_x_runtime' not in sys.modules"
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def test_cli_explains_how_to_install_missing_graspgenx(
    mocker: MockerFixture,
    capsys: pytest.CaptureFixture[str],
) -> None:
    missing = ModuleNotFoundError("No module named 'graspgenx'", name="graspgenx")
    failure = RuntimeError("failed to initialize GraspGenX")
    failure.__cause__ = missing
    mocker.patch.object(__main__, "run_contributor_demo", side_effect=failure)

    assert __main__.main([]) == 2
    output = capsys.readouterr().err
    assert "uv run --extra graspgenx" in output
    assert "No module named 'graspgenx'" in output


@pytest.mark.parametrize(("success", "exit_code"), [(True, 0), (False, 1)])
def test_cli_forwards_options_and_reports_result(
    mocker: MockerFixture,
    tmp_path: Path,
    success: bool,
    exit_code: int,
) -> None:
    run = mocker.patch.object(
        __main__,
        "run_contributor_demo",
        return_value=PipelineDemoResult(
            success,
            tmp_path,
            tmp_path / "summary.json",
            tmp_path / "plans.json",
            None,
            2,
            1 if success else None,
            0.9 if success else None,
            (),
            None if success else "no_complete_candidate",
        ),
    )

    assert (
        __main__.main(
            [
                "--output-dir",
                str(tmp_path),
                "--max-candidates",
                "7",
                "--workspace-center",
                "0.4",
                "0.1",
                "0.3",
            ]
        )
        == exit_code
    )
    run.assert_called_once_with(
        output_dir=tmp_path,
        max_candidates=7,
        workspace_center=[0.4, 0.1, 0.3],
    )
