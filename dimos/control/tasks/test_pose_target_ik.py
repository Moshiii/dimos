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

"""Behavior tests for the shared pose-target IK control core."""

from pathlib import Path
from typing import cast

import pytest
from pytest_mock import MockerFixture

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.pose_target_ik import (
    FrameTargetSnapshot,
    PoseTargetIKTask,
    PoseTargetIKTaskConfig,
)
from dimos.manipulation.planning.kinematics.pink_ik import PinkIK, PinkIKFeedbackLimitError
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState


class _Task(PoseTargetIKTask):
    def __init__(
        self,
        config: PoseTargetIKTaskConfig,
        ik: PinkIK,
        snapshot: FrameTargetSnapshot | None,
        additional_joints: tuple[str, ...] = (),
    ) -> None:
        self.snapshot = snapshot
        self.timed_out = False
        super().__init__(
            "pose_target",
            config,
            additional_claimed_joints=additional_joints,
            ik=ik,
        )

    def is_active(self) -> bool:
        return self.snapshot is not None

    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        return self.snapshot

    def _on_target_timeout(self) -> None:
        self.timed_out = True


def _robot_model() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("fake.urdf"),
        joint_names=["model_a", "model_b"],
        joint_name_mapping={"arm/a": "model_a", "arm/b": "model_b"},
    )


def _config(
    *,
    joint_names: tuple[str, ...] = ("arm/a", "arm/b"),
    target_frames: tuple[str, ...] = ("tool",),
    timeout: float = 0.5,
    max_joint_delta_deg: float = 10.0,
) -> PoseTargetIKTaskConfig:
    return PoseTargetIKTaskConfig(
        joint_names=joint_names,
        robot_model=_robot_model(),
        target_frames=target_frames,
        timeout=timeout,
        max_joint_delta_deg=max_joint_delta_deg,
    )


def _ik(mocker: MockerFixture, positions: list[float] | None = None) -> PinkIK:
    ik = mocker.Mock(spec=PinkIK)
    ik.step_frame_targets.return_value = JointState(
        name=["arm/a", "arm/b"], position=positions or [0.01, -0.01]
    )
    ik.frame_poses.return_value = {"tool": PoseStamped(frame_id="base")}
    return cast("PinkIK", ik)


def _state(*, t_now: float = 1.0, positions: dict[str, float] | None = None) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions=positions or {"arm/a": 0.0, "arm/b": 0.0}),
        t_now=t_now,
        dt=0.01,
    )


def _snapshot(
    *,
    targets: dict[str, PoseStamped] | None = None,
    last_update_time: float = 1.0,
    extra_joint_positions: dict[str, float] | None = None,
) -> FrameTargetSnapshot:
    return FrameTargetSnapshot(
        targets=targets or {"tool": PoseStamped(frame_id="world")},
        last_update_time=last_update_time,
        extra_joint_positions=extra_joint_positions or {},
    )


def test_constructor_validates_model_frames_and_controlled_joints(
    mocker: MockerFixture,
) -> None:
    ik = _ik(mocker)

    _Task(_config(), ik, _snapshot())

    ik.validate_frame_targets.assert_called_once_with(_robot_model(), ("tool",), ("arm/a", "arm/b"))


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("joint_names", (), "at least one joint"),
        ("joint_names", ("arm/a", "arm/a"), "unique joint names"),
        ("target_frames", (), "at least one target frame"),
        ("target_frames", ("tool", "tool"), "unique target frames"),
    ],
)
def test_constructor_rejects_invalid_common_configuration(
    mocker: MockerFixture, field: str, value: tuple[str, ...], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        config = (
            _config(joint_names=value) if field == "joint_names" else _config(target_frames=value)
        )
        _Task(config, _ik(mocker), _snapshot())


def test_compute_calls_one_pink_step_and_preserves_output_order(
    mocker: MockerFixture,
) -> None:
    ik = _ik(mocker, [0.02, -0.03])
    task = _Task(
        _config(),
        ik,
        _snapshot(extra_joint_positions={"arm/gripper": 0.4}),
        additional_joints=("arm/gripper",),
    )

    output = task.compute(_state())

    assert output is not None
    assert output.joint_names == ["arm/a", "arm/b", "arm/gripper"]
    assert output.positions == [0.02, -0.03, 0.4]
    ik.step_frame_targets.assert_called_once()
    assert ik.step_frame_targets.call_args.kwargs["dt"] == 0.01
    assert task.claim().joints == frozenset({"arm/a", "arm/b", "arm/gripper"})


def test_compute_without_complete_joint_state_skips_pink(mocker: MockerFixture) -> None:
    ik = _ik(mocker)
    task = _Task(_config(), ik, _snapshot())

    output = task.compute(_state(positions={"arm/a": 0.0}))

    assert output is None
    ik.step_frame_targets.assert_not_called()


def test_compute_skips_tick_when_pink_solver_raises(mocker: MockerFixture) -> None:
    ik = _ik(mocker)
    ik.step_frame_targets.side_effect = Exception("QP solver found no solution")
    task = _Task(_config(), ik, _snapshot())

    assert task.compute(_state()) is None


def test_feedback_limit_warnings_are_rate_limited(mocker: MockerFixture) -> None:
    ik = _ik(mocker)
    ik.step_frame_targets.side_effect = PinkIKFeedbackLimitError(
        joint_name="arm/a",
        value=-1.0011,
        lower=-1.0,
        upper=1.0,
        tolerance=1e-3,
    )
    warning = mocker.patch("dimos.control.tasks.pose_target_ik.logger.warning")
    task = _Task(_config(timeout=0.0), ik, _snapshot())

    assert task.compute(_state(t_now=1.0)) is None
    assert task.compute(_state(t_now=1.1)) is None
    assert task.compute(_state(t_now=2.0)) is None

    assert warning.call_count == 2


@pytest.mark.parametrize("positions", [[float("nan"), 0.0], [1.0, 0.0]])
def test_compute_rejects_unsafe_pink_result(mocker: MockerFixture, positions: list[float]) -> None:
    task = _Task(_config(), _ik(mocker, positions), _snapshot())

    assert task.compute(_state()) is None


def test_stale_snapshot_times_out_without_calling_pink(mocker: MockerFixture) -> None:
    ik = _ik(mocker)
    task = _Task(_config(timeout=0.2), ik, _snapshot(last_update_time=1.0))

    output = task.compute(_state(t_now=1.3))

    assert output is None
    assert task.timed_out
    ik.step_frame_targets.assert_not_called()


def test_current_frame_poses_uses_live_coordinator_seed(mocker: MockerFixture) -> None:
    ik = _ik(mocker)
    task = _Task(_config(), ik, _snapshot())

    poses = task.current_frame_poses(_state(positions={"arm/a": 0.2, "arm/b": 0.3}), ["tool"])

    assert poses == {"tool": PoseStamped(frame_id="base")}
    seed = ik.frame_poses.call_args.args[3]
    assert seed.name == ["arm/a", "arm/b"]
    assert seed.position == [0.2, 0.3]
