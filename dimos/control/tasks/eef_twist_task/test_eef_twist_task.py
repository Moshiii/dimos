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

from pathlib import Path
from typing import cast

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.eef_twist_task.eef_twist_task import (
    EEFTwistTask,
    EEFTwistTaskConfig,
)
from dimos.manipulation.planning.kinematics.pink_ik import PinkIK
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool


def _robot_model() -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("fake.urdf"),
        joint_names=["joint1", "joint2"],
        joint_name_mapping={"arm/joint1": "joint1", "arm/joint2": "joint2"},
    )


def _ik(mocker: MockerFixture) -> PinkIK:
    ik = mocker.Mock(spec=PinkIK)
    ik.frame_poses.return_value = {"tool": PoseStamped(frame_id="world", position=[0.0, 0.0, 0.0])}
    ik.step_frame_targets.return_value = JointState(
        name=["arm/joint1", "arm/joint2"],
        position=[0.01, 0.02],
    )
    return cast("PinkIK", ik)


def _task(mocker: MockerFixture, *, gripper: bool = False) -> tuple[EEFTwistTask, PinkIK]:
    ik = _ik(mocker)
    task = EEFTwistTask(
        "eef",
        EEFTwistTaskConfig(
            joint_names=("arm/joint1", "arm/joint2"),
            robot_model=_robot_model(),
            target_frames=("tool",),
            timeout=0.0,
            command_timeout=0.3,
            gripper_joint="arm/gripper" if gripper else None,
            gripper_open_pos=0.8,
            gripper_closed_pos=0.1,
        ),
        ik=ik,
    )
    return task, ik


def _state(t_now: float = 1.0, *, dt: float = 0.01) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={"arm/joint1": 0.0, "arm/joint2": 0.0},
        ),
        t_now=t_now,
        dt=dt,
    )


def _twist(x: float = 0.1) -> TwistStamped:
    return TwistStamped(frame_id="tool", linear=[x, 0.0, 0.0], angular=[0.0, 0.0, 0.0])


def test_twist_integrates_target_and_uses_shared_persistent_command(
    mocker: MockerFixture,
) -> None:
    task, ik = _task(mocker)
    assert task.on_ee_twist_command(_twist(), t_now=1.0)

    first = task.compute(_state(1.01))
    second = task.compute(_state(1.02))

    assert first is not None
    assert second is not None
    first_target = ik.step_frame_targets.call_args_list[0].kwargs["frame_targets"]["tool"]
    second_target = ik.step_frame_targets.call_args_list[1].kwargs["frame_targets"]["tool"]
    assert first_target.position.x == pytest.approx(0.001)
    assert second_target.position.x == pytest.approx(0.002)
    assert ik.step_frame_targets.call_args_list[1].kwargs["command_state"].position == [0.01, 0.02]


def test_zero_twist_holds_the_integrated_target(mocker: MockerFixture) -> None:
    task, ik = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))
    task.on_ee_twist_command(_twist(0.0), t_now=1.02)

    task.compute(_state(1.03))

    target = ik.step_frame_targets.call_args.kwargs["frame_targets"]["tool"]
    assert target.position.x == pytest.approx(0.001)


def test_stale_twist_stops_motion_without_dropping_hold(mocker: MockerFixture) -> None:
    task, ik = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))

    output = task.compute(_state(1.31))

    assert output is not None
    target = ik.step_frame_targets.call_args.kwargs["frame_targets"]["tool"]
    assert target.position.x == pytest.approx(0.001)


def test_gripper_is_claimed_and_appended_to_joint_output(mocker: MockerFixture) -> None:
    task, _ = _task(mocker, gripper=True)
    task.on_gripper_command(Bool(True), t_now=1.0)

    output = task.compute(_state())

    assert output is not None
    assert task.claim().joints == frozenset({"arm/joint1", "arm/joint2", "arm/gripper"})
    assert output.joint_names == ["arm/joint1", "arm/joint2", "arm/gripper"]
    assert output.positions[-1] == pytest.approx(0.1)


def test_estop_rejects_input_and_reanchors_after_clear(mocker: MockerFixture) -> None:
    task, ik = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))

    task.set_estop(True)
    assert not task.on_ee_twist_command(_twist(), t_now=1.02)
    assert task.compute(_state(1.02)) is None

    task.set_estop(False)
    assert task.compute(_state(1.03)) is not None
    assert ik.frame_poses.call_count == 2


def test_preemption_discards_twist_target_and_command_state(mocker: MockerFixture) -> None:
    task, ik = _task(mocker)
    task.on_ee_twist_command(_twist(), t_now=1.0)
    task.compute(_state(1.01))

    task.on_preempted("trajectory", frozenset({"arm/joint1"}))
    task.compute(_state(1.02))

    assert ik.frame_poses.call_count == 2
    assert ik.step_frame_targets.call_args.kwargs["command_state"].position == [0.0, 0.0]


def test_invalid_twist_is_rejected(mocker: MockerFixture) -> None:
    task, _ = _task(mocker)
    invalid = _twist()
    invalid.linear.x = np.nan

    assert not task.on_ee_twist_command(invalid, t_now=1.0)
