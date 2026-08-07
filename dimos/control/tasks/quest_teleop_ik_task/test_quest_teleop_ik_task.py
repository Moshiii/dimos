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

"""Behavior tests for unified single- and two-hand Quest teleoperation."""

from pathlib import Path
from types import SimpleNamespace
from typing import cast

import pytest
from pytest_mock import MockerFixture

from dimos.control.coordinator import TaskConfig
from dimos.control.task import CoordinatorState, JointStateSnapshot
import dimos.control.tasks.pose_target_ik as pose_target_module
from dimos.control.tasks.quest_teleop_ik_task.quest_teleop_ik_task import (
    OperatorHand,
    QuestHandBinding,
    QuestTeleopIKTask,
    QuestTeleopIKTaskConfig,
    create_task,
)
from dimos.manipulation.planning.kinematics.pink_ik import PinkIK
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_types import Buttons


def _robot_model() -> RobotModelConfig:
    return RobotModelConfig(
        name="robot",
        model_path=Path("fake.urdf"),
        joint_names=["model_left", "model_right"],
        joint_name_mapping={
            "robot/left": "model_left",
            "robot/right": "model_right",
        },
    )


def _binding(
    hand: str,
    frame: str,
    gripper_joint: str | None = None,
) -> QuestHandBinding:
    return QuestHandBinding(
        hand=cast("OperatorHand", hand),
        target_frame=frame,
        gripper_joint=gripper_joint,
        gripper_open_position=1.0,
        gripper_closed_position=0.0,
    )


def _config(
    bindings: tuple[QuestHandBinding, ...], *, timeout: float = 0.5
) -> QuestTeleopIKTaskConfig:
    return QuestTeleopIKTaskConfig(
        joint_names=("robot/left", "robot/right"),
        robot_model=_robot_model(),
        bindings=bindings,
        timeout=timeout,
        max_joint_delta_deg=10.0,
    )


def _ik(mocker: MockerFixture) -> PinkIK:
    ik = mocker.Mock(spec=PinkIK)
    ik.frame_poses.return_value = {
        "left_tool": PoseStamped(
            position=Vector3(1.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        "right_tool": PoseStamped(
            position=Vector3(-1.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
    }
    ik.step_frame_targets.return_value = JointState(
        name=["robot/left", "robot/right"], position=[0.01, -0.01]
    )
    return cast("PinkIK", ik)


def _state(t_now: float = 1.0) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"robot/left": 0.0, "robot/right": 0.0}),
        t_now=t_now,
        dt=0.01,
    )


def _buttons(
    *,
    left: bool = False,
    right: bool = False,
    left_trigger: float = 0.0,
    right_trigger: float = 0.0,
) -> Buttons:
    buttons = Buttons()
    buttons.left_primary = left
    buttons.right_primary = right
    buttons.pack_analog_triggers(left_trigger, right_trigger)
    return buttons


def _pose(x: float) -> PoseStamped:
    return PoseStamped(
        position=Vector3(x, 0.0, 0.0),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


class _CustomPinkIK(PinkIK):
    instances: list["_CustomPinkIK"] = []

    def __init__(self, config: object) -> None:
        self.received_config = config
        self.instances.append(self)

    def validate_frame_targets(self, *args: object, **kwargs: object) -> None:
        pass


@pytest.mark.parametrize(
    ("bindings", "message"),
    [
        ((), "exactly one or two"),
        (
            (
                _binding("left", "left_tool"),
                _binding("right", "right_tool"),
                _binding("left", "third_tool"),
            ),
            "exactly one or two",
        ),
        (
            (_binding("left", "left_tool"), _binding("left", "right_tool")),
            "unique operator hands",
        ),
        (
            (_binding("left", "tool"), _binding("right", "tool")),
            "unique target frames",
        ),
        (
            (
                _binding("left", "left_tool", "robot/gripper"),
                _binding("right", "right_tool", "robot/gripper"),
            ),
            "unique gripper joints",
        ),
    ],
)
def test_binding_configuration_rejects_invalid_collections(
    mocker: MockerFixture,
    bindings: tuple[QuestHandBinding, ...],
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        QuestTeleopIKTask("quest", _config(bindings), ik=_ik(mocker))


def test_single_binding_tracks_relative_controller_motion(mocker: MockerFixture) -> None:
    ik = _ik(mocker)
    task = QuestTeleopIKTask(
        "quest",
        _config((_binding("right", "right_tool"),)),
        ik=ik,
    )
    task.on_teleop_buttons(_buttons(right=True), 1.0)
    task.on_right_cartesian_command(_pose(0.5), 1.0)
    assert task.compute(_state()) is not None

    task.on_right_cartesian_command(_pose(0.7), 1.1)
    output = task.compute(_state(1.1))

    assert output is not None
    target = ik.step_frame_targets.call_args.kwargs["frame_targets"]["right_tool"]
    assert target.position.x == pytest.approx(-0.8)


def test_bimanual_task_requires_both_hands_and_releases_atomically(
    mocker: MockerFixture,
) -> None:
    ik = _ik(mocker)
    task = QuestTeleopIKTask(
        "quest",
        _config(
            (
                _binding("left", "left_tool"),
                _binding("right", "right_tool"),
            )
        ),
        ik=ik,
    )
    task.on_teleop_buttons(_buttons(left=True), 1.0)
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    task.on_right_cartesian_command(_pose(-0.1), 1.0)
    assert task.compute(_state()) is None

    task.on_teleop_buttons(_buttons(left=True, right=True), 1.1)
    task.on_left_cartesian_command(_pose(0.1), 1.1)
    task.on_right_cartesian_command(_pose(-0.1), 1.1)
    assert task.compute(_state(1.1)) is not None
    assert ik.frame_poses.call_count == 1

    task.on_teleop_buttons(_buttons(left=True, right=False), 1.2)
    assert task.compute(_state(1.2)) is None
    assert not task.is_active()


def test_bimanual_timeout_clears_both_sides_and_reengagement_recaptures(
    mocker: MockerFixture,
) -> None:
    ik = _ik(mocker)
    task = QuestTeleopIKTask(
        "quest",
        _config(
            (
                _binding("left", "left_tool"),
                _binding("right", "right_tool"),
            ),
            timeout=0.2,
        ),
        ik=ik,
    )
    task.on_teleop_buttons(_buttons(left=True, right=True), 1.0)
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    task.on_right_cartesian_command(_pose(-0.1), 1.0)
    assert task.compute(_state(1.0)) is not None

    task.on_left_cartesian_command(_pose(0.2), 1.25)
    assert task.compute(_state(1.25)) is None
    assert not task.is_active()

    task.on_teleop_buttons(_buttons(), 1.3)
    task.on_teleop_buttons(_buttons(left=True, right=True), 1.4)
    task.on_left_cartesian_command(_pose(0.2), 1.4)
    task.on_right_cartesian_command(_pose(-0.2), 1.4)
    assert task.compute(_state(1.4)) is not None
    assert ik.frame_poses.call_count == 2


def test_bimanual_step_contains_both_targets_and_grippers(
    mocker: MockerFixture,
) -> None:
    ik = _ik(mocker)
    task = QuestTeleopIKTask(
        "quest",
        _config(
            (
                _binding("left", "left_tool", "robot/left_gripper"),
                _binding("right", "right_tool", "robot/right_gripper"),
            )
        ),
        ik=ik,
    )
    task.on_teleop_buttons(
        _buttons(left=True, right=True, left_trigger=0.25, right_trigger=0.75),
        1.0,
    )
    task.on_left_cartesian_command(_pose(0.1), 1.0)
    task.on_right_cartesian_command(_pose(-0.1), 1.0)

    output = task.compute(_state())

    assert output is not None
    assert set(ik.step_frame_targets.call_args.kwargs["frame_targets"]) == {
        "left_tool",
        "right_tool",
    }
    assert output.joint_names == [
        "robot/left",
        "robot/right",
        "robot/left_gripper",
        "robot/right_gripper",
    ]
    assert output.positions is not None
    assert output.positions[2:] == pytest.approx([0.75, 0.25], abs=0.01)


def test_factory_rejects_gripper_missing_from_hardware(mocker: MockerFixture) -> None:
    mocker.patch.object(pose_target_module, "PinkIK")
    cfg = TaskConfig(
        name="quest",
        type="quest_teleop_ik",
        joint_names=["robot/left", "robot/right"],
        params={
            "robot_model": _robot_model(),
            "bindings": [
                {
                    "hand": "left",
                    "target_frame": "left_tool",
                    "gripper_joint": "robot/missing_gripper",
                }
            ],
        },
    )

    with pytest.raises(ValueError, match="unknown gripper"):
        create_task(
            cfg,
            hardware={"robot": SimpleNamespace(joint_names=["robot/left", "robot/right"])},
        )


def test_factory_constructs_plain_pink_backend_by_default(mocker: MockerFixture) -> None:
    init = mocker.patch.object(PinkIK, "__init__", return_value=None)
    mocker.patch.object(PinkIK, "validate_frame_targets")
    cfg = TaskConfig(
        name="quest",
        type="quest_teleop_ik",
        joint_names=["robot/left", "robot/right"],
        params={
            "robot_model": _robot_model(),
            "bindings": [{"hand": "left", "target_frame": "left_tool"}],
        },
    )

    task = create_task(cfg, hardware={})

    assert type(task._ik) is PinkIK
    assert task._config.max_joint_velocity_rad_s == 1.0
    init.assert_called_once_with(task._config.pink)


def test_factory_constructs_fresh_custom_backend_for_each_task() -> None:
    _CustomPinkIK.instances.clear()
    cfg = TaskConfig(
        name="quest",
        type="quest_teleop_ik",
        joint_names=["robot/left", "robot/right"],
        params={
            "robot_model": _robot_model(),
            "bindings": [{"hand": "left", "target_frame": "left_tool"}],
            "ik_backend_type": _CustomPinkIK,
        },
    )

    first = create_task(cfg, hardware={})
    second = create_task(cfg, hardware={})

    assert len(_CustomPinkIK.instances) == 2
    assert first._ik is _CustomPinkIK.instances[0]
    assert second._ik is _CustomPinkIK.instances[1]
    assert first._ik is not second._ik
    assert _CustomPinkIK.instances[0].received_config == first._config.pink
