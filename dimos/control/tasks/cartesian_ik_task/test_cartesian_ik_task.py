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

"""Behavior tests for the absolute Cartesian Pink task leaf."""

from pathlib import Path

from pytest_mock import MockerFixture

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.cartesian_ik_task.cartesian_ik_task import (
    CartesianIKTask,
    CartesianIKTaskConfig,
)
import dimos.control.tasks.pose_target_ik as pose_target_module
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


def _config() -> CartesianIKTaskConfig:
    return CartesianIKTaskConfig(
        joint_names=("arm/joint",),
        robot_model=RobotModelConfig(
            name="arm",
            model_path=Path("fake.urdf"),
            joint_names=["model_joint"],
            joint_name_mapping={"arm/joint": "model_joint"},
        ),
        target_frames=("tool",),
        max_joint_delta_deg=10.0,
    )


def test_cartesian_leaf_maps_absolute_pose_to_configured_frame(
    mocker: MockerFixture,
) -> None:
    ik = mocker.patch.object(pose_target_module, "PinkIK").return_value
    ik.step_frame_targets.return_value = JointState(name=["arm/joint"], position=[0.01])
    task = CartesianIKTask("cartesian", _config())
    target = PoseStamped(position=Vector3(0.4, 0.2, 0.1), frame_id="world")

    task.on_cartesian_command(target, t_now=2.0)
    output = task.compute(
        CoordinatorState(
            joints=JointStateSnapshot(joint_positions={"arm/joint": 0.0}),
            t_now=2.0,
            dt=0.01,
        )
    )

    assert output is not None
    assert ik.step_frame_targets.call_args.kwargs["frame_targets"] == {"tool": target}


def test_cartesian_leaf_clears_after_timeout(mocker: MockerFixture) -> None:
    ik = mocker.patch.object(pose_target_module, "PinkIK").return_value
    task = CartesianIKTask("cartesian", _config())
    task.on_cartesian_command(PoseStamped(), t_now=1.0)

    output = task.compute(
        CoordinatorState(
            joints=JointStateSnapshot(joint_positions={"arm/joint": 0.0}),
            t_now=2.0,
            dt=0.01,
        )
    )

    assert output is None
    assert not task.is_active()
    ik.step_frame_targets.assert_not_called()


def test_cartesian_clear_reseeds_command_from_feedback(mocker: MockerFixture) -> None:
    ik = mocker.patch.object(pose_target_module, "PinkIK").return_value
    ik.step_frame_targets.return_value = JointState(name=["arm/joint"], position=[0.1])
    task = CartesianIKTask("cartesian", _config())
    state = CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"arm/joint": 0.0}),
        t_now=1.0,
        dt=0.01,
    )
    task.on_cartesian_command(PoseStamped(), t_now=1.0)
    assert task.compute(state) is not None

    task.clear()
    task.on_cartesian_command(PoseStamped(), t_now=1.1)
    ik.step_frame_targets.return_value = JointState(name=["arm/joint"], position=[0.01])
    assert task.compute(state) is not None

    assert ik.step_frame_targets.call_args.kwargs["command_state"].position == [0.0]
