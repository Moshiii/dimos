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

import importlib
from typing import Any

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.a1z.blueprints import teleop


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def test_keyboard_teleop_uses_real_hardware_and_resolved_can_port(monkeypatch) -> None:
    try:
        with monkeypatch.context() as patch:
            patch.setattr(global_config, "simulation", "")
            patch.setattr(global_config, "can_port", "can7")

            blueprint = importlib.reload(teleop).keyboard_teleop_a1z

            coordinator = _module_kwargs(blueprint, ControlCoordinator)
            hardware = coordinator["hardware"]
            assert len(hardware) == 1
            assert hardware[0].adapter_type == "galaxea_a1z"
            assert hardware[0].address == "can7"
            assert coordinator["tick_rate"] == 100.0
            assert coordinator["publish_joint_state"] is True
            assert coordinator["joint_state_frame_id"] == "coordinator"
            assert [task.type for task in coordinator["tasks"]] == [
                "eef_twist",
                "servo",
                "trajectory",
            ]
            gripper = coordinator["tasks"][1]
            assert gripper.name == "servo_gripper"
            assert gripper.joint_names == ["arm/gripper"]
            trajectory = coordinator["tasks"][2]
            assert trajectory.name == "traj_arm"
            assert trajectory.joint_names == hardware[0].joints
            assert trajectory.priority > coordinator["tasks"][0].priority
            assert _module_kwargs(blueprint, ManipulationModule)
    finally:
        importlib.reload(teleop)
