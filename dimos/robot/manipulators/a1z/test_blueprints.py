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


def test_keyboard_teleop_wires_hardware_manipulation_and_trajectory_priority(
    monkeypatch,
) -> None:
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
            tasks = coordinator["tasks"]
            trajectory = next(task for task in tasks if task.type == "trajectory")
            eef_teleop = next(task for task in tasks if task.type == "eef_twist")
            assert trajectory.priority > eef_teleop.priority
            assert _module_kwargs(blueprint, ManipulationModule)
    finally:
        importlib.reload(teleop)
