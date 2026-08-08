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

import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.global_config import global_config
from dimos.robot.manipulators.piper.blueprints import basic
from dimos.robot.manipulators.piper.config import (
    PIPER_HOME_JOINTS,
    PIPER_ROBOT_MODEL_PATH,
    PIPER_SIM_GRIPPER_OPEN,
    make_piper_sim_hardware,
)
from dimos.simulation.providers import SimulationBinding, SimulationRequest


def test_piper_sim_hardware_matches_provider_control_contract() -> None:
    hardware = make_piper_sim_hardware(
        Path("/tmp/pimsim-piper"),
        adapter_type="test_adapter",
    )

    assert hardware.adapter_type == "test_adapter"
    assert hardware.address == "/tmp/pimsim-piper"
    assert hardware.joints == [f"arm/joint{index}" for index in range(1, 7)]
    assert hardware.gripper_joints == ["arm/gripper"]
    assert hardware.gripper_open_position == PIPER_SIM_GRIPPER_OPEN
    assert hardware.gripper_closed_position == 0.0
    assert hardware.adapter_kwargs == {"initial_positions": PIPER_HOME_JOINTS}


def test_existing_piper_blueprint_requests_selected_simulation_provider(
    monkeypatch: pytest.MonkeyPatch,
    mocker,
) -> None:
    binding = SimulationBinding(
        backend=Blueprint(blueprints=()),
        adapter_type="sim_mujoco",
        adapter_address=Path("/tmp/pimsim-piper"),
    )
    provider = mocker.Mock()
    provider.build.return_value = binding
    load_provider = mocker.patch.object(
        basic,
        "load_simulation_provider",
        return_value=provider,
    )
    monkeypatch.setattr(global_config, "simulation_provider", "pimsim")
    monkeypatch.setattr(global_config, "scene_package", "tabletop-test")

    assert basic._resolve_piper_simulation() is binding
    load_provider.assert_called_once_with("pimsim")
    provider.build.assert_called_once_with(
        SimulationRequest(
            robot_model="agilex_piper",
            model_path=PIPER_ROBOT_MODEL_PATH,
            scene_package="tabletop-test",
        )
    )
