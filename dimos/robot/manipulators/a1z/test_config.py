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

from dimos.core.global_config import global_config
from dimos.robot.manipulators.a1z.config import A1Z_G1Z_MODEL_PATH, a1z_hardware


def test_real_hardware_uses_stable_can_interface_and_lazy_model(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "can_port", None)

    hardware = a1z_hardware(dynamics_urdf_path=A1Z_G1Z_MODEL_PATH)

    assert hardware.adapter_type == "galaxea_a1z"
    assert hardware.address == "a1zcan"
    assert hardware.adapter_kwargs["gripper"] is True
    assert hardware.adapter_kwargs["urdf_path"] is A1Z_G1Z_MODEL_PATH
    assert hardware.gripper_open_position == 0.1
    assert hardware.gripper_closed_position == 0.0


def test_explicit_can_interface_is_forwarded(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "can_port", "can7")

    hardware = a1z_hardware()

    assert hardware.adapter_type == "galaxea_a1z"
    assert hardware.address == "can7"


def test_simulation_uses_mock_hardware(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    monkeypatch.setattr(global_config, "can_port", "can7")

    hardware = a1z_hardware()

    assert hardware.adapter_type == "mock"
    assert hardware.address is None


def test_mock_without_address_requires_explicit_can_port(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "can_port", None)

    hardware = a1z_hardware(mock_without_address=True)

    assert hardware.adapter_type == "mock"
    assert hardware.address is None
