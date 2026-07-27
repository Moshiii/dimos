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

"""Composition tests for the R1 Lite blueprints.

No hardware, no build: assertions run on the declarative blueprint
structure so authority, wiring, and topic pins are checked on every
commit.
"""

from __future__ import annotations

from typing import Any

from dimos.control.coordinator import ControlCoordinator
from dimos.robot.galaxea.r1lite import config as cfg
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import (
    r1lite_control_base,
    r1lite_coordinator,
)
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_keyboard_teleop import (
    r1lite_keyboard_teleop,
)
from dimos.robot.galaxea.r1lite.connection import R1LiteConnection


def _pins(blueprint: Any) -> dict[str, str]:
    return {
        name: str(getattr(t.topic, "topic", t.topic))
        for (name, _), t in blueprint.transport_map.items()
    }


def _coordinator_kwargs(blueprint: Any) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is ControlCoordinator)


def test_coordinator_authority_is_exactly_the_commandable_joints() -> None:
    hardware = _coordinator_kwargs(r1lite_coordinator)["hardware"]
    by_id = {h.hardware_id: h for h in hardware}
    assert set(by_id) == {"r1lite", "chassis"}
    assert by_id["r1lite"].joints == cfg.R1LITE_ARM_JOINTS
    assert len(by_id["r1lite"].joints) == 12
    assert all("torso" not in j for h in hardware for j in h.joints)
    assert by_id["r1lite"].adapter_type == "transport_lcm"
    assert by_id["chassis"].adapter_type == "transport_lcm"


def test_tasks_match_authority_with_no_duplicate_owners() -> None:
    tasks = _coordinator_kwargs(r1lite_coordinator)["tasks"]
    by_name = {t.name: t for t in tasks}
    assert set(by_name) == {"servo_r1lite", "vel_chassis"}
    assert by_name["servo_r1lite"].joint_names == cfg.R1LITE_ARM_JOINTS
    assert by_name["servo_r1lite"].priority == 10
    claimed: list[str] = []
    for t in tasks:
        claimed.extend(t.joint_names)
    assert len(claimed) == len(set(claimed))


def test_exactly_one_connection_and_one_coordinator() -> None:
    modules = [atom.module for atom in r1lite_coordinator.blueprints]
    assert modules.count(R1LiteConnection) == 1
    assert modules.count(ControlCoordinator) == 1


def test_adapter_topic_pins_match_hardware_ids() -> None:
    # The transport adapters derive their topics from hardware_id; the
    # stream pins must land on exactly those topics.
    pins = _pins(r1lite_control_base())
    assert pins["motor_states"] == "/r1lite/motor_states"
    assert pins["motor_command"] == "/r1lite/motor_command"
    assert pins["imu_chassis"] == "/r1lite/imu"
    assert pins["cmd_vel"] == "/chassis/cmd_vel"
    assert pins["odom"] == "/chassis/odom"


def test_arming_topics_pinned_for_preflight() -> None:
    # preflight.py reads these exact topics; the blueprint owns keeping
    # them aligned with the r1lite config.
    pins = _pins(r1lite_control_base())
    assert pins["arming"] == cfg.ARMING_TOPIC
    assert pins["connection_status"] == "/r1lite/connection_status"


def test_torso_stream_pinned_read_only() -> None:
    pins = _pins(r1lite_control_base())
    assert pins["torso_states"] == "/r1lite/torso_states"
    # No torso command stream exists anywhere in the composition.
    assert "torso_command" not in pins


def test_keyboard_teleop_is_a_thin_client() -> None:
    modules = [atom.module for atom in r1lite_keyboard_teleop.blueprints]
    assert ControlCoordinator not in modules
    assert R1LiteConnection not in modules
    assert len(modules) == 1
    pins = _pins(r1lite_keyboard_teleop)
    assert pins["cmd_vel"] == "/cmd_vel"


def test_coordinator_listens_on_the_shared_teleop_topic() -> None:
    pins = _pins(r1lite_control_base())
    assert pins["twist_command"] == "/cmd_vel"
