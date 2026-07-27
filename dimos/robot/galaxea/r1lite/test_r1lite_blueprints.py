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

from dimos.control.components import HardwareType
from dimos.control.coordinator import ControlCoordinator
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.std_msgs.String import String
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


def _typed_pins(blueprint: Any) -> dict[tuple[str, type], str]:
    # Exact type identity: a different class with the same name must fail,
    # because DimOS stream matching keys on the actual class.
    return {
        (name, typ): str(getattr(t.topic, "topic", t.topic))
        for (name, typ), t in blueprint.transport_map.items()
    }


def _coordinator_kwargs(blueprint: Any) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is ControlCoordinator)


def test_coordinator_authority_is_exactly_the_commandable_joints() -> None:
    hardware = _coordinator_kwargs(r1lite_coordinator)["hardware"]
    by_id = {h.hardware_id: h for h in hardware}
    assert set(by_id) == {"r1lite", "chassis"}
    assert by_id["r1lite"].hardware_type == HardwareType.WHOLE_BODY
    assert by_id["r1lite"].joints == cfg.R1LITE_ARM_JOINTS
    assert len(by_id["r1lite"].joints) == 12
    assert by_id["chassis"].hardware_type == HardwareType.BASE
    assert by_id["chassis"].joints == ["chassis/vx", "chassis/vy", "chassis/wz"]
    assert all("torso" not in j for h in hardware for j in h.joints)
    assert by_id["r1lite"].adapter_type == "transport_lcm"
    assert by_id["chassis"].adapter_type == "transport_lcm"


def test_tasks_match_authority_with_no_duplicate_owners() -> None:
    tasks = _coordinator_kwargs(r1lite_coordinator)["tasks"]
    by_name = {t.name: t for t in tasks}
    assert set(by_name) == {"servo_r1lite", "vel_chassis"}
    assert by_name["servo_r1lite"].type == "servo"
    assert by_name["servo_r1lite"].joint_names == cfg.R1LITE_ARM_JOINTS
    assert by_name["servo_r1lite"].priority == 10
    assert by_name["vel_chassis"].type == "velocity"
    assert by_name["vel_chassis"].joint_names == ["chassis/vx", "chassis/vy", "chassis/wz"]
    assert by_name["vel_chassis"].priority == 10
    claimed: list[str] = []
    for t in tasks:
        claimed.extend(t.joint_names)
    assert len(claimed) == len(set(claimed))


def test_full_transport_inventory_is_pinned() -> None:
    assert _typed_pins(r1lite_control_base()) == {
        ("arming", String): "/r1lite/arming",
        ("cmd_vel", Twist): "/chassis/cmd_vel",
        ("connection_status", String): "/r1lite/connection_status",
        ("gripper_left_command", JointState): "/r1lite/gripper_left_command",
        ("gripper_left_state", JointState): "/r1lite/gripper_left_state",
        ("gripper_right_command", JointState): "/r1lite/gripper_right_command",
        ("gripper_right_state", JointState): "/r1lite/gripper_right_state",
        ("head_left_color", Image): "/r1lite/head_left_color",
        ("head_right_color", Image): "/r1lite/head_right_color",
        ("imu_chassis", Imu): "/r1lite/imu",
        ("imu_torso", Imu): "/r1lite/imu_torso",
        ("joint_command", JointState): "/r1lite/joint_command",
        ("motor_command", MotorCommandArray): "/r1lite/motor_command",
        ("motor_states", JointState): "/r1lite/motor_states",
        ("odom", PoseStamped): "/chassis/odom",
        ("torso_states", JointState): "/r1lite/torso_states",
        ("twist_command", Twist): "/cmd_vel",
        ("wrist_left_color", Image): "/r1lite/wrist_left_color",
        ("wrist_left_depth", Image): "/r1lite/wrist_left_depth",
        ("wrist_right_color", Image): "/r1lite/wrist_right_color",
        ("wrist_right_depth", Image): "/r1lite/wrist_right_depth",
    }


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
    assert _typed_pins(r1lite_keyboard_teleop) == {("cmd_vel", Twist): "/cmd_vel"}


def test_coordinator_listens_on_the_shared_teleop_topic() -> None:
    pins = _pins(r1lite_control_base())
    assert pins["twist_command"] == "/cmd_vel"
