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

"""Galaxea R1 Lite constants: joints, vendor topics, rates, arming matrix."""

from __future__ import annotations

from pathlib import Path

ROS_NODE_NAME = "dimos_r1lite_connection"

_ASSETS = Path(__file__).parent / "assets"
# Kinematic models derived from Galaxea's published R1 Lite description;
# provenance in each file's header comment.
R1LITE_LEFT_ARM_MODEL = _ASSETS / "r1lite_left_arm.urdf"
R1LITE_RIGHT_ARM_MODEL = _ASSETS / "r1lite_right_arm.urdf"

ARM_DOF = 6
TORSO_DOF = 4

# Command joint order for motor_states and motor_command: left then right.
LEFT_ARM_JOINTS: list[str] = [f"r1lite/left_arm_joint{i}" for i in range(1, ARM_DOF + 1)]
RIGHT_ARM_JOINTS: list[str] = [f"r1lite/right_arm_joint{i}" for i in range(1, ARM_DOF + 1)]
R1LITE_ARM_JOINTS: list[str] = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS


def strip_hardware_prefix(names: list[str]) -> list[str]:
    """Hardware joint names to URDF names: drop the leading namespace."""
    return [name.split("/", 1)[1] for name in names]


# The same joints in the URDF namespace, derived so the lists cannot drift.
LEFT_ARM_URDF_JOINTS: list[str] = strip_hardware_prefix(LEFT_ARM_JOINTS)
RIGHT_ARM_URDF_JOINTS: list[str] = strip_hardware_prefix(RIGHT_ARM_JOINTS)

# torso_joint4 is the fourth HDAS motor value the URDF's 3-joint linkage
# model does not expose. Feedback only; never commanded.
R1LITE_TORSO_JOINTS: list[str] = [f"r1lite/torso_joint{i}" for i in range(1, TORSO_DOF + 1)]

GRIPPER_OPEN = 100.0
GRIPPER_CLOSED = 0.0

CMD_ARM_LEFT = "/motion_target/target_joint_state_arm_left"
CMD_ARM_RIGHT = "/motion_target/target_joint_state_arm_right"
CMD_GRIPPER_LEFT = "/motion_target/target_position_gripper_left"
CMD_GRIPPER_RIGHT = "/motion_target/target_position_gripper_right"
CMD_CHASSIS_SPEED = "/motion_target/target_speed_chassis"
CMD_CHASSIS_ACC_LIMIT = "/motion_target/chassis_acc_limit"
CMD_BRAKE_MODE = "/motion_target/brake_mode"
CMD_TORSO_JOINT = "/motion_target/target_joint_state_torso"
CMD_TORSO_SPEED = "/motion_target/target_speed_torso"

FB_ARM_LEFT = "/hdas/feedback_arm_left"
FB_ARM_RIGHT = "/hdas/feedback_arm_right"
FB_TORSO = "/hdas/feedback_torso"
FB_GRIPPER_LEFT = "/hdas/feedback_gripper_left"
FB_GRIPPER_RIGHT = "/hdas/feedback_gripper_right"
FB_CHASSIS = "/hdas/feedback_chassis"
FB_CHASSIS_SPEED = "/motion_control/chassis_speed"

# Publisher-count matrix for the two-phase ownership contract. Values are
# the maximum publisher count per phase; "arm" additionally requires each
# permitted publisher to be ROS_NODE_NAME. Torso command topics stay at
# zero in both phases: this connection never publishes torso commands.
ARMING_MATRIX: dict[str, tuple[int, int]] = {
    CMD_ARM_LEFT: (0, 1),
    CMD_ARM_RIGHT: (0, 1),
    CMD_GRIPPER_LEFT: (0, 1),
    CMD_GRIPPER_RIGHT: (0, 1),
    CMD_CHASSIS_SPEED: (0, 1),
    CMD_CHASSIS_ACC_LIMIT: (0, 1),
    CMD_BRAKE_MODE: (0, 1),
    CMD_TORSO_JOINT: (0, 0),
    CMD_TORSO_SPEED: (0, 0),
}

# Nominal feedback rates in Hz, measured during bring-up. The arming check
# requires at least half nominal. None means never measured: arming fails
# closed on that topic until a hardware session pins the value.
# chassis_speed runs at a quarter of the other feedback rates; measured on
# the robot with the vendor stack up and the chassis stationary.
FEEDBACK_NOMINAL_HZ: dict[str, float | None] = {
    FB_ARM_LEFT: 200.0,
    FB_ARM_RIGHT: 200.0,
    FB_TORSO: 488.0,
    FB_GRIPPER_LEFT: 200.0,
    FB_GRIPPER_RIGHT: 200.0,
    FB_CHASSIS: 200.0,
    FB_CHASSIS_SPEED: 50.0,
}

# Preflight-only vendor-health topics. FB_CHASSIS (wheel joint states) is
# checked before dimos starts to prove the vendor chassis node is alive,
# but the connection consumes no data from it, so it is not part of the
# connection's arming invariant. The arming invariant covers exactly the
# sources the connection consumes for control and safety decisions.
PREFLIGHT_ONLY_FEEDBACK: frozenset[str] = frozenset({FB_CHASSIS})

# The feedback sources whose freshness gates ARM and holds it.
ARMING_REQUIRED_FEEDBACK: frozenset[str] = frozenset(FEEDBACK_NOMINAL_HZ) - PREFLIGHT_ONLY_FEEDBACK

ARMING_TOPIC = "/r1lite/arming"
