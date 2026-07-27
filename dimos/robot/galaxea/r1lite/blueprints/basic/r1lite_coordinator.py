#!/usr/bin/env python3
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

"""R1 Lite ControlCoordinator: connection plus servo and chassis tasks.

The coordinator commands exactly the 12 arm joints; the torso is feedback
only and appears in no hardware component. The connection starts disarmed:
run scripts/r1lite_test/preflight.py to verify sole ownership and arm it
before any motion. Chassis software control needs RC mode 5.

    dimos run r1lite-coordinator
"""

from __future__ import annotations

from dimos.control.components import HardwareComponent, HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.std_msgs.String import String
from dimos.robot.galaxea.r1lite import config as cfg
from dimos.robot.galaxea.r1lite.connection import R1LiteConnection
from dimos.visualization.vis_module import vis_module

_CHASSIS_JOINTS = make_twist_base_joints("chassis")

_CAMERA_STREAMS = (
    "head_left_color",
    "head_right_color",
    "wrist_left_color",
    "wrist_left_depth",
    "wrist_right_color",
    "wrist_right_depth",
)


def r1lite_standard_tasks() -> list[TaskConfig]:
    return [
        TaskConfig(
            name="servo_r1lite",
            type="servo",
            joint_names=list(cfg.R1LITE_ARM_JOINTS),
            priority=10,
        ),
        TaskConfig(
            name="vel_chassis",
            type="velocity",
            joint_names=list(_CHASSIS_JOINTS),
            priority=10,
        ),
    ]


def r1lite_control_base(
    extra_tasks: list[TaskConfig] | None = None,
    connection_kwargs: dict[str, object] | None = None,
) -> Blueprint:
    """Connection plus coordinator with every stream pinned to its topic.

    The topic pins are load-bearing: the transport adapters build their own
    subscriptions from the hardware_id, and preflight.py reads the arming
    and status topics from the r1lite config.
    """
    transports: dict[tuple[str, type], object] = {
        # TransportWholeBodyAdapter endpoints (topics derive from hardware_id).
        ("motor_states", JointState): LCMTransport("/r1lite/motor_states", JointState),
        ("imu_chassis", Imu): LCMTransport("/r1lite/imu", Imu),
        ("motor_command", MotorCommandArray): LCMTransport(
            "/r1lite/motor_command", MotorCommandArray
        ),
        # TransportTwistAdapter endpoints.
        ("cmd_vel", Twist): LCMTransport("/chassis/cmd_vel", Twist),
        ("odom", PoseStamped): LCMTransport("/chassis/odom", PoseStamped),
        # Thin teleop clients publish here; the coordinator listens.
        ("twist_command", Twist): LCMTransport("/cmd_vel", Twist),
        # Operator arming path; preflight.py expects exactly these topics.
        ("arming", String): LCMTransport(cfg.ARMING_TOPIC, String),
        ("connection_status", String): LCMTransport("/r1lite/connection_status", String),
        # Read-only feedback and command passthroughs.
        ("torso_states", JointState): LCMTransport("/r1lite/torso_states", JointState),
        ("imu_torso", Imu): LCMTransport("/r1lite/imu_torso", Imu),
        ("joint_command", JointState): LCMTransport("/r1lite/joint_command", JointState),
        ("gripper_left_command", JointState): LCMTransport(
            "/r1lite/gripper_left_command", JointState
        ),
        ("gripper_right_command", JointState): LCMTransport(
            "/r1lite/gripper_right_command", JointState
        ),
        ("gripper_left_state", JointState): LCMTransport("/r1lite/gripper_left_state", JointState),
        ("gripper_right_state", JointState): LCMTransport(
            "/r1lite/gripper_right_state", JointState
        ),
    }
    for stream in _CAMERA_STREAMS:
        transports[(stream, Image)] = LCMTransport(f"/r1lite/{stream}", Image)

    return autoconnect(
        R1LiteConnection.blueprint(**(connection_kwargs or {})),
        ControlCoordinator.blueprint(
            hardware=[
                HardwareComponent(
                    hardware_id="r1lite",
                    hardware_type=HardwareType.WHOLE_BODY,
                    joints=list(cfg.R1LITE_ARM_JOINTS),
                    adapter_type="transport_lcm",
                ),
                HardwareComponent(
                    hardware_id="chassis",
                    hardware_type=HardwareType.BASE,
                    joints=list(_CHASSIS_JOINTS),
                    adapter_type="transport_lcm",
                ),
            ],
            tasks=[*r1lite_standard_tasks(), *(extra_tasks or [])],
        ),
    ).transports(transports)  # type: ignore[arg-type]


r1lite_coordinator = autoconnect(
    vis_module(viewer_backend=global_config.viewer),
    r1lite_control_base(),
)
