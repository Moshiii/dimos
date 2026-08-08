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

"""Keyboard teleop for a standalone gripper — the hardware witness blueprint.

Drives GRIPPER-SPEC 8's standalone path end to end on an xArm we own: its
gripper is exposed as a one-joint standalone device over its OWN connection
(test vehicle; in production an integrated gripper rides its arm's adapter,
R1). The arm is never commanded, enabled, or mode-switched.

Run:
    XARM6_IP=<controller ip> uv run dimos run keyboard-teleop-gripper-xarm

then `[` opens and `]` closes. Expect the startup log to show the task
resolving `limits=[(0.0, 850.0)]` from the standalone adapter. There is no
mock fallback: without an IP, startup fails loudly.
"""

from __future__ import annotations

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_gripper_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.teleop.keyboard.keyboard_teleop_module import KeyboardTeleopModule

_xarm_hand_hw = HardwareComponent(
    hardware_id="hand",
    hardware_type=HardwareType.GRIPPER,
    all_joints=make_gripper_joints("hand"),
    gripper_dof=1,
    adapter_type="xarm_gripper",
    address=global_config.xarm6_ip or global_config.xarm7_ip,
)

keyboard_teleop_gripper_xarm = autoconnect(
    KeyboardTeleopModule.blueprint(),
    ControlCoordinator.blueprint(
        hardware=[_xarm_hand_hw],
        tasks=[
            TaskConfig(
                name="hand_gripper",
                type="gripper",
                joint_names=_xarm_hand_hw.gripper_joints,
                priority=20,
            ),
        ],
    ),
)
