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

"""Standalone gripper blueprints — the reference for a gripper with no arm.

This is the shape the H100 lands in (GRIPPER-SPEC 8): a ``GRIPPER`` component
whose joints are all gripper joints, driven by its own ``{hardware_id}_gripper``
task. The mock mimics an H100-like device — six joints on a dimensionless
``0-100`` firmware scale — so the whole path runs without hardware.
"""

from __future__ import annotations

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_gripper_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig

_mock_hand_hw = HardwareComponent(
    hardware_id="hand",
    hardware_type=HardwareType.GRIPPER,
    all_joints=make_gripper_joints("hand", 6),
    gripper_dof=6,
    adapter_type="mock",
    # The mock declares an H100-like scale; a real device declares its own
    # range in its adapter and this kwarg disappears (R13).
    adapter_kwargs={"limits": (0.0, 100.0)},
)

coordinator_gripper_mock = ControlCoordinator.blueprint(
    hardware=[_mock_hand_hw],
    tasks=[
        TaskConfig(
            name="hand_gripper",
            type="gripper",
            joint_names=_mock_hand_hw.gripper_joints,
            priority=20,
            params={
                # A multi-joint gripper MUST declare its grasp posture: joint
                # limits describe travel, not grasping (R19a). These stand in
                # for a vendor's documented gesture until the H100's arrive.
                "reference_pose": [80.0, 20.0, 50.0, 80.0, 20.0, 50.0],
            },
        ),
    ],
)
