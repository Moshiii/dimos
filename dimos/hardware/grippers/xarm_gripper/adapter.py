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

"""The xArm gripper as a standalone device — a hardware witness for GRIPPER-SPEC 8.

**Test vehicle, not a production shape.** In production the xArm's gripper is a
capability of its arm's adapter (R1) and rides the arm's joint array. This
adapter exists so the *standalone* path — GRIPPER component → grippers registry
→ ``GripperAdapter`` → wrapper → task — can be verified on hardware we own
before the H100 arrives: it opens its **own** connection to the controller and
exposes only the gripper, as a one-joint device. The arm is never commanded,
enabled, or mode-switched.
"""

from __future__ import annotations

from xarm.wrapper import XArmAPI

from dimos.hardware.manipulators.spec import ControlMode, JointLimits
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The gripper's own scale: the dimensionless 0-850 its SDK call takes. The same
# range the integrated path declares in manipulators/xarm/adapter.py — restated
# here because this witness deliberately shares no code with the arm adapter.
_GRIPPER_MIN = 0.0
_GRIPPER_MAX = 850.0


class XArmGripperAdapter:
    """One-joint standalone ``GripperAdapter`` over the xArm SDK's gripper calls."""

    def __init__(self, address: str | None = None, dof: int = 1, **_: object) -> None:
        if not address:
            raise ValueError(
                "xarm_gripper needs the controller IP (set XARM6_IP / XARM7_IP)"
            )
        if dof != 1:
            raise ValueError(f"the xArm gripper is one joint (got dof={dof})")
        self._ip = address
        self._arm: XArmAPI | None = None
        self._control_mode = ControlMode.POSITION
        self._gripper_enabled = False
        # Last good reading, held across transient SDK read failures so the
        # published state doesn't glitch to an arbitrary value.
        self._last_position = 0.0

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> bool:
        """Open our own connection. The arm's mode and state are left alone."""
        try:
            self._arm = XArmAPI(self._ip)
            self._arm.connect()
            if not self._arm.connected:
                logger.error("xarm_gripper: controller at %s not reachable", self._ip)
                return False
            return True
        except Exception as e:
            logger.error("xarm_gripper: failed to connect to %s: %s", self._ip, e)
            return False

    def disconnect(self) -> None:
        if self._arm:
            self._arm.disconnect()
            self._arm = None

    def is_connected(self) -> bool:
        return self._arm is not None and bool(self._arm.connected)

    def activate(self) -> bool:
        """Enable the gripper. Deliberately no arm motion_enable/set_mode."""
        return self.write_enable(True)

    def deactivate(self) -> bool:
        """Nothing to wind down: the gripper holds, and the arm is not ours."""
        return True

    def write_enable(self, enable: bool) -> bool:
        if not self._arm:
            return False
        code: int = self._arm.set_gripper_enable(enable)
        self._gripper_enabled = enable and code == 0
        return code == 0

    # ---------------------------------------------------- identity and range

    def get_dof(self) -> int:
        """One joint — this device is only the gripper."""
        return 1

    def get_limits(self) -> JointLimits:
        """The gripper's own 0-850 scale (R13). No SDK rate limit exposed."""
        return JointLimits(
            position_lower=[_GRIPPER_MIN],
            position_upper=[_GRIPPER_MAX],
            velocity_max=[0.0],
        )

    # ---------------------------------------------------------- control mode

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Position modes only; the refusal keeps the velocity branch closed."""
        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            return False
        self._control_mode = mode
        return True

    # ---------------------------------------------------------- joint arrays

    def read_joint_positions(self) -> list[float]:
        """The gripper position in its own 0-850 scale, unconverted."""
        if self._arm:
            code, pos = self._arm.get_gripper_position()
            if code == 0 and pos is not None:
                self._last_position = float(pos)
        return [self._last_position]

    def read_joint_velocities(self) -> list[float]:
        return [0.0]

    def read_joint_efforts(self) -> list[float]:
        return [0.0]

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        """Command the gripper in its own scale. No conversion — that is the test."""
        if not self._arm or len(positions) != 1:
            return False
        if not self._gripper_enabled and not self.write_enable(True):
            return False
        code: int = self._arm.set_gripper_position(positions[0], wait=False)
        return code == 0

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Defined refusal — grippers are not velocity-controlled (R4a/R22)."""
        return False
