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

"""Mock standalone gripper for unit tests and blueprints without hardware."""

from __future__ import annotations

from dimos.hardware.manipulators.spec import ControlMode, JointLimits


class MockGripperAdapter:
    """Fake standalone gripper implementing the ``GripperAdapter`` protocol.

    In-memory state, instant moves. ``limits`` sets one ``(lo, hi)`` applied to
    every joint so a test can mimic any vendor scale — ``(0.0, 100.0)`` for an
    H100-like device, ``(0.0, 0.085)`` for a metric jaw.
    """

    def __init__(
        self,
        dof: int = 1,
        limits: tuple[float, float] = (0.0, 1.0),
        initial_positions: list[float] | None = None,
        **_: object,
    ) -> None:
        if dof < 1:
            raise ValueError(f"MockGripperAdapter needs at least one joint (got {dof})")
        self._dof = dof
        self._limits = limits
        self._positions = (
            list(initial_positions) if initial_positions is not None else [limits[0]] * dof
        )
        if len(self._positions) != dof:
            raise ValueError(
                f"initial_positions has {len(self._positions)} values for {dof} joints"
            )
        self._connected = False
        self._enabled = False
        self._control_mode = ControlMode.POSITION

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        return self.write_enable(False)

    def write_enable(self, enable: bool) -> bool:
        self._enabled = enable
        return True

    # ---------------------------------------------------- identity and range

    def get_dof(self) -> int:
        """This device's joints — a gripper has no arm to be ambiguous with."""
        return self._dof

    def get_limits(self) -> JointLimits:
        lo, hi = self._limits
        return JointLimits(
            position_lower=[lo] * self._dof,
            position_upper=[hi] * self._dof,
            velocity_max=[0.0] * self._dof,
        )

    # ---------------------------------------------------------- control mode

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Accept position modes only; the refusal keeps the velocity branch
        of ``ConnectedHardware`` unreachable (R4a/R22)."""
        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            return False
        self._control_mode = mode
        return True

    # ---------------------------------------------------------- joint arrays

    def read_joint_positions(self) -> list[float]:
        return self._positions.copy()

    def read_joint_velocities(self) -> list[float]:
        return [0.0] * self._dof

    def read_joint_efforts(self) -> list[float]:
        return [0.0] * self._dof

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        if len(positions) != self._dof:
            return False
        self._positions = list(positions)
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Defined refusal — grippers are not velocity-controlled (R4a/R22)."""
        return False
