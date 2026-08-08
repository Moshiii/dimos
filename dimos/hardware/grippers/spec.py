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

"""Gripper adapter specification: the protocol for standalone gripper devices.

A first-party integrated gripper — the jaw an arm ships with — is a capability
of that arm's SDK and stays inside its ``ManipulatorAdapter`` (GRIPPER-SPEC R1).
This protocol is for a gripper that is a **device in its own right**: its own
connection, its own driver, no arm — the H100 hand, or any third-party gripper.

``GripperAdapter`` is a deliberate **signature-subset** of ``ManipulatorAdapter``:
every method here exists there with the byte-identical signature, so
``ConnectedHardware`` wraps either kind unchanged (R25) and the two protocols
cannot drift apart — ``test_spec_parity.py`` fails CI the moment they disagree.

A protocol method requires a **defined answer**, not a capability. A refusal
(``False`` / zeros) is a valid answer; ``get_limits()`` is the one method that
may never refuse, because the gripper task converts with it (R13/R14a).
"""

from typing import Protocol, runtime_checkable

# Shared vocabulary, imported rather than restated: one declaration of what a
# control mode and a joint limit are (the pattern control/task.py already uses).
from dimos.hardware.manipulators.spec import ControlMode, JointLimits


@runtime_checkable
class GripperAdapter(Protocol):
    """Hardware IO for a standalone gripper.

    Implement this per device. Every joint value — read, written, or declared
    in limits — is in the unit **this adapter declares** through
    ``get_limits()`` (R12/R13): metres for a sliding jaw, or the vendor's own
    scale where the firmware is dimensionless (the H100's ``0-100`` per joint).
    Nothing above the hardware layer converts it.
    """

    # ------------------------------------------------------------ lifecycle

    def connect(self) -> bool:
        """Connect to the device. Returns True on success."""
        ...

    def disconnect(self) -> None:
        """Disconnect from the device."""
        ...

    def is_connected(self) -> bool:
        """Check if connected."""
        ...

    def activate(self) -> bool:
        """Prepare the device for commanded motion after ``connect()``.

        Run the vendor's activation sequence, enable torque — or return True
        if the device needs nothing.
        """
        ...

    def deactivate(self) -> bool:
        """Gracefully stop commanded motion before ``disconnect()``."""
        ...

    def write_enable(self, enable: bool) -> bool:
        """Enable or disable actuation. True if the device has no such concept.

        NOTE: kept for lifecycle symmetry with ``ManipulatorAdapter`` and for
        direct driver/test use; the coordinator itself always goes through
        ``activate()``. Flagged as a trim candidate in GRIPPER-SPEC 8.2.
        """
        ...

    # ---------------------------------------------------- identity and range

    def get_dof(self) -> int:
        """This device's actuated joint count — six for the H100, one for a jaw.

        Unlike an arm adapter (where ``get_dof()`` is arm-only, R8), a gripper
        has no arm to be ambiguous with: this is simply its joints. There is no
        ``get_gripper_dof()`` here — the component's ``gripper_dof`` is the
        authoritative count everywhere above the adapter (R7).
        """
        ...

    def get_limits(self) -> JointLimits:
        """Limits for every joint, length ``get_dof()``, in this device's units.

        **The one method that may never refuse.** This is the single
        authoritative declaration of the gripper's travel (R13): the gripper
        task reads it to convert normalized commands, and nothing above the
        adapter may hard-code an endpoint. ``velocity_max`` entries are ``0.0``
        where the firmware exposes no rate limit.
        """
        ...

    # ---------------------------------------------------------- control mode

    def set_control_mode(self, mode: ControlMode) -> bool:
        """Accept ``POSITION`` / ``SERVO_POSITION``; refuse everything else.

        The refusal is load-bearing: ``ConnectedHardware`` switches modes
        through this before writing, so returning False for ``VELOCITY`` keeps
        its velocity branch unreachable on a gripper device (R4a/R22).
        """
        ...

    # ------------------------------------------- the joint arrays (R4, R12)

    def read_joint_positions(self) -> list[float]:
        """Measured positions, one per joint, in this device's declared units.

        An **open-loop** device (no position feedback) MUST echo its last
        commanded target and say so in its docstring — zeros would make the
        task's ``get_position`` lie. This forfeits stall detection, which a
        feedback-less device never had.
        """
        ...

    def read_joint_velocities(self) -> list[float]:
        """Measured velocities, one per joint; ``0.0`` where unmeasured.

        Reads stay index-aligned with positions (R4a): callers zip positions,
        velocities and efforts by index.
        """
        ...

    def read_joint_efforts(self) -> list[float]:
        """Measured efforts, one per joint; ``0.0`` where unmeasured."""
        ...

    def write_joint_positions(
        self,
        positions: list[float],
        velocity: float = 1.0,
    ) -> bool:
        """Command every joint, in this device's declared units. Returns success.

        The array is the whole device — for a gripper there is no arm prefix.
        The values are already in the unit ``get_limits()`` declares; a second
        conversion here is the bug class GRIPPER-SPEC 3.5 measures.
        """
        ...

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        """Refuse (return False). Grippers are not velocity-controlled here.

        This is a **defined refusal, not a capability**: R22 fixes the gripper
        task in ``SERVO_POSITION`` and nothing in the system produces a gripper
        velocity (R4a). It is unreachable through a correct adapter — the mode
        gate above runs first — and exists because ``ConnectedHardware`` wraps
        ``ManipulatorAdapter | GripperAdapter`` and mypy checks every branch
        against that union. Do not implement velocity control behind it.
        """
        ...
