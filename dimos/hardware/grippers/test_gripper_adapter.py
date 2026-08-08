# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The GripperAdapter contract: parity with ManipulatorAdapter, and the mock.

The parity test is the drift guard GRIPPER-SPEC 8.3 mandates. GripperAdapter
restates part of ManipulatorAdapter's contract in a second file; Python's
structural typing would let the two disagree without any error ever firing —
the exact two-authorities shape 3.5 is about. This test makes divergence a CI
failure instead of a silent fact.
"""

from __future__ import annotations

import inspect

import pytest

from dimos.hardware.grippers.spec import GripperAdapter
from dimos.hardware.manipulators.spec import ControlMode, ManipulatorAdapter

# The agreed surface (GRIPPER-SPEC 8.1). Adding or removing a protocol method
# without updating the spec — and this list — is a reviewable act, not a drift.
_AGREED_SURFACE = {
    "connect",
    "disconnect",
    "is_connected",
    "activate",
    "deactivate",
    "write_enable",
    "get_dof",
    "get_limits",
    "set_control_mode",
    "read_joint_positions",
    "read_joint_velocities",
    "read_joint_efforts",
    "write_joint_positions",
    "write_joint_velocities",
}


def _methods(proto: type) -> dict[str, inspect.Signature]:
    return {
        name: inspect.signature(fn)
        for name, fn in inspect.getmembers(proto, inspect.isfunction)
        if not name.startswith("_")
    }


class TestSignatureParity:
    def test_gripper_adapter_is_a_strict_subset_of_manipulator_adapter(self) -> None:
        """Every GripperAdapter method exists on ManipulatorAdapter, byte-identical.

        One array contract, declared once, checked mechanically (8.3). If this
        fails after a ManipulatorAdapter change, update GripperAdapter — and
        the spec — in the same commit.
        """
        gripper = _methods(GripperAdapter)
        manipulator = _methods(ManipulatorAdapter)

        missing = sorted(set(gripper) - set(manipulator))
        assert not missing, f"GripperAdapter declares methods ManipulatorAdapter lacks: {missing}"

        for name, signature in gripper.items():
            assert signature == manipulator[name], (
                f"signature drift on {name!r}:\n"
                f"  GripperAdapter    : {signature}\n"
                f"  ManipulatorAdapter: {manipulator[name]}"
            )

    def test_the_surface_is_exactly_what_the_spec_agreed(self) -> None:
        assert set(_methods(GripperAdapter)) == _AGREED_SURFACE

    def test_get_gripper_dof_is_deliberately_absent(self) -> None:
        """The component's gripper_dof is the authoritative count (R7); inside a
        gripper-only protocol get_dof() answers the question itself (R8)."""
        assert "get_gripper_dof" not in _methods(GripperAdapter)


class TestMockConformance:
    def test_satisfies_the_protocol(self) -> None:
        from dimos.hardware.grippers.mock.adapter import MockGripperAdapter

        assert isinstance(MockGripperAdapter(dof=6), GripperAdapter)

    def test_registry_builds_it(self) -> None:
        from dimos.hardware.grippers.registry import gripper_adapter_registry

        adapter = gripper_adapter_registry.create("mock", dof=6, limits=(0.0, 100.0))
        assert adapter.connect()
        assert adapter.get_dof() == 6

    @pytest.mark.parametrize("limits", [(0.0, 100.0), (0.0, 0.085), (0.0, 1.0)])
    def test_declares_its_range_and_moves_in_it(self, limits: tuple[float, float]) -> None:
        """R13 for a standalone device: lengths equal get_dof(), units are its own."""
        from dimos.hardware.grippers.mock.adapter import MockGripperAdapter

        adapter = MockGripperAdapter(dof=6, limits=limits)
        adapter.connect()

        declared = adapter.get_limits()
        assert len(declared.position_lower) == 6
        assert len(declared.position_upper) == 6
        assert len(declared.velocity_max) == 6
        assert declared.position_upper[-1] == limits[1]

        target = [limits[1]] * 6
        assert adapter.write_joint_positions(target)
        assert adapter.read_joint_positions() == target

    def test_reads_stay_index_aligned(self) -> None:
        from dimos.hardware.grippers.mock.adapter import MockGripperAdapter

        adapter = MockGripperAdapter(dof=3)
        n = len(adapter.read_joint_positions())
        assert len(adapter.read_joint_velocities()) == n
        assert len(adapter.read_joint_efforts()) == n

    def test_refusals_are_refusals(self) -> None:
        """Velocity is a defined no; the mode gate keeps the branch unreachable."""
        from dimos.hardware.grippers.mock.adapter import MockGripperAdapter

        adapter = MockGripperAdapter(dof=6)
        assert adapter.write_joint_velocities([0.0] * 6) is False
        assert adapter.set_control_mode(ControlMode.VELOCITY) is False
        assert adapter.set_control_mode(ControlMode.SERVO_POSITION) is True
