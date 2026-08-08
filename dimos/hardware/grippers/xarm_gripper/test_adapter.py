# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The witness adapter: the xArm gripper as a standalone one-joint device.

Everything here runs against a recording fake SDK; the point of the witness is
that the *same* adapter then runs unchanged against the real controller.
"""

from __future__ import annotations

import pytest

from dimos.hardware.grippers.spec import GripperAdapter
from dimos.hardware.grippers.xarm_gripper import adapter as xarm_gripper
from dimos.hardware.manipulators.spec import ControlMode


class _FakeXArmSDK:
    """Records gripper calls; screams if the arm is touched.

    The witness's one safety promise is that the arm is never commanded,
    enabled, or mode-switched — so the fake makes any arm call a test failure.
    """

    def __init__(self, *_: object, **__: object) -> None:
        self.connected = True
        self.gripper_enabled: bool | None = None
        self.sent: list[float] = []
        self._position = 0.0

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...

    # -- gripper surface --
    def set_gripper_enable(self, enable: bool) -> int:
        self.gripper_enabled = enable
        return 0

    def set_gripper_position(self, position: float, **__: object) -> int:
        self.sent.append(position)
        self._position = position
        return 0

    def get_gripper_position(self) -> tuple[int, float]:
        return 0, self._position

    # -- the arm: forbidden territory --
    def __getattr__(self, name: str) -> object:
        raise AssertionError(
            f"witness adapter touched the arm: called {name!r} — it must only "
            "use the gripper surface"
        )


@pytest.fixture
def adapter(monkeypatch: pytest.MonkeyPatch) -> xarm_gripper.XArmGripperAdapter:
    monkeypatch.setattr(xarm_gripper, "XArmAPI", _FakeXArmSDK)
    a = xarm_gripper.XArmGripperAdapter(address="192.168.1.210")
    assert a.connect()
    return a


class TestWitness:
    def test_satisfies_the_protocol(self, adapter: xarm_gripper.XArmGripperAdapter) -> None:
        assert isinstance(adapter, GripperAdapter)

    def test_registry_builds_it_as_the_coordinator_would(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.setattr(xarm_gripper, "XArmAPI", _FakeXArmSDK)
        from dimos.hardware.grippers.registry import gripper_adapter_registry

        built = gripper_adapter_registry.create(
            "xarm_gripper", dof=1, address="192.168.1.210", hardware_id="hand"
        )
        assert built.connect()
        assert built.get_dof() == 1

    def test_declares_the_sdk_scale(self, adapter: xarm_gripper.XArmGripperAdapter) -> None:
        limits = adapter.get_limits()
        assert limits.position_lower == [0.0]
        assert limits.position_upper == [850.0]

    def test_commands_pass_through_unconverted(
        self, adapter: xarm_gripper.XArmGripperAdapter
    ) -> None:
        """The 3.5 property, on the standalone path: emitted == received."""
        assert adapter.activate()
        for value in (0.0, 425.0, 850.0):
            assert adapter.write_joint_positions([value])
        assert adapter._arm.sent == [0.0, 425.0, 850.0]

    def test_reads_come_back_unconverted(
        self, adapter: xarm_gripper.XArmGripperAdapter
    ) -> None:
        adapter.activate()
        adapter.write_joint_positions([612.0])
        assert adapter.read_joint_positions() == [612.0]

    def test_write_enables_lazily(self, adapter: xarm_gripper.XArmGripperAdapter) -> None:
        """First write enables the gripper if activate() was never called."""
        assert adapter.write_joint_positions([100.0])
        assert adapter._arm.gripper_enabled is True

    def test_never_touches_the_arm(self, adapter: xarm_gripper.XArmGripperAdapter) -> None:
        """The fake raises on ANY arm call, so surviving a full cycle is proof."""
        adapter.activate()
        adapter.write_joint_positions([850.0])
        adapter.read_joint_positions()
        adapter.read_joint_velocities()
        adapter.read_joint_efforts()
        adapter.deactivate()
        adapter.disconnect()

    def test_refusals(self, adapter: xarm_gripper.XArmGripperAdapter) -> None:
        assert adapter.write_joint_velocities([1.0]) is False
        assert adapter.set_control_mode(ControlMode.VELOCITY) is False
        assert adapter.set_control_mode(ControlMode.SERVO_POSITION) is True

    def test_requires_an_address(self) -> None:
        with pytest.raises(ValueError, match="controller IP"):
            xarm_gripper.XArmGripperAdapter(address=None)

    def test_transient_read_failure_holds_the_last_value(
        self, adapter: xarm_gripper.XArmGripperAdapter
    ) -> None:
        adapter.activate()
        adapter.write_joint_positions([400.0])
        assert adapter.read_joint_positions() == [400.0]
        adapter._arm.get_gripper_position = lambda: (1, None)  # SDK hiccup
        assert adapter.read_joint_positions() == [400.0], "no glitch to an arbitrary value"


class TestWitnessBlueprint:
    def test_shape(self) -> None:
        from dimos.control.components import HardwareType
        from dimos.control.coordinator import ControlCoordinator
        from dimos.robot.grippers.blueprints.teleop import keyboard_teleop_gripper_xarm

        kwargs = next(
            a.kwargs
            for a in keyboard_teleop_gripper_xarm.blueprints
            if isinstance(a.module, type) and issubclass(a.module, ControlCoordinator)
        )
        hw = kwargs["hardware"][0]
        assert hw.hardware_type is HardwareType.GRIPPER
        assert hw.adapter_type == "xarm_gripper"
        assert hw.gripper_dof == len(hw.all_joints) == 1
        assert hw.arm_joints == []

        (task,) = kwargs["tasks"]
        assert task.type == "gripper"
        assert task.name == "hand_gripper"
        assert task.joint_names == ["hand/gripper"]
