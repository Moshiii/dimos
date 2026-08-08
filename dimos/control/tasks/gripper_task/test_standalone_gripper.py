# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A gripper that is its own device, end to end (GRIPPER-SPEC 8, R28's degeneracy).

Pins in CI what 7.2 demonstrated in a transcript: a GRIPPER component is the
same arithmetic as an arm's — ``gripper_dof == len(all_joints)`` puts R28's
split at zero, ``arm_joints`` is empty, and everything above the adapter works
verbatim. No new task, no new wrapper, no special case.
"""

from __future__ import annotations

import pytest

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_gripper_joints,
)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.control.hardware_interface import ConnectedHardware
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.gripper_task.gripper_task import create_task
from dimos.hardware.grippers.registry import gripper_adapter_registry
from dimos.msgs.std_msgs.Bool import Bool

# The reference blueprint's grasp posture, H100-like 0-100 scale.
_GRASP = [80.0, 20.0, 50.0, 80.0, 20.0, 50.0]


def _component() -> HardwareComponent:
    return HardwareComponent(
        hardware_id="hand",
        hardware_type=HardwareType.GRIPPER,
        all_joints=make_gripper_joints("hand", 6),
        gripper_dof=6,
        adapter_type="mock",
        adapter_kwargs={"limits": (0.0, 100.0)},
    )


@pytest.fixture
def wired():
    """Component -> registry-built adapter -> wrapper -> task, as the coordinator wires it."""
    component = _component()
    adapter = gripper_adapter_registry.create(
        component.adapter_type,
        dof=component.gripper_dof,
        address=component.address,
        hardware_id=component.hardware_id,
        **component.adapter_kwargs,
    )
    assert adapter.connect()
    hardware = ConnectedHardware(adapter, component)
    task = create_task(
        TaskConfig(
            name="hand_gripper",
            type="gripper",
            joint_names=component.gripper_joints,
            priority=20,
            params={"reference_pose": _GRASP},
        ),
        {"hand": hardware},
    )
    return task, hardware, adapter


def _tick(task, hardware) -> list[float]:
    positions = {n: s.position for n, s in hardware.read_state().items()}
    out = task.compute(
        CoordinatorState(joints=JointStateSnapshot(joint_positions=positions), t_now=0.0)
    )
    if out is not None:
        hardware.write_command(dict(zip(out.joint_names, out.positions, strict=True)), out.mode)
    return hardware.adapter.read_joint_positions()


class TestTheDegenerateSplit:
    def test_all_joints_are_gripper_joints(self) -> None:
        component = _component()
        assert component.arm_joints == []
        assert component.gripper_joints == component.all_joints
        assert len(component.gripper_joints) == 6

    def test_the_invariant_rejects_a_partial_gripper(self) -> None:
        """A GRIPPER component is all gripper — checked, not conventional (8.4)."""
        with pytest.raises(ValueError, match="GRIPPER component must have"):
            HardwareComponent(
                hardware_id="hand",
                hardware_type=HardwareType.GRIPPER,
                all_joints=make_gripper_joints("hand", 6),
                gripper_dof=5,
            )


class TestEndToEnd:
    def test_task_resolves_the_device_range_unprompted(self, wired) -> None:
        """R14a on a GRIPPER component: trailing slice at split zero = everything."""
        task, _, _ = wired
        assert task.get_state()["limits"] == [(0.0, 100.0)] * 6

    def test_fine_grained_per_joint_control(self, wired) -> None:
        task, hardware, _ = wired
        task.set_position([10.0, 20.0, 30.0, 40.0, 50.0, 60.0], t_now=0.0)
        assert _tick(task, hardware) == [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]

    def test_the_same_bool_the_keyboard_sends_drives_a_six_joint_hand(self, wired) -> None:
        """Close sweeps to the vendor grasp pose, not to a six-joint fist (R19a)."""
        task, hardware, _ = wired

        task.on_gripper_command(Bool(data=True), 0.0)
        assert _tick(task, hardware) == pytest.approx(_GRASP)

        task.on_gripper_command(Bool(data=False), 0.0)
        assert _tick(task, hardware) == pytest.approx([100.0] * 6)

    def test_normalized_vector_lands_on_the_device_scale(self, wired) -> None:
        task, hardware, _ = wired
        task.set_normalized([0.0, 0.25, 0.5, 0.75, 1.0, 0.5], t_now=0.0)
        assert _tick(task, hardware) == pytest.approx([0.0, 25.0, 50.0, 75.0, 100.0, 50.0])

    def test_get_position_agrees_with_the_wrapped_state(self, wired) -> None:
        task, hardware, adapter = wired
        adapter.write_joint_positions([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        _tick(task, hardware)
        assert task.get_position() == pytest.approx([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])


class TestReferenceBlueprint:
    def test_shape(self) -> None:
        """The blueprint others copy for the H100 carries the whole pattern."""
        from dimos.robot.grippers.blueprints.basic import coordinator_gripper_mock

        kwargs = next(
            a.kwargs
            for a in coordinator_gripper_mock.blueprints
            if isinstance(a.module, type) and issubclass(a.module, ControlCoordinator)
        )
        hw = kwargs["hardware"][0]
        assert hw.hardware_type is HardwareType.GRIPPER
        assert hw.gripper_dof == len(hw.all_joints) == 6

        (task,) = kwargs["tasks"]
        assert task.type == "gripper"
        assert task.name == "hand_gripper"
        assert task.joint_names == hw.gripper_joints
        assert len(task.params["reference_pose"]) == 6, "R19a: multi-joint declares its grasp"
