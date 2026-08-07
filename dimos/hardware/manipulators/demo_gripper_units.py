# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bench check for GRIPPER-SPEC part 1.2 — the gripper unit claim.

Proves the one thing part 1.2 asserts: **the value a task emits is the value
that reaches the vendor SDK**. Exercises the real chain
(ConnectedHardware -> adapter -> vendor SDK) while leaving the arm alone —
arm joints are commanded to the pose they are already measured at, so only
the gripper moves.

Each adapter had its endpoint declared in the wrong place, and each gains
travel back:

    xarm   0-850 (dimensionless SDK scale)  was capped at 722.5   +15.0%
    piper  0-0.08 m                         was capped at 0.07    +12.5%
    a1z    0-0.1 m                          already correct        0.0%

Usage::

    # sanity-check the script itself, no hardware (xarm only)
    uv run python -m dimos.hardware.manipulators.demo_gripper_units --fake --move

    # read-only against a real device (commands nothing)
    uv run python -m dimos.hardware.manipulators.demo_gripper_units xarm  --ip 192.168.1.210 --dof 6
    uv run python -m dimos.hardware.manipulators.demo_gripper_units piper --ip can0
    uv run python -m dimos.hardware.manipulators.demo_gripper_units a1z   --ip can0

    # drive the gripper open, then closed
    ... --move                 # gripper only; the arm reports code=9 and stays put
    ... --move --enable-arm    # also enables the arm servos (xarm)
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any, NamedTuple

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.hardware_interface import ConnectedHardware
from dimos.hardware.manipulators.spec import ControlMode
from dimos.msgs.std_msgs.Bool import Bool as _Bool


class _Device(NamedTuple):
    dof: int
    expected_max: float
    legacy_cap: float  # what the old, wrongly-placed endpoint capped it at
    unit: str


_DEVICES = {
    "xarm": _Device(6, 850.0, 722.5, "SDK scale"),
    "piper": _Device(6, 0.08, 0.07, "m"),
    "a1z": _Device(6, 0.1, 0.1, "m"),
}


class _FakeXArmSDK:
    """Stands in for the vendor SDK so the script can be checked without a robot."""

    def __init__(self, *_: object, **__: object) -> None:
        self.connected = True
        self.state = 0
        self.mode = 0
        self.error_code = 0
        self._gripper = 0.0
        self.sent: list[float] = []

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def set_gripper_enable(self, _on: bool) -> None: ...

    def set_mode(self, _mode: int) -> int:
        return 0

    def set_state(self, _state: int) -> int:
        return 0

    def get_servo_angle(self) -> tuple[int, list[float]]:
        return 0, [0.0] * 7

    def set_servo_angle_j(self, _angles: list[float], **__: object) -> int:
        return 0

    def get_joints_torque(self) -> tuple[int, list[float]]:
        return 0, [0.0] * 7

    def set_gripper_position(self, position: float, **__: object) -> int:
        self.sent.append(position)
        self._gripper = position
        return 0

    def get_gripper_position(self) -> tuple[int, float]:
        return 0, self._gripper


def _make_adapter(name: str, address: str, dof: int, fake: bool) -> Any:
    if name == "xarm":
        from dimos.hardware.manipulators.xarm import adapter as mod

        if fake:
            mod.XArmAPI = _FakeXArmSDK  # type: ignore[misc]
        return mod.XArmAdapter(address=address, dof=dof, gripper_dof=1)

    if name == "piper":
        from dimos.hardware.manipulators.piper.adapter import PiperAdapter

        return PiperAdapter(address=address, dof=6, gripper_dof=1)

    from dimos.hardware.manipulators.galaxea_a1z.adapter import GalaxeaA1ZAdapter
    from dimos.hardware.manipulators.galaxea_a1z.config import A1ZConfig, A1ZGripperConfig

    return GalaxeaA1ZAdapter(
        address=address, config=A1ZConfig(gripper=A1ZGripperConfig()), gripper_dof=1
    )


def _gripper_task(component: HardwareComponent, hardware: ConnectedHardware) -> Any:
    """Build the real task against this device, as a blueprint would."""
    from dimos.control.coordinator import TaskConfig
    from dimos.control.tasks.gripper_task.gripper_task import create_task

    return create_task(
        TaskConfig(
            name=f"{component.hardware_id}_gripper",
            type="gripper",
            joint_names=component.gripper_joints,
            priority=20,
        ),
        {component.hardware_id: hardware},
    )


def _snapshot(hardware: ConnectedHardware) -> Any:
    """One tick's measured state, as the tick loop builds it."""
    from dimos.control.task import CoordinatorState, JointStateSnapshot

    positions = {n: js.position for n, js in hardware.read_state().items()}
    return CoordinatorState(joints=JointStateSnapshot(joint_positions=positions), t_now=0.0)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("device", nargs="?", default="xarm", choices=sorted(_DEVICES))
    ap.add_argument("--ip", default="192.168.1.185", help="IP for xarm, CAN port for piper/a1z")
    ap.add_argument("--dof", type=int, default=6, choices=[6, 7])
    ap.add_argument("--move", action="store_true", help="actually command the gripper")
    ap.add_argument("--fake", action="store_true", help="stub SDK, no hardware (xarm only)")
    ap.add_argument(
        "--enable-arm",
        action="store_true",
        help="clear errors and enable the arm servos so the arm write also succeeds; "
        "without it the arm may report an error and stay put, which still proves "
        "the gripper claim",
    )
    ap.add_argument(
        "--via-task",
        action="store_true",
        help="drive through GripperControlTask (part 1.3) instead of commanding the "
        "wrapper directly: proves the task resolves its own range and converts once",
    )
    ap.add_argument("--settle", type=float, default=2.0, help="seconds to wait after a command")
    args = ap.parse_args()

    spec = _DEVICES[args.device]
    dof = args.dof if args.device == "xarm" else spec.dof

    adapter = _make_adapter(args.device, args.ip, dof, args.fake)
    if not adapter.connect():
        print(f"FAILED to connect to {args.device} at {args.ip}", file=sys.stderr)
        return 2

    component = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        all_joints=[*make_joints("arm", dof), "arm/gripper"],
        gripper_dof=1,
        adapter_type=args.device,
    )
    hardware = ConnectedHardware(adapter, component)
    print(f"\nconnected: {args.device} at {args.ip}{'  (FAKE SDK)' if args.fake else ''}")

    # --- 1. what does the adapter declare? -------------------------------
    limits = adapter.get_limits()
    lo, hi = limits.position_lower[-1], limits.position_upper[-1]
    total = adapter.get_dof() + adapter.get_gripper_dof()

    print("\n1. DECLARED RANGE  (R13 — the single authoritative declaration)")
    print(f"     get_dof()          = {adapter.get_dof()}   (arm only)")
    print(f"     get_gripper_dof()  = {adapter.get_gripper_dof()}")
    print(f"     get_limits() len   = {len(limits.position_lower)}   (expected {total})")
    print(f"     gripper range      = ({lo}, {hi}) {spec.unit}")
    ok_range = hi == spec.expected_max and len(limits.position_lower) == total
    print(f"     -> {'OK' if ok_range else f'MISMATCH — expected (0.0, {spec.expected_max})'}")

    # --- 2. read the gripper through the joint array ---------------------
    positions = adapter.read_joint_positions()
    print("\n2. READ  (R4 — one array, gripper trailing)")
    print(f"     read_joint_positions() len = {len(positions)}   (expected {total})")
    print(f"     gripper entry              = {positions[-1]:.4g} {spec.unit}  (unconverted)")

    if not args.move:
        print("\n(read-only; pass --move to command the gripper)\n")
        return 0 if ok_range else 1

    # --- 3. the measurement 3.5 is about ---------------------------------
    prepare = getattr(adapter, "_prepare_for_position_motion", None)
    if args.enable_arm and callable(prepare):
        prepare()  # clears warn/error and enables servos, WITHOUT a home move
        print("\n   arm servos enabled (errors cleared, no home move)")
    elif not args.enable_arm:
        print(
            "\n   arm servos NOT enabled: the arm write may report an error and the arm"
            "\n   will not move. Expected, and it does not affect the gripper claim."
        )

    task = _gripper_task(component, hardware) if args.via_task else None
    if task is not None:
        resolved = task.get_state()["limits"][0]
        print("\n   driving through GripperControlTask (part 1.3)")
        print(f"     task resolved its range from the adapter: {resolved}")
        if resolved != (lo, hi):
            print("     -> MISMATCH against get_limits(); R14a resolution is wrong")

    print("\n3. COMMAND  (the 3.5 claim)")
    results = []
    for label, target in (("FULLY OPEN", hi), ("FULLY CLOSED", lo)):
        if task is not None:
            # A wish, not a value — exactly what the keyboard sends. The task
            # converts using the range it read from this adapter.
            task.on_gripper_command(_Bool(data=(target == lo)), 0.0)
            out = task.compute(_snapshot(hardware))
            hardware.write_command(dict(zip(out.joint_names, out.positions, strict=True)), out.mode)
        else:
            hardware.write_command({"arm/gripper": target}, ControlMode.SERVO_POSITION)
        time.sleep(args.settle)
        measured = adapter.read_joint_positions()[-1]
        if task is not None:
            # Refresh the task's cache from a post-command snapshot; it only
            # sees state through compute(), exactly as the tick loop feeds it.
            task.compute(_snapshot(hardware))
        if task is not None:
            reported = task.get_position()
            line = (
                f"     {label:<13} commanded {target:>9.4g}   measured {measured:>9.4g}"
                f"   task.get_position {reported[0]:>9.4g}"
            )
            if abs(reported[0] - measured) > 1e-9:
                line += "  <- DISAGREES with coordinator_joint_state"
        else:
            line = f"     {label:<13} commanded {target:>9.4g}   measured {measured:>9.4g}"
        if args.fake:
            line += f"   SDK got {adapter._arm.sent[-1]:>9.4g}"
        print(line)
        results.append((target, measured))

    open_target, open_measured = results[0]
    drift = abs(open_measured - open_target) / open_target * 100 if open_target else 0.0
    legacy_drift = abs(spec.legacy_cap - open_target) / open_target * 100

    print("\n     VERDICT")
    print(
        f"       commanded {open_target:.4g}, gripper reached {open_measured:.4g}"
        f"  ({drift:.1f}% short)"
    )
    if legacy_drift > 0.0:
        print(
            f"       the old endpoint would have capped at {spec.legacy_cap:.4g}"
            f"  ({legacy_drift:.1f}% short)"
        )
    else:
        print("       this device's endpoint was already correct; expect no change")
    # A misplaced endpoint is multiplicative and large; a few units of
    # mechanical shortfall at the hard stop is not.
    fixed = drift < max(5.0, legacy_drift / 3.0)
    print(f"       -> {'FIXED — full travel available' if fixed else 'STILL CAPPED'}\n")

    adapter.disconnect()
    return 0 if (ok_range and fixed) else 1


if __name__ == "__main__":
    raise SystemExit(main())
