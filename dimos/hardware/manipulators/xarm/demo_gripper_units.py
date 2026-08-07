# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bench check for GRIPPER-SPEC part 1.2 on a real xArm.

Proves the one claim part 1.2 makes: **the value a task emits is the value
that reaches the vendor SDK**. Before this part, commanding fully-open sent
722.5 to `set_gripper_position` instead of 850.0 — 15% of the travel lost to
a conversion applied twice.

Exercises the real chain (ConnectedHardware -> XArmAdapter -> xArm SDK) while
leaving the arm alone: arm joints are commanded to the pose they are already
measured at, so only the gripper moves.

    # 1. sanity-check the script itself, no hardware
    uv run python -m dimos.hardware.manipulators.xarm.demo_gripper_units --fake

    # 2. read-only against the real arm (default; commands nothing)
    uv run python -m dimos.hardware.manipulators.xarm.demo_gripper_units --ip 192.168.1.185

    # 3. actually drive the gripper open, then closed
    uv run python -m dimos.hardware.manipulators.xarm.demo_gripper_units --ip 192.168.1.185 --move
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Any

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.hardware_interface import ConnectedHardware
from dimos.hardware.manipulators.spec import ControlMode

_EXPECTED_MAX = 850.0
_LEGACY_BUG_VALUE = 722.5


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


def _build(ip: str, dof: int, fake: bool) -> tuple[ConnectedHardware, Any]:
    from dimos.hardware.manipulators.xarm import adapter as xarm_adapter

    if fake:
        xarm_adapter.XArmAPI = _FakeXArmSDK  # type: ignore[misc]

    adapter = xarm_adapter.XArmAdapter(address=ip, dof=dof, gripper_dof=1)
    if not adapter.connect():
        print(f"FAILED to connect to xArm at {ip}", file=sys.stderr)
        raise SystemExit(2)

    component = HardwareComponent(
        hardware_id="arm",
        hardware_type=HardwareType.MANIPULATOR,
        all_joints=[*make_joints("arm", dof), "arm/gripper"],
        gripper_dof=1,
        adapter_type="xarm",
    )
    return ConnectedHardware(adapter, component), adapter


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ip", default="192.168.1.185", help="xArm controller IP")
    ap.add_argument("--dof", type=int, default=7, choices=[6, 7], help="6 for xArm6, 7 for xArm7")
    ap.add_argument("--move", action="store_true", help="actually command the gripper")
    ap.add_argument("--fake", action="store_true", help="run against a stub SDK, no hardware")
    ap.add_argument(
        "--enable-arm",
        action="store_true",
        help="clear errors and enable the arm servos so the arm write also succeeds; "
        "without it the arm reports code=9 and stays put, which still proves the "
        "gripper claim",
    )
    ap.add_argument("--settle", type=float, default=2.0, help="seconds to wait after a command")
    args = ap.parse_args()

    hardware, adapter = _build(args.ip, args.dof, args.fake)
    print(f"\nconnected: xArm{args.dof} at {args.ip}{'  (FAKE SDK)' if args.fake else ''}")

    # --- 1. what does the adapter declare? -------------------------------
    limits = adapter.get_limits()
    lo, hi = limits.position_lower[-1], limits.position_upper[-1]
    total = adapter.get_dof() + adapter.get_gripper_dof()

    print("\n1. DECLARED RANGE  (R13 — the single authoritative declaration)")
    print(f"     get_dof()          = {adapter.get_dof()}   (arm only)")
    print(f"     get_gripper_dof()  = {adapter.get_gripper_dof()}")
    print(f"     get_limits() len   = {len(limits.position_lower)}   (expected {total})")
    print(f"     gripper range      = ({lo}, {hi})")
    ok_range = hi == _EXPECTED_MAX and len(limits.position_lower) == total
    print(f"     -> {'OK' if ok_range else 'MISMATCH — expected (0.0, 850.0)'}")

    # --- 2. read the gripper through the joint array ---------------------
    positions = adapter.read_joint_positions()
    print("\n2. READ  (R4 — one array, gripper trailing)")
    print(f"     read_joint_positions() len = {len(positions)}   (expected {total})")
    print(f"     gripper entry              = {positions[-1]:.1f}  (native, unconverted)")

    if not args.move:
        print("\n(read-only; pass --move to command the gripper)\n")
        return 0 if ok_range else 1

    # --- 3. the measurement 3.5 is about ---------------------------------
    if args.enable_arm:
        # Clears warn/error and enables the servos, WITHOUT moving to home.
        adapter._prepare_for_position_motion()
        print("\n   arm servos enabled (errors cleared, no home move)")
    else:
        print(
            "\n   arm servos NOT enabled: set_servo_angle_j will report code=9 and the"
            "\n   arm will not move. That is expected and does not affect the gripper"
            "\n   claim below. Pass --enable-arm to exercise the arm path too."
        )

    print("\n3. COMMAND  (the 3.5 claim)")
    results = []
    for label, target in (("FULLY OPEN", hi), ("FULLY CLOSED", lo)):
        hardware.write_command({"arm/gripper": target}, ControlMode.SERVO_POSITION)
        time.sleep(args.settle)
        measured = adapter.read_joint_positions()[-1]
        sent = adapter._arm.sent[-1] if args.fake else None
        line = f"     {label:<13} commanded {target:>7.1f}   measured {measured:>7.1f}"
        if sent is not None:
            line += f"   SDK got {sent:>7.1f}"
        print(line)
        results.append((label, target, measured))

    open_target, open_measured = results[0][1], results[0][2]
    drift = abs(open_measured - open_target) / open_target * 100 if open_target else 0.0
    bug_drift = abs(_LEGACY_BUG_VALUE - open_target) / open_target * 100

    print("\n     VERDICT")
    print(
        f"       commanded {open_target:.0f}, gripper reached {open_measured:.0f}"
        f"  ({drift:.1f}% short)"
    )
    print(f"       the bug would have reached ~{_LEGACY_BUG_VALUE:.0f}  ({bug_drift:.1f}% short)")
    # A surviving double conversion is multiplicative and large; a few units of
    # mechanical shortfall at the hard stop is not.
    fixed = drift < 5.0
    print(f"       -> {'FIXED — no second conversion' if fixed else 'STILL CONVERTING TWICE'}\n")

    adapter.disconnect()
    return 0 if (ok_range and fixed) else 1


if __name__ == "__main__":
    raise SystemExit(main())
