#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Two-phase ownership preflight and operator arming for the R1 Lite.

phase1: run BEFORE dimos starts. Verifies vendor feedback rates and that
ZERO publishers exist on every actuator command topic.

arm: run while dimos runs disarmed. Verifies rates, verifies the
publisher-count matrix (exactly one dimos publisher on dimos-owned
topics, zero on torso topics), reads the arming nonce from the
connection status stream, asks the operator to type the arming
attestation, publishes it, and then waits for the connection to report
ARMED. Success is claimed only from an observed state transition.

Both subcommands exit nonzero at the first failed check.

    python scripts/r1lite_test/preflight.py phase1
    python scripts/r1lite_test/preflight.py arm
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
import sys
import time
from typing import Any

from dimos.robot.galaxea.r1lite import config as cfg

RATE_MEASURE_WINDOW_S = 2.0
ARM_OBSERVE_TIMEOUT_S = 5.0


def check_rates(measured: dict[str, float], phase: str) -> list[str]:
    """Errors for the feedback-rate contract, empty when the phase passes.

    Topics with a pinned nominal must measure at least half of it in both
    phases. A topic with no pinned nominal must be present with a positive
    measured rate in phase1, and always fails the arm phase: arming stays
    closed until a hardware session pins the nominal in config.
    """
    errors: list[str] = []
    for topic, nominal in cfg.FEEDBACK_NOMINAL_HZ.items():
        rate = measured.get(topic, 0.0)
        if nominal is None:
            if phase == "arm":
                errors.append(
                    f"{topic} has no pinned nominal rate; measure it and pin it "
                    f"in r1lite config before arming (measured {rate:.0f} Hz)"
                )
            elif rate <= 0.0:
                errors.append(f"{topic} absent or silent (measured 0 Hz)")
            continue
        if rate < nominal / 2.0:
            errors.append(f"{topic} at {rate:.0f} Hz, below half of nominal {nominal:.0f} Hz")
    return errors


def check_matrix(publishers: dict[str, list[str]], phase: str) -> list[str]:
    """Errors for the publisher-count matrix, empty when the phase passes.

    publishers maps topic to the exact node names currently publishing.
    Identity is exact equality with the connection node name.
    """
    column = 0 if phase == "phase1" else 1
    errors: list[str] = []
    for topic, counts in cfg.ARMING_MATRIX.items():
        allowed = counts[column]
        names = publishers.get(topic, [])
        if len(names) > allowed:
            errors.append(f"{topic} has publishers {names}; allowed {allowed} in {phase}")
        elif allowed == 1:
            if not names:
                errors.append(f"{topic} has no publisher; dimos connection is not running")
            elif names[0] != cfg.ROS_NODE_NAME:
                errors.append(f"{topic} publisher is {names[0]!r}, not {cfg.ROS_NODE_NAME!r}")
    return errors


def wait_for_status(
    read_status: Callable[[], dict[str, str]],
    predicate: Callable[[dict[str, str]], bool],
    deadline: float,
    now: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, str]:
    """Poll read_status until predicate passes or the deadline expires.

    Returns the last observed status; the caller decides pass or fail from
    the predicate on that value.
    """
    status = read_status()
    while not predicate(status) and now() < deadline:
        sleep(0.1)
        status = read_status()
    return status


def _fail(message: str) -> None:
    print(f"FAIL {message}")
    sys.exit(1)


def _import_msg_type(type_name: str) -> object:
    import importlib

    package, _, name = type_name.partition("/msg/")
    module = importlib.import_module(f"{package}.msg")
    return getattr(module, name)


def _measure_rates(node: object, topics: list[str]) -> dict[str, float]:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    counts = dict.fromkeys(topics, 0)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subs = []
    for topic in topics:
        infos = node.get_publishers_info_by_topic(topic)  # type: ignore[attr-defined]
        if not infos:
            continue
        msg_type = _import_msg_type(infos[0].topic_type)

        def _count(_msg: object, name: str = topic) -> None:
            counts[name] += 1

        subs.append(node.create_subscription(msg_type, topic, _count, qos))  # type: ignore[attr-defined]
    deadline = time.monotonic() + RATE_MEASURE_WINDOW_S
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    for sub in subs:
        node.destroy_subscription(sub)  # type: ignore[attr-defined]
    return {t: c / RATE_MEASURE_WINDOW_S for t, c in counts.items()}


def _publisher_node_names(node: object, topic: str) -> list[str]:
    infos = node.get_publishers_info_by_topic(topic)  # type: ignore[attr-defined]
    return [str(info.node_name) for info in infos]


def _read_status_lcm() -> dict[str, str]:
    from dimos.core.transport import LCMTransport
    from dimos.msgs.std_msgs.String import String

    holder: dict[str, str] = {}
    transport: Any = LCMTransport("/r1lite/connection_status", String)

    def _on_status(msg: String) -> None:
        for part in msg.data.split():
            key, _, value = part.partition("=")
            if key:
                holder[key] = value

    unsub = transport.subscribe(_on_status)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and "state" not in holder:
        time.sleep(0.05)
    unsub()
    transport.stop()
    return holder


def _send_arm(nonce: str) -> None:
    from dimos.core.transport import LCMTransport
    from dimos.msgs.std_msgs.String import String

    transport: Any = LCMTransport(cfg.ARMING_TOPIC, String)
    transport.publish(String(data=f"ARM RC5 {nonce}"))
    time.sleep(0.2)
    transport.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=["phase1", "arm"])
    args = parser.parse_args()

    import rclpy

    rclpy.init()
    node = rclpy.create_node("dimos_r1lite_preflight")
    try:
        rates = _measure_rates(node, list(cfg.FEEDBACK_NOMINAL_HZ))
        for error in check_rates(rates, args.phase):
            _fail(error)
        for topic, nominal in cfg.FEEDBACK_NOMINAL_HZ.items():
            print(f"OK   {topic} {rates.get(topic, 0.0):.0f} Hz (nominal {nominal})")
        publishers = {t: _publisher_node_names(node, t) for t in cfg.ARMING_MATRIX}
        for error in check_matrix(publishers, args.phase):
            _fail(error)
        for topic, names in publishers.items():
            print(f"OK   {topic} publishers={names or 'none'}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.phase == "phase1":
        print("PASS phase1: zero actuator publishers, feedback healthy")
        return

    status = _read_status_lcm()
    if status.get("state") != "READY_DISARMED" or not status.get("nonce"):
        _fail(f"connection status {status or 'absent'}; need READY_DISARMED with a nonce")
    nonce = status["nonce"]
    print(
        "Arming attestation: confirm the RC is ON with all switches in "
        "position 1 (mode 5) and you hold the e-stop."
    )
    answer = input(f"Type exactly 'ARM RC5 {nonce}' to arm: ").strip()
    if answer != f"ARM RC5 {nonce}":
        _fail("attestation text did not match; not arming")
    _send_arm(nonce)

    observed = wait_for_status(
        _read_status_lcm,
        lambda s: s.get("state") == "ARMED",
        deadline=time.monotonic() + ARM_OBSERVE_TIMEOUT_S,
    )
    if observed.get("state") != "ARMED":
        _fail(f"connection did not report ARMED (last status {observed or 'absent'})")
    print("PASS arm: connection reports ARMED")


if __name__ == "__main__":
    main()
