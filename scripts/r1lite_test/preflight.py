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
connection status stream, asks the operator to confirm RC mode 5, and
publishes the arming message.

Both subcommands exit nonzero at the first failed check.

    python scripts/r1lite_test/preflight.py phase1
    python scripts/r1lite_test/preflight.py arm
"""

from __future__ import annotations

import argparse
import sys
import time

from dimos.robot.galaxea.r1lite import config as cfg

RATE_MEASURE_WINDOW_S = 2.0


def _fail(message: str) -> None:
    print(f"FAIL {message}")
    sys.exit(1)


def _measure_rates(node: object, topics: list[str]) -> dict[str, float]:
    """Count messages per topic for a fixed window using raw subscriptions."""
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    counts = dict.fromkeys(topics, 0)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    subs = []
    for topic in topics:
        infos = node.get_publishers_info_by_topic(topic)  # type: ignore[attr-defined]
        if not infos:
            counts[topic] = 0
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


def _import_msg_type(type_name: str) -> object:
    import importlib

    package, _, name = type_name.partition("/msg/")
    module = importlib.import_module(f"{package}.msg")
    return getattr(module, name)


def _check_rates(node: object) -> None:
    topics = list(cfg.FEEDBACK_NOMINAL_HZ)
    rates = _measure_rates(node, topics)
    for topic, nominal in cfg.FEEDBACK_NOMINAL_HZ.items():
        measured = rates.get(topic, 0.0)
        if nominal is None:
            print(f"INFO {topic} measured {measured:.0f} Hz (no pinned nominal yet)")
            continue
        if measured < nominal / 2.0:
            _fail(f"{topic} at {measured:.0f} Hz, below half of nominal {nominal:.0f} Hz")
        print(f"OK   {topic} {measured:.0f} Hz (nominal {nominal:.0f})")


def _publishers(node: object, topic: str) -> list[str]:
    infos = node.get_publishers_info_by_topic(topic)  # type: ignore[attr-defined]
    return [f"{info.node_namespace.rstrip('/')}/{info.node_name}".lstrip("/") for info in infos]


def _check_matrix(node: object, phase: str) -> None:
    column = 0 if phase == "phase1" else 1
    for topic, counts in cfg.ARMING_MATRIX.items():
        allowed = counts[column]
        names = _publishers(node, topic)
        if len(names) > allowed:
            _fail(f"{topic} has publishers {names}; allowed {allowed} in {phase}")
        if allowed == 1 and len(names) == 1 and cfg.ROS_NODE_NAME not in names[0]:
            _fail(f"{topic} publisher is {names[0]}, not {cfg.ROS_NODE_NAME}")
        if allowed == 1 and len(names) == 0:
            _fail(f"{topic} has no publisher; dimos connection is not running")
        print(f"OK   {topic} publishers={names or 'none'}")


def _read_nonce() -> str:
    from dimos.core.transport import LCMTransport
    from dimos.msgs.std_msgs.String import String

    holder: dict[str, str] = {}
    transport = LCMTransport("/r1lite/connection_status", String)

    def _on_status(msg: String) -> None:
        for part in msg.data.split():
            key, _, value = part.partition("=")
            if key == "nonce" and value:
                holder["nonce"] = value
            if key == "state":
                holder["state"] = value

    unsub = transport.subscribe(_on_status)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and "nonce" not in holder:
        time.sleep(0.1)
    unsub()
    transport.stop()
    if holder.get("state") not in (None, "READY_DISARMED"):
        _fail(f"connection state is {holder.get('state')}, not READY_DISARMED")
    if "nonce" not in holder:
        _fail("no connection status received; is the dimos blueprint running?")
    return holder["nonce"]


def _send_arm(nonce: str) -> None:
    from dimos.core.transport import LCMTransport
    from dimos.msgs.std_msgs.String import String

    transport = LCMTransport(cfg.ARMING_TOPIC, String)
    transport.publish(String(data=f"ARM RC5 {nonce}"))
    time.sleep(0.2)
    transport.stop()
    print("OK   arming message sent")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=["phase1", "arm"])
    args = parser.parse_args()

    import rclpy

    rclpy.init()
    node = rclpy.create_node("dimos_r1lite_preflight")
    try:
        _check_rates(node)
        _check_matrix(node, args.phase)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if args.phase == "phase1":
        print("PASS phase1: zero actuator publishers, feedback healthy")
        return

    nonce = _read_nonce()
    print(
        "Arming attestation: confirm the RC is ON with all switches in "
        "position 1 (mode 5) and you hold the e-stop."
    )
    answer = input(f"Type exactly 'ARM RC5 {nonce}' to arm: ").strip()
    if answer != f"ARM RC5 {nonce}":
        _fail("attestation text did not match; not arming")
    _send_arm(nonce)
    print("PASS arm: check the blueprint log for 'ARMED by operator'")


if __name__ == "__main__":
    main()
