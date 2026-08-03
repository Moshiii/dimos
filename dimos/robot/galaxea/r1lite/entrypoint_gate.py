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

"""Vendor-stack gate for the R1 Lite container entrypoint.

Run as `python3 -m dimos.robot.galaxea.r1lite.entrypoint_gate <dimos args>`
before the dimos CLI. For an invocation that can command hardware, every
feedback stream in the arming contract must deliver at least one message
within the wait window or the gate exits nonzero, and the container
refuses to launch. Invocations without a command-capable blueprint pass
through untouched.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
import sys
import time

from dimos.robot.galaxea.r1lite import config as cfg

COMMAND_CAPABLE = frozenset({"r1lite-coordinator", "r1lite-keyboard-teleop", "r1lite-quest-teleop"})

# Every feedback source the connection consumes plus the preflight-only
# chassis wheel states: the same set the arming contract requires.
REQUIRED_TOPICS = tuple(sorted(cfg.ARMING_REQUIRED_FEEDBACK | cfg.PREFLIGHT_ONLY_FEEDBACK))

WAIT_WINDOW_S = 120.0
POLL_WINDOW_S = 5.0


def command_capable_blueprint(argv: Sequence[str]) -> str | None:
    """The blueprint name if this invocation can command hardware.

    Matches any exact argv element, so global options placed before the
    run subcommand cannot dodge the gate; distinct names such as the sim
    variant do not match. Over-matching a non-run invocation that merely
    names a blueprint is accepted: the gate then requires a healthy
    vendor stack for a command that did not need one, which fails closed
    rather than open.
    """
    for arg in argv:
        if arg in COMMAND_CAPABLE:
            return arg
    return None


def missing_streams(counts: dict[str, int]) -> list[str]:
    """Required topics that delivered no message."""
    return [t for t in REQUIRED_TOPICS if counts.get(t, 0) < 1]


def _import_msg_type(type_name: str) -> object:
    import importlib

    package, _, name = type_name.partition("/msg/")
    module = importlib.import_module(f"{package}.msg")
    return getattr(module, name)


def _count_messages(
    node: object,
    topics: Sequence[str],
    window_s: float,
    graph_seen: set[str] | None = None,
) -> dict[str, int]:
    """Measure topics strictly one at a time, never concurrently.

    The vendor stack starves concurrent fresh readers (field finding,
    2026-07-28 — the same behavior that moved preflight to sequential
    measurement), so each topic gets a lone subscription for its slice
    of the window and the subscription is destroyed before the next
    topic is tried. `graph_seen` collects topics whose publisher info
    ever appeared, so a failure can distinguish "graph never showed a
    publisher" from "publisher seen but no data".
    """
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    counts = dict.fromkeys(topics, 0)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    slice_s = window_s / max(1, len(topics))
    for topic in topics:
        slice_deadline = time.monotonic() + slice_s
        infos = node.get_publishers_info_by_topic(topic)  # type: ignore[attr-defined]
        if not infos:
            # Spin out the slice anyway: discovery needs a live executor
            # and the next round retries this topic.
            while time.monotonic() < slice_deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            continue
        if graph_seen is not None:
            graph_seen.add(topic)
        msg_type = _import_msg_type(infos[0].topic_type)

        def _count(_msg: object, name: str = topic) -> None:
            counts[name] += 1

        sub = node.create_subscription(msg_type, topic, _count, qos)  # type: ignore[attr-defined]
        while time.monotonic() < slice_deadline and counts[topic] < 1:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_subscription(sub)  # type: ignore[attr-defined]
    return counts


def poll_rounds(
    count: Callable[[list[str], float], dict[str, int]],
    topics: Sequence[str],
    wait_window_s: float,
    poll_window_s: float,
    now: Callable[[], float] = time.monotonic,
) -> list[str]:
    """Poll until every topic delivered a message or the deadline passes.

    A topic satisfied in an earlier round is not required again, and the
    final poll window is bounded by the remaining deadline so the total
    wait cannot overshoot it by a window.
    """
    deadline = now() + wait_window_s
    missing = list(topics)
    while missing:
        remaining = deadline - now()
        if remaining <= 0:
            break
        counts = count(missing, min(poll_window_s, remaining))
        missing = [t for t in missing if counts.get(t, 0) < 1]
    return missing


def main(argv: Sequence[str]) -> int:
    blueprint = command_capable_blueprint(argv)
    if blueprint is None:
        return 0
    print(
        f"[entrypoint-gate] {blueprint} is command-capable: requiring one message "
        f"on each of {len(REQUIRED_TOPICS)} vendor feedback topics"
    )
    import rclpy

    rclpy.init()
    node = rclpy.create_node("dimos_r1lite_entrypoint_gate")
    graph_seen: set[str] = set()
    started = time.monotonic()

    def _measure(topics: Sequence[str], window: float) -> dict[str, int]:
        counts = _count_messages(node, topics, window, graph_seen)
        elapsed = time.monotonic() - started
        for topic in topics:
            if counts.get(topic, 0) > 0:
                print(f"[entrypoint-gate] OK {topic} ({elapsed:.0f}s)")
        silent = [t for t in topics if counts.get(t, 0) < 1]
        if silent:
            print(
                f"[entrypoint-gate] {elapsed:.0f}s: {len(silent)} still silent: "
                + ", ".join(silent)
            )
        return counts

    try:
        missing = poll_rounds(_measure, REQUIRED_TOPICS, WAIT_WINDOW_S, POLL_WINDOW_S)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if missing:
        for topic in missing:
            reason = (
                "publisher seen but no data (reader starvation?)"
                if topic in graph_seen
                else "no publisher info in the graph (discovery)"
            )
            print(f"[entrypoint-gate] FAIL no message on {topic} — {reason}", file=sys.stderr)
        return 1
    print("[entrypoint-gate] vendor feedback healthy")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
