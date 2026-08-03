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

"""Deterministic tests for the container entrypoint gate."""

from __future__ import annotations

import sys
import types
from typing import Any

import pytest

from dimos.robot.galaxea.r1lite import config as cfg, entrypoint_gate
from dimos.robot.galaxea.r1lite.entrypoint_gate import (
    REQUIRED_TOPICS,
    command_capable_blueprint,
    missing_streams,
)


@pytest.mark.parametrize(
    "argv,expected",
    [
        (["run", "r1lite-coordinator"], "r1lite-coordinator"),
        (["run", "r1lite-keyboard-teleop"], "r1lite-keyboard-teleop"),
        (["run", "r1lite-quest-teleop"], "r1lite-quest-teleop"),
        # Global options before the subcommand must not dodge the gate.
        (["--viewer", "none", "run", "r1lite-coordinator"], "r1lite-coordinator"),
        (["--log-level", "debug", "run", "r1lite-quest-teleop"], "r1lite-quest-teleop"),
        # Inert invocations pass through.
        (["run", "r1lite-quest-teleop-sim"], None),
        (["list"], None),
        ([], None),
        # Substrings and lookalikes are not exact-element matches.
        (["run", "r1lite-coordinator-v2"], None),
    ],
)
def test_command_capable_detection(argv: list[str], expected: str | None) -> None:
    assert command_capable_blueprint(argv) == expected


def test_required_topics_are_the_arming_contract() -> None:
    assert set(REQUIRED_TOPICS) == set(cfg.ARMING_REQUIRED_FEEDBACK | cfg.PREFLIGHT_ONLY_FEEDBACK)
    assert cfg.FB_CHASSIS in REQUIRED_TOPICS
    assert cfg.FB_ARM_RIGHT in REQUIRED_TOPICS


def test_all_streams_present_passes() -> None:
    counts = dict.fromkeys(REQUIRED_TOPICS, 3)
    assert missing_streams(counts) == []


def test_single_silent_stream_fails() -> None:
    counts = dict.fromkeys(REQUIRED_TOPICS, 1)
    counts[cfg.FB_ARM_RIGHT] = 0
    assert missing_streams(counts) == [cfg.FB_ARM_RIGHT]


def test_absent_streams_fail() -> None:
    assert missing_streams({}) == list(REQUIRED_TOPICS)


# Poll-round behavior (pure, injected clock and counter)


class _Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def test_final_poll_window_bounded_by_deadline() -> None:
    clock = _Clock()
    windows: list[float] = []

    def count(topics: list[str], window: float) -> dict[str, int]:
        windows.append(window)
        clock.t += window
        return dict.fromkeys(topics, 0)

    missing = entrypoint_gate.poll_rounds(
        count, ["/a", "/b"], wait_window_s=12.0, poll_window_s=5.0, now=clock
    )
    assert missing == ["/a", "/b"]
    assert windows == [5.0, 5.0, 2.0]
    assert clock.t == pytest.approx(12.0)


def test_satisfied_topics_not_required_again() -> None:
    clock = _Clock()
    seen: list[list[str]] = []

    def count(topics: list[str], window: float) -> dict[str, int]:
        seen.append(list(topics))
        clock.t += window
        return {topics[0]: 1}

    missing = entrypoint_gate.poll_rounds(
        count, ["/a", "/b", "/c"], wait_window_s=100.0, poll_window_s=5.0, now=clock
    )
    assert missing == []
    assert seen == [["/a", "/b", "/c"], ["/b", "/c"], ["/c"]]


def test_all_topics_satisfied_first_round_polls_once() -> None:
    clock = _Clock()
    calls = {"n": 0}

    def count(topics: list[str], window: float) -> dict[str, int]:
        calls["n"] += 1
        return dict.fromkeys(topics, 2)

    assert (
        entrypoint_gate.poll_rounds(count, ["/a"], wait_window_s=10.0, poll_window_s=5.0, now=clock)
        == []
    )
    assert calls["n"] == 1


# main() production path with a fake rclpy


class _FakeSub:
    def __init__(self, topic: str, callback: Any) -> None:
        self.topic = topic
        self.callback = callback


class _FakeNode:
    def __init__(self, publishers: dict[str, str]) -> None:
        self.publishers = publishers
        self.subs: list[_FakeSub] = []
        self.destroyed_subs: list[_FakeSub] = []
        self.destroyed = False

    def get_publishers_info_by_topic(self, topic: str) -> list[Any]:
        if topic in self.publishers:
            return [types.SimpleNamespace(topic_type=self.publishers[topic])]
        return []

    def create_subscription(self, msg_type: Any, topic: str, callback: Any, qos: Any) -> _FakeSub:
        sub = _FakeSub(topic, callback)
        self.subs.append(sub)
        return sub

    def destroy_subscription(self, sub: _FakeSub) -> None:
        self.destroyed_subs.append(sub)

    def destroy_node(self) -> None:
        self.destroyed = True


def _fake_rclpy(monkeypatch: Any, node: _FakeNode, deliver: bool) -> dict[str, bool]:
    state = {"init": False, "shutdown": False}
    fake = types.ModuleType("rclpy")

    def _init() -> None:
        state["init"] = True

    def _shutdown() -> None:
        state["shutdown"] = True

    def _spin_once(n: Any, timeout_sec: float = 0.0) -> None:
        if deliver:
            for sub in node.subs:
                sub.callback(object())

    fake.init = _init  # type: ignore[attr-defined]
    fake.shutdown = _shutdown  # type: ignore[attr-defined]
    fake.create_node = lambda name: node  # type: ignore[attr-defined]
    fake.spin_once = _spin_once  # type: ignore[attr-defined]
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kw: object()  # type: ignore[attr-defined]
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)  # type: ignore[attr-defined]
    fake.qos = qos  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "rclpy", fake)
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos)
    return state


def test_main_fails_and_cleans_up_when_streams_absent(
    monkeypatch: Any, capsys: pytest.CaptureFixture[str]
) -> None:
    node = _FakeNode(publishers={})
    state = _fake_rclpy(monkeypatch, node, deliver=False)
    monkeypatch.setattr(entrypoint_gate, "WAIT_WINDOW_S", 0.2)
    monkeypatch.setattr(entrypoint_gate, "POLL_WINDOW_S", 0.1)
    rc = entrypoint_gate.main(["run", "r1lite-coordinator"])
    assert rc == 1
    err = capsys.readouterr().err
    for topic in entrypoint_gate.REQUIRED_TOPICS:
        assert topic in err
    assert state["init"] and state["shutdown"]
    assert node.destroyed


def test_main_passes_and_cleans_up_when_streams_deliver(monkeypatch: Any) -> None:
    fake_msgs = types.ModuleType("fake_msgs")
    fake_msgs_msg = types.ModuleType("fake_msgs.msg")
    fake_msgs_msg.Thing = object  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "fake_msgs", fake_msgs)
    monkeypatch.setitem(sys.modules, "fake_msgs.msg", fake_msgs_msg)
    node = _FakeNode(
        publishers=dict.fromkeys(entrypoint_gate.REQUIRED_TOPICS, "fake_msgs/msg/Thing")
    )
    state = _fake_rclpy(monkeypatch, node, deliver=True)
    monkeypatch.setattr(entrypoint_gate, "WAIT_WINDOW_S", 5.0)
    rc = entrypoint_gate.main(["run", "r1lite-quest-teleop"])
    assert rc == 0
    assert state["init"] and state["shutdown"]
    assert node.destroyed
    assert len(node.destroyed_subs) == len(entrypoint_gate.REQUIRED_TOPICS)


def test_main_inert_invocation_never_touches_rclpy(monkeypatch: Any) -> None:
    monkeypatch.setitem(sys.modules, "rclpy", None)
    assert entrypoint_gate.main(["run", "r1lite-quest-teleop-sim"]) == 0


class _ConcurrencyNode(_FakeNode):
    """Tracks the maximum number of simultaneously live subscriptions."""

    def __init__(self, publishers: dict[str, str]) -> None:
        super().__init__(publishers)
        self.max_live = 0

    def create_subscription(self, msg_type: Any, topic: str, callback: Any, qos: Any) -> _FakeSub:
        sub = super().create_subscription(msg_type, topic, callback, qos)
        live = len(self.subs) - len(self.destroyed_subs)
        self.max_live = max(self.max_live, live)
        return sub


def test_measurement_never_subscribes_concurrently(monkeypatch: Any) -> None:
    # The vendor stack starves concurrent fresh readers (2026-07-28 field
    # finding): the gate must hold at most ONE live subscription at any
    # moment, like the reworked preflight.
    fake_msgs = types.ModuleType("fake_msgs")
    fake_msgs_msg = types.ModuleType("fake_msgs.msg")
    fake_msgs_msg.Thing = object  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "fake_msgs", fake_msgs)
    monkeypatch.setitem(sys.modules, "fake_msgs.msg", fake_msgs_msg)
    node = _ConcurrencyNode(
        publishers=dict.fromkeys(entrypoint_gate.REQUIRED_TOPICS, "fake_msgs/msg/Thing")
    )
    _fake_rclpy(monkeypatch, node, deliver=True)
    counts = entrypoint_gate._count_messages(node, list(entrypoint_gate.REQUIRED_TOPICS), 0.5)
    assert all(counts[t] >= 1 for t in entrypoint_gate.REQUIRED_TOPICS)
    assert node.max_live == 1
    assert len(node.destroyed_subs) == len(entrypoint_gate.REQUIRED_TOPICS)


def test_failure_distinguishes_graph_absence_from_silence(
    monkeypatch: Any, capsys: pytest.CaptureFixture[str]
) -> None:
    fake_msgs = types.ModuleType("fake_msgs")
    fake_msgs_msg = types.ModuleType("fake_msgs.msg")
    fake_msgs_msg.Thing = object  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "fake_msgs", fake_msgs)
    monkeypatch.setitem(sys.modules, "fake_msgs.msg", fake_msgs_msg)
    # One topic has a visible publisher that never delivers; the rest have
    # no publisher info at all.
    import dimos.robot.galaxea.r1lite.config as cfg_mod

    node = _FakeNode(publishers={cfg_mod.FB_ARM_LEFT: "fake_msgs/msg/Thing"})
    _fake_rclpy(monkeypatch, node, deliver=False)
    monkeypatch.setattr(entrypoint_gate, "WAIT_WINDOW_S", 0.2)
    monkeypatch.setattr(entrypoint_gate, "POLL_WINDOW_S", 0.1)
    rc = entrypoint_gate.main(["run", "r1lite-coordinator"])
    assert rc == 1
    err = capsys.readouterr().err
    assert f"FAIL no message on {cfg_mod.FB_ARM_LEFT} — publisher seen but no data" in err
    assert "no publisher info in the graph" in err
