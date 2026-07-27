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

"""Deterministic tests for the preflight decision logic.

The script lives outside the package; its pure check functions are
imported through a path insertion, the same pattern the quest replay
tooling uses.
"""

from __future__ import annotations

from pathlib import Path
import sys

from dimos.robot.galaxea.r1lite import config as cfg

_SCRIPTS = str(Path(__file__).resolve().parents[4] / "scripts" / "r1lite_test")
sys.path.insert(0, _SCRIPTS)
try:
    from preflight import check_matrix, check_rates, wait_for_status
finally:
    sys.path.remove(_SCRIPTS)


def _healthy_rates() -> dict[str, float]:
    return {
        topic: (nominal if nominal is not None else 50.0)
        for topic, nominal in cfg.FEEDBACK_NOMINAL_HZ.items()
    }


def _phase_publishers(phase: str) -> dict[str, list[str]]:
    if phase == "phase1":
        return {topic: [] for topic in cfg.ARMING_MATRIX}
    return {
        topic: ([cfg.ROS_NODE_NAME] if allowed == 1 else [])
        for topic, (_, allowed) in cfg.ARMING_MATRIX.items()
    }


def test_healthy_phase1_and_arm_pass_rates() -> None:
    assert check_rates(_healthy_rates(), "phase1") == []


def test_low_rate_fails_both_phases() -> None:
    rates = _healthy_rates()
    rates[cfg.FB_ARM_LEFT] = 40.0
    assert check_rates(rates, "phase1")
    assert check_rates(rates, "arm")


def test_unpinned_topic_must_be_present_in_phase1() -> None:
    rates = _healthy_rates()
    rates[cfg.FB_CHASSIS_SPEED] = 0.0
    errors = check_rates(rates, "phase1")
    assert any(cfg.FB_CHASSIS_SPEED in e for e in errors)


def test_unpinned_topic_always_fails_arm_phase() -> None:
    errors = check_rates(_healthy_rates(), "arm")
    assert any(cfg.FB_CHASSIS_SPEED in e and "pin" in e for e in errors)


def test_phase1_requires_zero_publishers_everywhere() -> None:
    publishers = _phase_publishers("phase1")
    assert check_matrix(publishers, "phase1") == []
    publishers[cfg.CMD_ARM_LEFT] = ["gello_node"]
    errors = check_matrix(publishers, "phase1")
    assert any("gello_node" in e for e in errors)


def test_arm_phase_requires_exactly_dimos() -> None:
    publishers = _phase_publishers("arm")
    assert check_matrix(publishers, "arm") == []


def test_arm_phase_rejects_missing_dimos_publisher() -> None:
    publishers = _phase_publishers("arm")
    publishers[cfg.CMD_CHASSIS_SPEED] = []
    errors = check_matrix(publishers, "arm")
    assert any("not running" in e for e in errors)


def test_arm_phase_rejects_substring_impostor() -> None:
    publishers = _phase_publishers("arm")
    publishers[cfg.CMD_ARM_RIGHT] = [f"not_{cfg.ROS_NODE_NAME}"]
    errors = check_matrix(publishers, "arm")
    assert any("not_" in e for e in errors)


def test_arm_phase_rejects_second_publisher() -> None:
    publishers = _phase_publishers("arm")
    publishers[cfg.CMD_ARM_LEFT] = [cfg.ROS_NODE_NAME, "relaxed_ik_left"]
    errors = check_matrix(publishers, "arm")
    assert any("relaxed_ik_left" in str(e) for e in errors)


def test_torso_topics_zero_in_both_phases() -> None:
    for phase in ("phase1", "arm"):
        publishers = _phase_publishers(phase)
        publishers[cfg.CMD_TORSO_JOINT] = [cfg.ROS_NODE_NAME]
        errors = check_matrix(publishers, phase)
        assert any(cfg.CMD_TORSO_JOINT in e for e in errors)


def test_wait_for_status_success_rejection_timeout() -> None:
    sequences = iter(
        [
            {"state": "READY_DISARMED"},
            {"state": "READY_DISARMED"},
            {"state": "ARMED"},
        ]
    )
    clock = {"t": 0.0}

    def now() -> float:
        return clock["t"]

    def sleep(dt: float) -> None:
        clock["t"] += dt

    result = wait_for_status(
        lambda: next(sequences),
        lambda s: s.get("state") == "ARMED",
        deadline=10.0,
        now=now,
        sleep=sleep,
    )
    assert result["state"] == "ARMED"

    rejected = wait_for_status(
        lambda: {"state": "READY_DISARMED"},
        lambda s: s.get("state") == "ARMED",
        deadline=1.0,
        now=now,
        sleep=sleep,
    )
    assert rejected["state"] == "READY_DISARMED"
