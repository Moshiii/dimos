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

"""Deterministic tests for the preflight decision logic and phase flow.

The script lives outside the package; it is loaded by file path so these
tests sit inside the collected test tree (pytest collects only under
dimos/), keeping the safety flow in CI.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from dimos.robot.galaxea.r1lite import config as cfg

_PREFLIGHT_PATH = Path(__file__).resolve().parents[4] / "scripts" / "r1lite_test" / "preflight.py"
_spec = importlib.util.spec_from_file_location("r1lite_preflight", _PREFLIGHT_PATH)
assert _spec is not None and _spec.loader is not None
preflight = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(preflight)

check_matrix = preflight.check_matrix
check_rates = preflight.check_rates
wait_for_status = preflight.wait_for_status


def _never_ask(prompt: str) -> str:
    pytest.fail("this phase must not prompt for attestation")


def _never_arm(nonce: str) -> None:
    pytest.fail("this phase must not reach the arming publisher")


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


def _disarmed_status() -> dict[str, str]:
    return {"state": "READY_DISARMED", "nonce": "abc123"}


# check_rates


def test_healthy_phase1_rates_pass() -> None:
    assert check_rates(_healthy_rates(), "phase1") == []


def test_low_rate_fails_every_phase() -> None:
    rates = _healthy_rates()
    rates[cfg.FB_ARM_LEFT] = 40.0
    assert check_rates(rates, "phase1")
    assert check_rates(rates, "phase2")
    assert check_rates(rates, "arm")


def test_unpinned_topic_must_be_present_in_phase1_and_phase2() -> None:
    rates = _healthy_rates()
    rates[cfg.FB_CHASSIS_SPEED] = 0.0
    for phase in ("phase1", "phase2"):
        errors = check_rates(rates, phase)
        assert any(cfg.FB_CHASSIS_SPEED in e for e in errors)


def test_unpinned_topic_always_fails_arm_phase() -> None:
    errors = check_rates(_healthy_rates(), "arm")
    assert any(cfg.FB_CHASSIS_SPEED in e and "pin" in e for e in errors)


# check_matrix


def test_phase1_requires_zero_publishers_everywhere() -> None:
    publishers = _phase_publishers("phase1")
    assert check_matrix(publishers, "phase1") == []
    publishers[cfg.CMD_ARM_LEFT] = ["gello_node"]
    errors = check_matrix(publishers, "phase1")
    assert any("gello_node" in e for e in errors)


def test_sole_writer_column_requires_exactly_dimos() -> None:
    publishers = _phase_publishers("arm")
    assert check_matrix(publishers, "arm") == []
    assert check_matrix(publishers, "phase2") == []


def test_sole_writer_rejects_missing_dimos_publisher() -> None:
    publishers = _phase_publishers("arm")
    publishers[cfg.CMD_CHASSIS_SPEED] = []
    errors = check_matrix(publishers, "arm")
    assert any("not running" in e for e in errors)


def test_sole_writer_rejects_substring_impostor() -> None:
    publishers = _phase_publishers("arm")
    publishers[cfg.CMD_ARM_RIGHT] = [f"not_{cfg.ROS_NODE_NAME}"]
    errors = check_matrix(publishers, "arm")
    assert any("not_" in e for e in errors)


def test_sole_writer_rejects_second_publisher() -> None:
    publishers = _phase_publishers("arm")
    publishers[cfg.CMD_ARM_LEFT] = [cfg.ROS_NODE_NAME, "relaxed_ik_left"]
    errors = check_matrix(publishers, "arm")
    assert any("relaxed_ik_left" in str(e) for e in errors)


def test_torso_topics_zero_in_every_phase() -> None:
    for phase in ("phase1", "phase2", "arm"):
        publishers = _phase_publishers(phase)
        publishers[cfg.CMD_TORSO_JOINT] = [cfg.ROS_NODE_NAME]
        errors = check_matrix(publishers, phase)
        assert any(cfg.CMD_TORSO_JOINT in e for e in errors)


# wait_for_status


def test_wait_for_status_success_and_timeout() -> None:
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


# run_phase flow


class TestPhase2:
    def test_passes_disarmed_and_never_arms(self, capsys: pytest.CaptureFixture[str]) -> None:
        preflight.run_phase(
            "phase2",
            _healthy_rates(),
            _phase_publishers("phase2"),
            read_status=_disarmed_status,
            ask=_never_ask,
            send_arm=_never_arm,
        )
        assert "PASS phase2" in capsys.readouterr().out

    def test_requires_ready_disarmed(self) -> None:
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase2",
                _healthy_rates(),
                _phase_publishers("phase2"),
                read_status=lambda: {"state": "ARMED", "nonce": "abc123"},
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_requires_a_nonce(self) -> None:
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase2",
                _healthy_rates(),
                _phase_publishers("phase2"),
                read_status=lambda: {"state": "READY_DISARMED"},
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_uses_the_sole_writer_column(self) -> None:
        publishers = _phase_publishers("phase2")
        publishers[cfg.CMD_ARM_LEFT] = []
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase2",
                _healthy_rates(),
                publishers,
                read_status=_disarmed_status,
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_rejects_a_foreign_publisher(self) -> None:
        publishers = _phase_publishers("phase2")
        publishers[cfg.CMD_ARM_LEFT] = ["intruder_node"]
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase2",
                _healthy_rates(),
                publishers,
                read_status=_disarmed_status,
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_unpinned_nominal_lenient_in_phase2_strict_in_arm(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.setattr(cfg, "FEEDBACK_NOMINAL_HZ", {"/x": None})
        assert check_rates({"/x": 5.0}, "phase2") == []
        assert check_rates({"/x": 5.0}, "arm") != []
        assert check_rates({"/x": 0.0}, "phase2") != []


class TestPhase1:
    def test_rejects_any_command_publisher(self) -> None:
        publishers = _phase_publishers("phase1")
        publishers[cfg.CMD_ARM_LEFT] = [cfg.ROS_NODE_NAME]
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase1",
                _healthy_rates(),
                publishers,
                read_status=_disarmed_status,
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_passes_with_zero_publishers(self, capsys: pytest.CaptureFixture[str]) -> None:
        preflight.run_phase(
            "phase1",
            _healthy_rates(),
            _phase_publishers("phase1"),
            read_status=lambda: pytest.fail("phase1 must not read connection status"),
            ask=_never_ask,
            send_arm=_never_arm,
        )
        assert "PASS phase1" in capsys.readouterr().out


class TestArm:
    @pytest.fixture(autouse=True)
    def _pin_all_nominals(self, monkeypatch: pytest.MonkeyPatch) -> None:
        # The real config keeps arming closed via an unpinned nominal. Pin
        # everything here so these tests reach the attestation path.
        pinned = {
            topic: (nominal if nominal is not None else 50.0)
            for topic, nominal in cfg.FEEDBACK_NOMINAL_HZ.items()
        }
        monkeypatch.setattr(cfg, "FEEDBACK_NOMINAL_HZ", pinned)

    def test_unpinned_nominal_keeps_arming_closed(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr(cfg, "FEEDBACK_NOMINAL_HZ", {"/x": None})
        sent: list[str] = []
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "arm",
                {"/x": 5.0},
                _phase_publishers("arm"),
                read_status=_disarmed_status,
                ask=_never_ask,
                send_arm=sent.append,
            )
        assert sent == []

    def test_wrong_attestation_never_arms(self) -> None:
        sent: list[str] = []
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "arm",
                _healthy_rates(),
                _phase_publishers("arm"),
                read_status=_disarmed_status,
                ask=lambda prompt: "wrong text",
                send_arm=sent.append,
            )
        assert sent == []

    def test_arms_after_exact_attestation_and_observed_transition(
        self, capsys: pytest.CaptureFixture[str]
    ) -> None:
        calls = {"n": 0}

        def read_status() -> dict[str, str]:
            calls["n"] += 1
            state = "READY_DISARMED" if calls["n"] == 1 else "ARMED"
            return {"state": state, "nonce": "abc123"}

        sent: list[str] = []
        preflight.run_phase(
            "arm",
            _healthy_rates(),
            _phase_publishers("arm"),
            read_status=read_status,
            ask=lambda prompt: "ARM RC5 abc123",
            send_arm=sent.append,
        )
        assert sent == ["abc123"]
        assert "PASS arm" in capsys.readouterr().out


# run_disarm flow


class TestRunDisarm:
    def test_disarms_from_armed_and_requires_nonce_rotation(
        self, capsys: pytest.CaptureFixture[str]
    ) -> None:
        sent = {"n": 0}
        state = {"value": {"state": "ARMED", "nonce": "old111"}}

        def send_disarm() -> None:
            sent["n"] += 1
            state["value"] = {"state": "READY_DISARMED", "nonce": "new222"}

        preflight.run_disarm(lambda: dict(state["value"]), send_disarm)
        assert sent["n"] == 1
        assert "PASS disarm" in capsys.readouterr().out

    def test_fails_when_nonce_does_not_rotate(self) -> None:
        state = {"value": {"state": "ARMED", "nonce": "old111"}}

        def send_disarm() -> None:
            state["value"] = {"state": "READY_DISARMED", "nonce": "old111"}

        with pytest.raises(SystemExit):
            preflight.run_disarm(lambda: dict(state["value"]), send_disarm)

    def test_fails_when_state_never_reaches_ready_disarmed(self) -> None:
        clock = {"t": 0.0}

        def now() -> float:
            return clock["t"]

        def sleep(seconds: float) -> None:
            clock["t"] += seconds

        with pytest.raises(SystemExit):
            preflight.run_disarm(
                lambda: {"state": "ARMED", "nonce": "old111"},
                lambda: None,
                now=now,
                sleep=sleep,
            )

    def test_fails_without_any_status(self) -> None:
        sent = {"n": 0}

        def send_disarm() -> None:
            sent["n"] += 1

        with pytest.raises(SystemExit):
            preflight.run_disarm(lambda: {}, send_disarm)
        assert sent["n"] == 0

    def test_idempotent_when_already_disarmed(self, capsys: pytest.CaptureFixture[str]) -> None:
        preflight.run_disarm(
            lambda: {"state": "READY_DISARMED", "nonce": "abc123"},
            lambda: None,
        )
        assert "PASS disarm" in capsys.readouterr().out
