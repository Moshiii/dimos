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

"""Deterministic tests for the preflight decision flow. No ROS required.

Run with: pytest scripts/r1lite_test/
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    "r1lite_preflight", Path(__file__).with_name("preflight.py")
)
assert _spec is not None and _spec.loader is not None
preflight = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(preflight)

cfg = preflight.cfg


def _never_ask(prompt: str) -> str:
    pytest.fail("this phase must not prompt for attestation")


def _never_arm(nonce: str) -> None:
    pytest.fail("this phase must not reach the arming publisher")


def _good_rates() -> dict[str, float]:
    return {t: (n if n is not None else 25.0) for t, n in cfg.FEEDBACK_NOMINAL_HZ.items()}


def _sole_writer_publishers() -> dict[str, list[str]]:
    return {
        t: ([cfg.ROS_NODE_NAME] if counts[1] == 1 else [])
        for t, counts in cfg.ARMING_MATRIX.items()
    }


def _disarmed_status() -> dict[str, str]:
    return {"state": "READY_DISARMED", "nonce": "abc123"}


class TestPhase2:
    def test_passes_disarmed_and_never_arms(self, capsys):
        preflight.run_phase(
            "phase2",
            _good_rates(),
            _sole_writer_publishers(),
            read_status=_disarmed_status,
            ask=_never_ask,
            send_arm=_never_arm,
        )
        assert "PASS phase2" in capsys.readouterr().out

    def test_requires_ready_disarmed(self):
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase2",
                _good_rates(),
                _sole_writer_publishers(),
                read_status=lambda: {"state": "ARMED", "nonce": "abc123"},
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_requires_a_nonce(self):
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase2",
                _good_rates(),
                _sole_writer_publishers(),
                read_status=lambda: {"state": "READY_DISARMED"},
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_uses_the_sole_writer_column(self):
        publishers = _sole_writer_publishers()
        topic = next(t for t, counts in cfg.ARMING_MATRIX.items() if counts[1] == 1)
        publishers[topic] = []
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase2",
                _good_rates(),
                publishers,
                read_status=_disarmed_status,
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_rejects_a_foreign_publisher(self):
        publishers = _sole_writer_publishers()
        topic = next(t for t, counts in cfg.ARMING_MATRIX.items() if counts[1] == 1)
        publishers[topic] = ["intruder_node"]
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase2",
                _good_rates(),
                publishers,
                read_status=_disarmed_status,
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_unpinned_nominal_is_lenient_in_phase2_and_strict_in_arm(self, monkeypatch):
        monkeypatch.setattr(cfg, "FEEDBACK_NOMINAL_HZ", {"/x": None})
        assert preflight.check_rates({"/x": 5.0}, "phase2") == []
        assert preflight.check_rates({"/x": 5.0}, "arm") != []
        assert preflight.check_rates({"/x": 0.0}, "phase2") != []


class TestPhase1:
    def test_rejects_any_command_publisher(self):
        publishers = {t: [] for t in cfg.ARMING_MATRIX}
        publishers[next(iter(cfg.ARMING_MATRIX))] = [cfg.ROS_NODE_NAME]
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "phase1",
                _good_rates(),
                publishers,
                read_status=_disarmed_status,
                ask=_never_ask,
                send_arm=_never_arm,
            )

    def test_passes_with_zero_publishers(self, capsys):
        preflight.run_phase(
            "phase1",
            _good_rates(),
            {t: [] for t in cfg.ARMING_MATRIX},
            read_status=lambda: pytest.fail("phase1 must not read connection status"),
            ask=_never_ask,
            send_arm=_never_arm,
        )
        assert "PASS phase1" in capsys.readouterr().out


class TestArm:
    @pytest.fixture(autouse=True)
    def _pin_all_nominals(self, monkeypatch):
        # The real config keeps arming closed via an unpinned nominal. Pin
        # everything here so these tests reach the attestation path.
        pinned = {t: (n if n is not None else 50.0) for t, n in cfg.FEEDBACK_NOMINAL_HZ.items()}
        monkeypatch.setattr(cfg, "FEEDBACK_NOMINAL_HZ", pinned)

    def test_unpinned_nominal_keeps_arming_closed(self, monkeypatch):
        monkeypatch.setattr(cfg, "FEEDBACK_NOMINAL_HZ", {"/x": None})
        sent: list[str] = []
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "arm",
                {"/x": 5.0},
                _sole_writer_publishers(),
                read_status=_disarmed_status,
                ask=_never_ask,
                send_arm=sent.append,
            )
        assert sent == []

    def test_wrong_attestation_never_arms(self):
        sent: list[str] = []
        with pytest.raises(SystemExit):
            preflight.run_phase(
                "arm",
                _good_rates(),
                _sole_writer_publishers(),
                read_status=_disarmed_status,
                ask=lambda prompt: "wrong text",
                send_arm=sent.append,
            )
        assert sent == []

    def test_arms_after_exact_attestation_and_observed_transition(self, capsys):
        calls = {"n": 0}

        def read_status() -> dict[str, str]:
            calls["n"] += 1
            state = "READY_DISARMED" if calls["n"] == 1 else "ARMED"
            return {"state": state, "nonce": "abc123"}

        sent: list[str] = []
        preflight.run_phase(
            "arm",
            _good_rates(),
            _sole_writer_publishers(),
            read_status=read_status,
            ask=lambda prompt: "ARM RC5 abc123",
            send_arm=sent.append,
        )
        assert sent == ["abc123"]
        assert "PASS arm" in capsys.readouterr().out
