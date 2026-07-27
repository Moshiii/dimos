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

"""Deterministic tests for the container entrypoint gate decisions."""

from __future__ import annotations

import pytest

from dimos.robot.galaxea.r1lite import config as cfg
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
