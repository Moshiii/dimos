# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.agents.code_policy_core import (
    CodePolicySession,
    CodePolicySessionConfig,
    FrozenMemoryEnvironment,
    LiveDimosEnvironment,
)
from dimos.memory2.store.sqlite import SqliteStore


def test_plain_session_persists_and_resets_without_module(mocker, tmp_path: Path) -> None:
    mocker.patch("dimos.agents.code_policy_core._bootstrap_source", return_value="pass")
    session = CodePolicySession(
        CodePolicySessionConfig(
            environment=LiveDimosEnvironment(recording_path=str(tmp_path / "unused.db"))
        )
    )
    session.start()
    try:
        assert "[1]" in session.python_exec("items = [1]\nitems")
        assert "[1, 2]" in session.python_exec("items.append(2)\nitems")
        first = session.get_session_receipt()
        second = session.reset_session()
        assert second.previous_session_id == first.session_id
        assert "NameError" in session.python_exec("items")
    finally:
        session.stop()


def test_frozen_session_bootstrap_exposes_memory_without_app(tmp_path: Path) -> None:
    source_path = tmp_path / "source.db"
    derived_path = tmp_path / "derived.db"
    with SqliteStore(path=str(source_path)) as source:
        source.stream("messages", str).append("before", ts=1.0)
        source.stream("messages", str).append("after", ts=3.0)
    with SqliteStore(path=str(derived_path)) as derived:
        derived.stream("global_map", str).append("map", ts=2.0)

    session = CodePolicySession(
        CodePolicySessionConfig(
            environment=FrozenMemoryEnvironment(
                recording_path=str(source_path),
                derived_recording_path=str(derived_path),
                memory_cutoff_timestamp=2.0,
            )
        )
    )
    session.start()
    try:
        result = session.python_exec(
            "([item.data for item in memory.streams.messages], "
            "memory.streams.global_map.last().data, 'app' in globals())"
        )
        assert "(['before'], 'map', False)" in result
    finally:
        session.stop()


def test_live_read_only_memory_observes_committed_writer_data(mocker, tmp_path: Path) -> None:
    path = tmp_path / "live.db"
    with SqliteStore(path=str(path)) as writer:
        writer.stream("events", int).append(1, ts=1.0)

    bootstrap = f"""
from dimos.memory2.store.sqlite import SqliteStore
memory = SqliteStore(path={str(path)!r}, must_exist=True, read_only=True)
memory.start()
"""
    mocker.patch("dimos.agents.code_policy_core._bootstrap_source", return_value=bootstrap)
    session = CodePolicySession(
        CodePolicySessionConfig(environment=LiveDimosEnvironment(recording_path=str(path)))
    )
    session.start()
    try:
        assert "\n\n1" in session.python_exec("memory.streams.events.count()")
        with SqliteStore(path=str(path)) as writer:
            writer.stream("events", int).append(2, ts=2.0)
        assert "\n\n2" in session.python_exec("memory.streams.events.count()")
        mutation = session.python_exec("memory.streams.events.append(3)")
        assert "PermissionError" in mutation
    finally:
        session.stop()


def test_frozen_environment_requires_all_fields() -> None:
    with pytest.raises(ValueError):
        FrozenMemoryEnvironment.model_validate(
            {"kind": "frozen_memory", "recording_path": "source.db"}
        )
