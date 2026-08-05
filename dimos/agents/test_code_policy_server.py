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

import requests

from dimos.agents.code_policy_core import (
    CodePolicySessionConfig,
    FrozenMemoryEnvironment,
)
from dimos.agents.code_policy_server import (
    StandaloneCodePolicyProcess,
    StandaloneCodePolicyServer,
)
from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.benchmark.agent_eval.pi_adapter import inspect_python_exec_inventory
from dimos.core.module import Module
from dimos.memory2.store.sqlite import SqliteStore


def _config(tmp_path: Path) -> CodePolicySessionConfig:
    source = tmp_path / "source.db"
    derived = tmp_path / "derived.db"
    with SqliteStore(path=str(source)) as store:
        store.stream("messages", str).append("visible", ts=1.0)
    with SqliteStore(path=str(derived)) as store:
        store.stream("global_map", str).append("map", ts=1.0)
    return CodePolicySessionConfig(
        environment=FrozenMemoryEnvironment(
            recording_path=str(source),
            derived_recording_path=str(derived),
            memory_cutoff_timestamp=1.0,
        )
    )


def test_standalone_server_has_exact_direct_mcp_surface(tmp_path: Path) -> None:
    server = StandaloneCodePolicyServer(_config(tmp_path))
    assert not isinstance(server, Module)
    server.start()
    adapter = McpAdapter(server.mcp_url, timeout=5)
    try:
        tools = adapter.list_tools()
        assert [tool["name"] for tool in tools] == ["python_exec"]
        inspect_python_exec_inventory(server.mcp_url, tools)
        first = adapter.call_tool_text("python_exec", {"code": "items = [1]\nitems"})
        second = adapter.call_tool_text("python_exec", {"code": "items.append(2)\nitems"})
        assert "[1]" in first
        assert "[1, 2]" in second
        receipt = server.session.get_session_receipt()
        assert len(server.session.get_execution_records(receipt.session_id)) == 2
    finally:
        server.stop()
    assert adapter.wait_for_down(timeout=2, interval=0.05)


def test_standalone_process_control_resets_and_collects_records(tmp_path: Path) -> None:
    process = StandaloneCodePolicyProcess(_config(tmp_path))
    process.start()
    adapter = McpAdapter(process.mcp_url, timeout=5)
    try:
        receipt = process.receipt()
        result = adapter.call_tool_text(
            "python_exec", {"code": "memory.streams.messages.last().data"}
        )
        assert "visible" in result
        assert "app" not in adapter.call_tool_text("python_exec", {"code": "sorted(globals())"})
        assert len(process.records(receipt["session_id"])) == 2
        reset = process.reset()
        assert reset["previous_session_id"] == receipt["session_id"]
        assert reset["session_id"] != receipt["session_id"]
    finally:
        process.close()
    assert process.process is None


def test_server_stops_after_execution_start_failure(tmp_path: Path) -> None:
    config = CodePolicySessionConfig(
        environment=FrozenMemoryEnvironment(
            recording_path=str(tmp_path / "missing.db"),
            derived_recording_path=str(tmp_path / "also-missing.db"),
            memory_cutoff_timestamp=1.0,
        )
    )
    server = StandaloneCodePolicyServer(config)
    server.start()
    adapter = McpAdapter(server.mcp_url, timeout=5)
    result = adapter.call_tool_text("python_exec", {"code": "1 + 1"})
    assert "failed to start" in result
    server.stop()
    try:
        requests.post(server.mcp_url, timeout=0.2)
    except (requests.ConnectionError, requests.ReadTimeout):
        pass
    else:
        raise AssertionError("standalone service remained reachable after stop")
