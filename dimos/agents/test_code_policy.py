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

import asyncio
from collections.abc import Iterator

import pytest

from dimos.agents.code_policy import (
    CodePolicyModule,
    _BoundedTextOutput,
)
from dimos.agents.mcp.mcp_server import handle_request


@pytest.fixture
def code_policy(mocker, tmp_path) -> Iterator[CodePolicyModule]:
    mocker.patch("dimos.agents.code_policy._bootstrap_source", return_value="pass")
    module = CodePolicyModule(
        recording_path=str(tmp_path / "unused.db"),
        interrupt_grace_s=1.0,
    )
    module.start()
    try:
        yield module
    finally:
        module.stop()


def test_output_collector_keeps_only_bounded_plain_text() -> None:
    output = _BoundedTextOutput(limit=20)

    output(
        {
            "header": {"msg_type": "stream"},
            "content": {"name": "stdout", "text": "hello\n"},
        }
    )
    output(
        {
            "header": {"msg_type": "execute_result"},
            "content": {
                "data": {
                    "text/plain": "42",
                    "image/png": "ignored",
                }
            },
        }
    )
    output(
        {
            "header": {"msg_type": "display_data"},
            "content": {"data": {"text/html": "<b>ignored</b>"}},
        }
    )

    assert output.text() == "hello\n42"


def test_output_collector_truncates_combined_output() -> None:
    output = _BoundedTextOutput(limit=30)

    output(
        {
            "header": {"msg_type": "stream"},
            "content": {"name": "stdout", "text": "x" * 100},
        }
    )
    output(
        {
            "header": {"msg_type": "stream"},
            "content": {"name": "stdout", "text": "not collected"},
        }
    )

    assert output.text() == "x" * 7 + "\n... [output truncated]"


def test_output_collector_honors_limits_smaller_than_marker() -> None:
    output = _BoundedTextOutput(limit=5)

    output(
        {
            "header": {"msg_type": "stream"},
            "content": {"name": "stdout", "text": "x" * 100},
        }
    )

    assert output.text() == "\n... "
    assert len(output.text()) == 5


def test_code_policy_exposes_only_python_exec_skill(code_policy: CodePolicyModule) -> None:
    assert [skill.func_name for skill in code_policy.get_skills()] == ["python_exec"]


def test_mcp_lists_python_exec(code_policy: CodePolicyModule) -> None:
    skills = code_policy.get_skills()

    response = asyncio.run(
        handle_request(
            {"method": "tools/list", "id": 1},
            skills,
            {"python_exec": code_policy.python_exec},
        )
    )

    assert response is not None
    assert [tool["name"] for tool in response["result"]["tools"]] == ["python_exec"]


def test_python_exec_persists_namespace_and_returns_text_output(
    code_policy: CodePolicyModule,
) -> None:
    first = code_policy.python_exec("marker = 'persistent'\nprint('hello from kernel')\n6 * 7")
    second = code_policy.python_exec("marker")

    assert "completed" in first
    assert "hello from kernel\n42" in first
    assert "completed" in second
    assert "'persistent'" in second


def test_python_error_preserves_namespace(code_policy: CodePolicyModule) -> None:
    failed = code_policy.python_exec(
        "items = ['before error']\nraise ValueError('expected failure')"
    )
    recovered = code_policy.python_exec("items")

    assert "failed" in failed
    assert "ValueError" in failed
    assert "expected failure" in failed
    assert "['before error']" in recovered


def test_timeout_interrupts_and_preserves_responsive_kernel(
    code_policy: CodePolicyModule,
) -> None:
    timed_out = code_policy.python_exec(
        "import time\nmarker = 'survived interrupt'\ntime.sleep(10)",
        timeout_s=0.1,
    )
    recovered = code_policy.python_exec("marker")

    assert "was interrupted" in timed_out
    assert "namespace was preserved" in timed_out
    assert "'survived interrupt'" in recovered


def test_timeout_restarts_kernel_when_interrupt_does_not_recover(
    mocker, code_policy: CodePolicyModule
) -> None:
    manager = mocker.Mock()
    client = mocker.Mock()
    client.execute_interactive.side_effect = TimeoutError
    client.wait_for_ready.side_effect = [TimeoutError, None]
    code_policy._kernel_manager = manager
    code_policy._kernel_client = client
    mocker.patch.object(code_policy, "_bootstrap")

    result = code_policy.python_exec("while True: pass", timeout_s=0.1)

    manager.interrupt_kernel.assert_called_once_with()
    manager.restart_kernel.assert_called_once_with(now=True)
    assert "was restarted" in result
    assert "namespace was reset" in result


def test_python_exec_rejects_invalid_timeout_without_starting_kernel(
    mocker, code_policy: CodePolicyModule
) -> None:
    ensure_kernel = mocker.patch.object(code_policy, "_ensure_kernel")

    result = code_policy.python_exec("pass", timeout_s=111.0)

    assert "Invalid timeout" in result
    ensure_kernel.assert_not_called()


def test_python_exec_rejects_concurrent_call(code_policy: CodePolicyModule) -> None:
    assert code_policy._execution_lock is not None
    code_policy._execution_lock.acquire()
    try:
        result = code_policy.python_exec("pass")
    finally:
        code_policy._execution_lock.release()

    assert result == "Code Policy Module busy: another python_exec call is active"
