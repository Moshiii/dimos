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

from dimos.agents.code_policy.session import PolicySession, format_cell_result


def test_session_preloads_native_handles_and_preserves_state() -> None:
    app = object()
    memory = object()
    session = PolicySession(app=app, memory=memory)

    first = session.execute("value = 40")
    second = session.execute("value + 2")

    assert first.success
    assert session.user_ns["app"] is app
    assert session.user_ns["memory"] is memory
    assert second.success
    assert "Out[2]: 42" in second.stdout


def test_session_preserves_mutations_before_ordinary_exception() -> None:
    session = PolicySession(app=object(), memory=object())

    failed = session.execute("value = 'kept'\nraise ValueError('bad policy')")
    recovered = session.execute("value")

    assert not failed.success
    assert failed.error_type == "ValueError"
    assert "ValueError: bad policy" in failed.stdout
    assert recovered.success
    assert "'kept'" in recovered.stdout


def test_session_captures_stdout_stderr_and_final_expression() -> None:
    session = PolicySession(app=object(), memory=object())

    result = session.execute(
        "import sys\nprint('out')\nprint('err', file=sys.stderr)\n{'answer': 42}"
    )
    transcript = format_cell_result(result)

    assert result.success
    assert "out" in transcript
    assert "err" in transcript
    assert "{'answer': 42}" in transcript


def test_session_returns_syntax_error_traceback() -> None:
    session = PolicySession(app=object(), memory=object())

    result = session.execute("if True print('missing colon')")

    assert not result.success
    assert result.error_type == "SyntaxError"
    assert "SyntaxError" in result.stdout


def test_session_truncates_oversized_output_and_remains_usable() -> None:
    session = PolicySession(app=object(), memory=object(), output_limit=80)

    oversized = session.execute("print('x' * 500)")
    following = session.execute("6 * 7")

    assert len(oversized.stdout) <= 80
    assert "[output truncated]" in oversized.stdout
    assert following.success
    assert "42" in following.stdout
