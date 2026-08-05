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

from typing import Any

import pytest

from dimos.agents.code_policy_core import CodePolicySessionConfig
from dimos.benchmark.agent_eval.live import compile_dimsim_case, run_live_case
from dimos.benchmark.agent_eval.test_runner import (
    FakeMcp,
    NativeResult,
    _setup,
    _tool,
)


class _StandaloneInventoryMcp(FakeMcp):
    def list_tools(self) -> list[dict[str, object]]:
        return [_tool()]


class _FakeStandaloneProcess:
    mcp_url = "http://127.0.0.1:32123/mcp"

    def __init__(self, config: CodePolicySessionConfig) -> None:
        self.config = config
        self.closed = False

    def start(self, timeout_s: float = 10.0) -> None:
        assert timeout_s > 0

    def receipt(self) -> dict[str, Any]:
        return {
            "session_id": "code_policy_session_" + "c" * 32,
            "reset_at": "2026-01-01T00:00:00Z",
            "previous_session_id": None,
        }

    def records(self, session_id: str | None = None) -> list[dict[str, Any]]:
        del session_id
        return []

    def close(self) -> None:
        self.closed = True


def test_generated_dimsim_destination_compiles_to_canonical_case(tmp_path) -> None:
    runner, _, _, _ = _setup(tmp_path, None, [])

    case = compile_dimsim_case(runner.selected)

    assert case.source.kind == "live_dimos"
    assert case.task.kind == "embodied_instruction"
    assert case.interaction.kind == "live_code_policy"
    assert case.validator.kind == "native"
    assert case.task.prompt == runner.selected.public.text
    assert case.validator.contract_sha256 == runner.selected.contract_sha256


@pytest.mark.parametrize(
    ("native", "expected_result"),
    [
        (
            NativeResult(
                evaluation_id="eval-canonical-pass",
                passed=True,
                reason="predicate_satisfied",
            ),
            "passed",
        ),
        (
            NativeResult(
                evaluation_id="eval-canonical-fail",
                passed=False,
                reason="deadline_exceeded",
            ),
            "failed",
        ),
        (None, "failed"),
    ],
)
def test_canonical_live_engine_preserves_native_scoring_and_evidence(
    tmp_path, native: NativeResult | None, expected_result: str
) -> None:
    runner, backend, motion, factory = _setup(
        tmp_path,
        native,
        [1],
    )

    result = run_live_case(
        selected=runner.selected,
        backend=backend,
        motion=motion,
        memory_path=str(tmp_path / "live.db"),
        output_root=tmp_path / "canonical-attempts",
        pi_factory=factory,
        timeouts=runner.config.timeouts,
        episode_timeout_s=runner.config.episode_timeout_s,
        process_factory=_FakeStandaloneProcess,
        mcp_factory=lambda _endpoint, _timeout: _StandaloneInventoryMcp(),
    )

    assert result.outcome.attempt_status == "completed"
    assert result.outcome.task_result == expected_result
    assert (result.attempt_path / "dimsim-result.v1.json").is_file()
    assert (result.attempt_path / "source-reset.private.v1.json").is_file()
    assert (result.attempt_path / "code-policy-calls.jsonl").is_file()
    assert (result.attempt_path / "attempt-manifest.v1.json").is_file()
    assert "pi-continuation" in (result.attempt_path / "events.jsonl").read_text()
    assert backend.cancelled and backend.cleaned and motion.cancelled
