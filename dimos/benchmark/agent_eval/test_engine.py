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
from typing import Any

import pytest

from dimos.benchmark.agent_eval.case import (
    AgentCondition,
    AgentOutcome,
    AttemptRequest,
    EvalCase,
    ExactIntegerValidatorRef,
    FrozenCodePolicyInteraction,
    FrozenRecordingSource,
    IntegerQuestionTask,
    PrivateScore,
    RuntimeBinding,
)
from dimos.benchmark.agent_eval.engine import AttemptEngine
from dimos.benchmark.agent_eval.interfaces import AttemptContext, PreparedSource
from dimos.benchmark.agent_eval.store import AttemptStore


def _request() -> AttemptRequest:
    case = EvalCase.compile(
        case_id="case",
        source=FrozenRecordingSource(recording="recording", progress=1.0),
        task=IntegerQuestionTask(prompt="Question"),
        interaction=FrozenCodePolicyInteraction(driver_revision="v1"),
        validator=ExactIntegerValidatorRef(
            revision="v1",
            private_path="private/oracle.json",
            private_sha256="a" * 64,
        ),
    )
    return AttemptRequest(
        case=case,
        agent=AgentCondition(
            agent_id="agent",
            adapter="fake",
            model="fake",
            thinking_level="off",
        ),
        runtime=RuntimeBinding(runtime_id="local"),
    )


class FakeAgent:
    def __init__(self, fail: bool = False, cleanup_fail: bool = False) -> None:
        self.fail = fail
        self.cleanup_fail = cleanup_fail
        self.closed = False

    def run(self, *, task: Any, context: AttemptContext, interface: Any = None) -> AgentOutcome:
        del task, context, interface
        if self.fail:
            raise RuntimeError("agent failed")
        return AgentOutcome(
            final_text="ANSWER: 4", tool_call_count=1, terminal_reason="agent completed"
        )

    def close(self) -> None:
        self.closed = True
        if self.cleanup_fail:
            raise RuntimeError("agent cleanup failed")


class FakeSource:
    def __init__(self, fail: bool = False) -> None:
        self.fail = fail
        self.closed = False

    def prepare(self, *, source: Any, context: AttemptContext, evidence: Any) -> PreparedSource:
        del source, context
        evidence.artifact("source-receipt.v1.json", {"ready": True})
        if self.fail:
            raise RuntimeError("source failed")
        return PreparedSource(public={"ready": True}, receipt={"source": "fake"})

    def close(self) -> None:
        self.closed = True


class FakeInteraction:
    def __init__(self, fail: bool = False) -> None:
        self.fail = fail
        self.closed = False

    def run(
        self,
        *,
        case: EvalCase,
        prepared_source: PreparedSource,
        agent: FakeAgent,
        context: AttemptContext,
        evidence: Any,
    ) -> AgentOutcome:
        del case, prepared_source, evidence
        if self.fail:
            raise RuntimeError("interaction failed")
        return agent.run(task=context.request.case.task, context=context)

    def close(self) -> None:
        self.closed = True


class FakeValidatorSession:
    def __init__(
        self, context: AttemptContext, *, passed: bool, evaluate_fail: bool = False
    ) -> None:
        self.context = context
        self.passed = passed
        self.evaluate_fail = evaluate_fail
        self.closed = False

    def evaluate(self, outcome: AgentOutcome) -> PrivateScore:
        del outcome
        if self.evaluate_fail:
            raise RuntimeError("validation failed")
        return PrivateScore(
            case_id=self.context.request.case.case_id,
            attempt_id=self.context.attempt_id,
            validator_revision="v1",
            passed=self.passed,
            prediction_status="parsed" if self.passed else "invalid",
        )

    def close(self) -> None:
        self.closed = True


class FakeValidator:
    def __init__(
        self, *, passed: bool = True, prepare_fail: bool = False, evaluate_fail: bool = False
    ) -> None:
        self.passed = passed
        self.prepare_fail = prepare_fail
        self.evaluate_fail = evaluate_fail

    def prepare(
        self,
        *,
        case: EvalCase,
        prepared_source: PreparedSource,
        context: AttemptContext,
        evidence: Any,
    ) -> FakeValidatorSession:
        del case, prepared_source, evidence
        if self.prepare_fail:
            raise RuntimeError("validator prepare failed")
        return FakeValidatorSession(context, passed=self.passed, evaluate_fail=self.evaluate_fail)


def _run(tmp_path: Path, **kwargs: Any):
    source = FakeSource(fail=kwargs.get("source_fail", False))
    interaction = FakeInteraction(fail=kwargs.get("interaction_fail", False))
    agent = FakeAgent(
        fail=kwargs.get("agent_fail", False),
        cleanup_fail=kwargs.get("cleanup_fail", False),
    )
    validator = FakeValidator(
        passed=kwargs.get("passed", True),
        prepare_fail=kwargs.get("validator_prepare_fail", False),
        evaluate_fail=kwargs.get("validator_evaluate_fail", False),
    )
    result = AttemptEngine(
        request=_request(),
        output_root=tmp_path,
        source=source,
        interaction=interaction,
        validator=validator,
        agent=agent,
    ).run()
    return result, source, interaction, agent


@pytest.mark.parametrize("passed", [True, False])
def test_engine_completes_pass_and_wrong_answer(tmp_path: Path, passed: bool) -> None:
    result, source, interaction, agent = _run(tmp_path, passed=passed)
    assert result.outcome.attempt_status == "completed"
    assert result.outcome.task_result == ("passed" if passed else "failed")
    assert (result.attempt_path / "score.private.v1.json").is_file()
    assert source.closed and interaction.closed and agent.closed


@pytest.mark.parametrize(
    "failure",
    [
        "source_fail",
        "interaction_fail",
        "agent_fail",
        "validator_prepare_fail",
        "validator_evaluate_fail",
    ],
)
def test_engine_retains_failed_prefix_and_not_evaluated(tmp_path: Path, failure: str) -> None:
    result, source, interaction, agent = _run(tmp_path, **{failure: True})
    assert result.outcome.attempt_status == "failed"
    assert result.outcome.task_result == "not_evaluated"
    assert (result.attempt_path / "outcome.v1.json").is_file()
    assert (result.attempt_path / "events.jsonl").is_file()
    assert source.closed and interaction.closed and agent.closed


def test_cleanup_failure_invalidates_completed_attempt(tmp_path: Path) -> None:
    result, _, _, _ = _run(tmp_path, cleanup_fail=True)
    assert result.outcome.attempt_status == "failed"
    assert result.outcome.task_result == "not_evaluated"
    assert "cleanup" in result.outcome.reason


def test_engine_writes_exactly_one_terminal_outcome(tmp_path: Path) -> None:
    result, _, _, _ = _run(tmp_path)
    assert [path.name for path in result.attempt_path.glob("outcome*")] == ["outcome.v1.json"]


def test_manifest_write_failure_returns_failed_attempt_and_releases_lock(
    tmp_path: Path, mocker
) -> None:
    original = AttemptStore.write_artifact

    def fail_manifest(store, relative_path, value):
        if relative_path == "attempt-manifest.v1.json":
            raise OSError("manifest disk failure")
        return original(store, relative_path, value)

    mocker.patch.object(AttemptStore, "write_artifact", autospec=True, side_effect=fail_manifest)

    result, _, _, _ = _run(tmp_path)

    assert result.outcome.attempt_status == "failed"
    assert "finalization" in result.outcome.reason
    with AttemptStore(tmp_path) as subsequent:
        assert subsequent.path != result.attempt_path


def test_terminal_publication_failure_returns_failed_attempt_and_releases_lock(
    tmp_path: Path, mocker
) -> None:
    mocker.patch.object(
        AttemptStore,
        "write_eval_outcome",
        autospec=True,
        side_effect=OSError("terminal link failure"),
    )

    result, _, _, _ = _run(tmp_path)

    assert result.outcome.attempt_status == "failed"
    assert "terminal publication" in result.outcome.reason
    assert not (result.attempt_path / "outcome.v1.json").exists()
    with AttemptStore(tmp_path) as subsequent:
        assert subsequent.path != result.attempt_path


def test_event_fsync_failure_still_cleans_resources_and_releases_lock(
    tmp_path: Path, mocker
) -> None:
    mocker.patch("dimos.benchmark.agent_eval.store.os.fsync", side_effect=OSError("fsync failed"))

    result, source, interaction, agent = _run(tmp_path)

    assert result.outcome.attempt_status == "failed"
    assert source.closed and interaction.closed and agent.closed
    with AttemptStore(tmp_path) as subsequent:
        assert subsequent.path != result.attempt_path
