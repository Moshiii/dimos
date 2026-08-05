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

import json

from pydantic import ValidationError
import pytest

from dimos.benchmark.agent_eval.case import (
    AgentCondition,
    AttemptRequest,
    EvalCase,
    EvalOutcome,
    ExactIntegerValidatorRef,
    FrozenCodePolicyInteraction,
    FrozenRecordingSource,
    IntegerQuestionTask,
    Prediction,
    RuntimeBinding,
)


def _case(prompt: str = "How many rooms?") -> EvalCase:
    return EvalCase.compile(
        case_id="office-room-count",
        source=FrozenRecordingSource(recording="office", progress=1.0),
        task=IntegerQuestionTask(prompt=prompt),
        interaction=FrozenCodePolicyInteraction(driver_revision="v1"),
        validator=ExactIntegerValidatorRef(
            revision="exact-v1",
            private_path="private/oracle.json",
            private_sha256="a" * 64,
        ),
    )


def test_compiled_case_has_stable_fingerprint_and_public_projection() -> None:
    first = _case()
    second = _case()

    assert first.fingerprint == second.fingerprint
    public = first.public_projection().model_dump(mode="json")
    assert "validator" not in public
    assert "private/oracle.json" not in json.dumps(public)
    assert "a" * 64 not in json.dumps(public)


def test_case_fingerprint_binds_task_and_private_validator() -> None:
    assert _case("How many rooms?").fingerprint != _case("Count the rooms.").fingerprint
    encoded = _case().model_dump(mode="json")
    encoded["fingerprint"] = "0" * 64
    with pytest.raises(ValidationError, match="fingerprint"):
        EvalCase.model_validate(encoded)


def test_case_rejects_missing_contract_and_unknown_discriminator() -> None:
    encoded = _case().model_dump(mode="json")
    del encoded["validator"]
    with pytest.raises(ValidationError):
        EvalCase.model_validate(encoded)

    encoded = _case().model_dump(mode="json")
    encoded["interaction"]["kind"] = "prompt_dump"
    with pytest.raises(ValidationError, match="frozen_code_policy"):
        EvalCase.model_validate(encoded)


@pytest.mark.parametrize("progress", [-0.1, 1.1, float("inf"), float("nan")])
def test_frozen_source_rejects_invalid_progress(progress: float) -> None:
    with pytest.raises(ValidationError):
        FrozenRecordingSource(recording="office", progress=progress)


def test_one_source_can_back_independent_tasks() -> None:
    first = _case("Question one")
    second = _case("Question two")
    assert first.source == second.source
    assert first.task != second.task
    assert first.fingerprint != second.fingerprint


def test_attempt_request_keeps_runtime_out_of_case_identity() -> None:
    case = _case()
    request = AttemptRequest(
        case=case,
        agent=AgentCondition(
            agent_id="pi",
            adapter="pi-node",
            model="gpt-5.6-luna",
            thinking_level="medium",
        ),
        runtime=RuntimeBinding(runtime_id="local", parameters={"port": 10090}),
    )
    assert request.case.fingerprint == case.fingerprint


def test_prediction_status_is_strict() -> None:
    common = {
        "case_id": "case",
        "attempt_id": "attempt",
        "agent_session_id": "pi",
        "interaction_session_id": "code-policy",
        "parser_revision": "v1",
        "final_text": "ANSWER: 4",
    }
    Prediction(**common, status="parsed", integer_answer=4)
    Prediction(**common, status="invalid", diagnostic="missing marker")
    with pytest.raises(ValidationError):
        Prediction(**common, status="parsed", diagnostic="bad")


@pytest.mark.parametrize(
    ("attempt_status", "task_result", "valid"),
    [
        ("completed", "passed", True),
        ("completed", "failed", True),
        ("completed", "not_evaluated", False),
        ("failed", "not_evaluated", True),
        ("failed", "failed", False),
    ],
)
def test_outcome_separates_operation_from_task(
    attempt_status: str, task_result: str, valid: bool
) -> None:
    values = {
        "attempt_id": "attempt",
        "attempt_status": attempt_status,
        "task_result": task_result,
        "reason": "test",
    }
    if valid:
        EvalOutcome.model_validate(values)
    else:
        with pytest.raises(ValidationError):
            EvalOutcome.model_validate(values)
