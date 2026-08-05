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

import hashlib
import json

import pytest

from dimos.benchmark.agent_eval.case import (
    EvalCase,
    ExactIntegerValidatorRef,
    FrozenCodePolicyInteraction,
    FrozenRecordingSource,
    IntegerQuestionTask,
)
import dimos.benchmark.agent_eval.single_case as single_case
from dimos.benchmark.agent_eval.single_case import (
    EvalRunConfig,
    OpenAIApiKeyConfig,
    _resolve_credential,
    execute_single_case,
)
from dimos.benchmark.short_horizon_qa.eval import load_exact_integer_oracle


def test_run_config_round_trips_through_pydantic() -> None:
    configured = EvalRunConfig()

    decoded = EvalRunConfig.model_validate_json(configured.model_dump_json())

    assert decoded == configured
    assert decoded.agent.backend == "pi"
    assert decoded.agent.model == "gpt-5.6-luna"
    assert decoded.agent.auth.mode == "codex-oauth"


def test_api_key_auth_uses_named_environment_without_serializing_secret(monkeypatch) -> None:
    monkeypatch.setenv("EVAL_TEST_KEY", "private-value")
    auth = OpenAIApiKeyConfig(env="EVAL_TEST_KEY")

    credential, digest = _resolve_credential(auth)

    assert credential.value == "private-value"
    assert len(digest) == 64
    assert "private-value" not in auth.model_dump_json()
    assert "private-value" not in digest


def test_private_validator_resolves_relative_to_case_directory(tmp_path) -> None:
    private = tmp_path / "private"
    private.mkdir()
    oracle = private / "oracle.json"
    oracle.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "expected_count": 4,
                "counting_policy": "Count enclosed rooms.",
                "rooms": [],
                "reviewed_by": ["reviewer"],
            }
        )
    )
    case = EvalCase.compile(
        case_id="case",
        source=FrozenRecordingSource(recording="recording", progress=1.0),
        task=IntegerQuestionTask(prompt="How many rooms?"),
        interaction=FrozenCodePolicyInteraction(driver_revision="v1"),
        validator=ExactIntegerValidatorRef(
            revision="v1",
            private_path="private/oracle.json",
            private_sha256=hashlib.sha256(oracle.read_bytes()).hexdigest(),
        ),
    )

    loaded = load_exact_integer_oracle(case, tmp_path)

    assert loaded.expected_count == 4


def test_single_case_emits_public_question_before_private_preflight(tmp_path, monkeypatch) -> None:
    case = EvalCase.compile(
        case_id="case",
        source=FrozenRecordingSource(recording="recording", progress=1.0),
        task=IntegerQuestionTask(prompt="How many rooms?"),
        interaction=FrozenCodePolicyInteraction(driver_revision="v1"),
        validator=ExactIntegerValidatorRef(
            revision="v1",
            private_path="private/oracle.json",
            private_sha256="0" * 64,
        ),
    )
    case_path = tmp_path / "case.json"
    case_path.write_text(case.model_dump_json())
    events = []

    def stop_at_private_preflight(*args, **kwargs):
        raise RuntimeError("stop after public header")

    monkeypatch.setattr(single_case, "load_exact_integer_oracle", stop_at_private_preflight)

    with pytest.raises(RuntimeError, match="stop after public header"):
        execute_single_case(case_path, config=EvalRunConfig(), progress=events.append)

    assert [event.kind for event in events] == ["status", "case_header", "status"]
    header = events[1]
    assert header.model_dump() == {
        "kind": "case_header",
        "case_id": "case",
        "source": "recording",
        "progress": 1.0,
        "question": "How many rooms?",
    }
