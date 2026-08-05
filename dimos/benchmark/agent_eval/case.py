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

"""Canonical source/task/interaction/validator contracts for agent evaluation."""

from __future__ import annotations

import hashlib
import math
from pathlib import PurePosixPath
from typing import Annotated, Literal

from pydantic import Field, JsonValue, model_validator

from dimos.benchmark.agent_eval.base import BaseEvalModel
from dimos.benchmark.spatial.utilities import canonical_json

NonEmpty = Annotated[str, Field(min_length=1)]
Sha256 = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
NormalizedProgress = Annotated[float, Field(ge=0.0, le=1.0, allow_inf_nan=False)]


class FrozenRecordingSource(BaseEvalModel):
    kind: Literal["frozen_memory"] = "frozen_memory"
    recording: NonEmpty
    progress: NormalizedProgress
    bundle_manifest_sha256: Sha256 | None = None

    @model_validator(mode="after")
    def progress_is_finite(self) -> FrozenRecordingSource:
        if not math.isfinite(self.progress):
            raise ValueError("recording progress must be finite")
        return self


class LiveDimosSource(BaseEvalModel):
    kind: Literal["live_dimos"] = "live_dimos"
    runtime: NonEmpty
    scene: NonEmpty | None = None
    episode: NonEmpty | None = None


SourceSpec = Annotated[
    FrozenRecordingSource | LiveDimosSource,
    Field(discriminator="kind"),
]


class IntegerQuestionTask(BaseEvalModel):
    kind: Literal["integer_question"] = "integer_question"
    prompt: NonEmpty
    answer_marker: Literal["ANSWER:"] = "ANSWER:"


class EmbodiedInstructionTask(BaseEvalModel):
    kind: Literal["embodied_instruction"] = "embodied_instruction"
    prompt: NonEmpty


TaskSpec = Annotated[
    IntegerQuestionTask | EmbodiedInstructionTask,
    Field(discriminator="kind"),
]


class FrozenCodePolicyInteraction(BaseEvalModel):
    kind: Literal["frozen_code_policy"] = "frozen_code_policy"
    driver_revision: NonEmpty
    session_lifetime: Literal["one_attempt"] = "one_attempt"


class LiveCodePolicyInteraction(BaseEvalModel):
    kind: Literal["live_code_policy"] = "live_code_policy"
    driver_revision: NonEmpty
    session_lifetime: Literal["one_attempt"] = "one_attempt"
    completion: Literal["agent_or_native_terminal"] = "agent_or_native_terminal"


InteractionSpec = Annotated[
    FrozenCodePolicyInteraction | LiveCodePolicyInteraction,
    Field(discriminator="kind"),
]


class ExactIntegerValidatorRef(BaseEvalModel):
    kind: Literal["exact_integer"] = "exact_integer"
    revision: NonEmpty
    private_path: NonEmpty
    private_sha256: Sha256

    @model_validator(mode="after")
    def private_path_is_relative(self) -> ExactIntegerValidatorRef:
        path = PurePosixPath(self.private_path)
        if path.is_absolute() or not path.parts or ".." in path.parts:
            raise ValueError("validator private_path must be a safe relative path")
        return self


class NativeValidatorRef(BaseEvalModel):
    kind: Literal["native"] = "native"
    revision: NonEmpty
    contract_sha256: Sha256


ValidatorRef = Annotated[
    ExactIntegerValidatorRef | NativeValidatorRef,
    Field(discriminator="kind"),
]


class PublicEvalCase(BaseEvalModel):
    """Agent-safe case projection; it intentionally cannot carry a validator."""

    case_id: NonEmpty
    source: SourceSpec
    task: TaskSpec
    interaction: InteractionSpec


class EvalCase(BaseEvalModel):
    """Compiled private case binding all four semantic contracts."""

    case_id: NonEmpty
    source: SourceSpec
    task: TaskSpec
    interaction: InteractionSpec
    validator: ValidatorRef
    fingerprint: Sha256

    @classmethod
    def compile(
        cls,
        *,
        case_id: str,
        source: SourceSpec,
        task: TaskSpec,
        interaction: InteractionSpec,
        validator: ValidatorRef,
    ) -> EvalCase:
        payload = _case_payload(case_id, source, task, interaction, validator)
        return cls(
            case_id=case_id,
            source=source,
            task=task,
            interaction=interaction,
            validator=validator,
            fingerprint=hashlib.sha256(canonical_json(payload)).hexdigest(),
        )

    @model_validator(mode="after")
    def fingerprint_matches_payload(self) -> EvalCase:
        payload = _case_payload(
            self.case_id,
            self.source,
            self.task,
            self.interaction,
            self.validator,
        )
        expected = hashlib.sha256(canonical_json(payload)).hexdigest()
        if self.fingerprint != expected:
            raise ValueError("evaluation case fingerprint does not match its contracts")
        return self

    def public_projection(self) -> PublicEvalCase:
        return PublicEvalCase(
            case_id=self.case_id,
            source=self.source,
            task=self.task,
            interaction=self.interaction,
        )


class AgentCondition(BaseEvalModel):
    agent_id: NonEmpty
    adapter: NonEmpty
    model: NonEmpty
    thinking_level: NonEmpty


class RuntimeBinding(BaseEvalModel):
    runtime_id: NonEmpty
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class AttemptRequest(BaseEvalModel):
    case: EvalCase
    agent: AgentCondition
    runtime: RuntimeBinding
    seed: int | None = None


class AgentOutcome(BaseEvalModel):
    final_text: str
    tool_call_count: int = Field(ge=0)
    terminal_reason: NonEmpty
    agent_session_id: NonEmpty | None = None
    interaction_session_id: NonEmpty | None = None


class Prediction(BaseEvalModel):
    case_id: NonEmpty
    attempt_id: NonEmpty
    agent_session_id: NonEmpty
    interaction_session_id: NonEmpty
    parser_revision: NonEmpty
    final_text: str
    status: Literal["parsed", "invalid"]
    integer_answer: int | None = None
    diagnostic: NonEmpty | None = None

    @model_validator(mode="after")
    def answer_matches_status(self) -> Prediction:
        if self.status == "parsed" and (self.integer_answer is None or self.diagnostic is not None):
            raise ValueError("parsed prediction requires only integer_answer")
        if self.status == "invalid" and (
            self.integer_answer is not None or self.diagnostic is None
        ):
            raise ValueError("invalid prediction requires only diagnostic")
        return self


class PrivateScore(BaseEvalModel):
    case_id: NonEmpty
    attempt_id: NonEmpty
    validator_revision: NonEmpty
    passed: bool
    prediction_status: Literal["parsed", "invalid", "native"]


class EvalOutcome(BaseEvalModel):
    attempt_id: NonEmpty
    attempt_status: Literal["completed", "failed"]
    task_result: Literal["passed", "failed", "not_evaluated"]
    reason: NonEmpty

    @model_validator(mode="after")
    def states_are_consistent(self) -> EvalOutcome:
        if self.attempt_status == "failed" and self.task_result != "not_evaluated":
            raise ValueError("failed infrastructure cannot claim a task result")
        if self.attempt_status == "completed" and self.task_result == "not_evaluated":
            raise ValueError("completed evaluation must report passed or failed")
        return self


def _case_payload(
    case_id: str,
    source: SourceSpec,
    task: TaskSpec,
    interaction: InteractionSpec,
    validator: ValidatorRef,
) -> dict[str, JsonValue]:
    return {
        "case_id": case_id,
        "source": source.model_dump(mode="json"),
        "task": task.model_dump(mode="json"),
        "interaction": interaction.model_dump(mode="json"),
        "validator": validator.model_dump(mode="json"),
    }
