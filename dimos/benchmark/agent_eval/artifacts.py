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

"""Generic records for immutable evaluation evidence."""

from __future__ import annotations

from datetime import datetime
from typing import Annotated, Literal

from pydantic import Field, JsonValue, model_validator

from dimos.benchmark.agent_eval.base import BaseEvalModel

NonEmpty = Annotated[str, Field(min_length=1)]
Sha256 = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
AttemptId = Annotated[str, Field(pattern=r"^attempt_[0-9a-f]{32}$")]
OperationId = Annotated[str, Field(pattern=r"^operation_[0-9a-f]{32}$")]
CodePolicySessionId = Annotated[str, Field(pattern=r"^code_policy_session_[0-9a-f]{32}$")]


class ArtifactReference(BaseEvalModel):
    record_type: Literal["artifact-reference"] = "artifact-reference"
    path: NonEmpty
    sha256: Sha256
    size_bytes: Annotated[int, Field(ge=0)]

    @model_validator(mode="after")
    def path_is_relative(self) -> ArtifactReference:
        if self.path.startswith("/") or ".." in self.path.split("/"):
            raise ValueError("artifact path must be attempt-relative")
        return self


class LifecycleEvent(BaseEvalModel):
    record_type: Literal["agent-eval-lifecycle-event"] = "agent-eval-lifecycle-event"
    sequence: Annotated[int, Field(ge=1)]
    attempt_id: AttemptId
    operation_id: OperationId | None = None
    occurred_at: datetime
    monotonic_offset_s: Annotated[float, Field(ge=0)]
    kind: NonEmpty
    payload: dict[str, JsonValue] = Field(default_factory=dict)


class NormalizedOutcome(BaseEvalModel):
    """Generic terminal record retained for attempt-store callers."""

    record_type: Literal["agent-eval-outcome"] = "agent-eval-outcome"
    attempt_id: AttemptId
    attempt_status: Literal["completed", "failed"]
    task_result: Literal["passed", "failed", "not_evaluated"]
    terminal_stage: NonEmpty
    reason: NonEmpty
    required_artifacts_complete: bool
    finished_at: datetime
    duration_s: Annotated[float, Field(ge=0)]

    @model_validator(mode="after")
    def infrastructure_and_task_states_are_consistent(self) -> NormalizedOutcome:
        if self.attempt_status == "failed" and self.task_result != "not_evaluated":
            raise ValueError("failed infrastructure cannot report a task result")
        if self.attempt_status == "completed" and self.task_result == "not_evaluated":
            raise ValueError("completed evaluation must report pass or fail")
        if self.attempt_status == "completed" and not self.required_artifacts_complete:
            raise ValueError("completed evaluation requires complete artifacts")
        return self
