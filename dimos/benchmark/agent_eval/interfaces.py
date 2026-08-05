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

"""Backend-neutral interfaces used by canonical agent-evaluation attempts."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Protocol

from pydantic import BaseModel, ConfigDict, JsonValue

from dimos.benchmark.agent_eval.case import (
    AgentOutcome,
    AttemptRequest,
    EvalCase,
    PrivateScore,
    SourceSpec,
    TaskSpec,
)


class InterfaceModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class AttemptContext(InterfaceModel):
    attempt_id: str
    path: Path
    request: AttemptRequest


class PreparedSource(InterfaceModel):
    public: dict[str, JsonValue]
    receipt: dict[str, JsonValue]
    private_handle: Any = None


class AgentAdapter(Protocol):
    def run(
        self, *, task: TaskSpec, context: AttemptContext, interface: Any = None
    ) -> AgentOutcome: ...

    def close(self) -> None: ...


class EvidenceSink(Protocol):
    def event(self, kind: str, payload: dict[str, JsonValue] | None = None) -> None: ...

    def artifact(self, relative_path: str, value: BaseModel | JsonValue | bytes | str) -> None: ...

    def reference(self, relative_path: str) -> None: ...


class SourceDriver(Protocol):
    def prepare(
        self,
        *,
        source: SourceSpec,
        context: AttemptContext,
        evidence: EvidenceSink,
    ) -> PreparedSource: ...

    def close(self) -> None: ...


class InteractionDriver(Protocol):
    def run(
        self,
        *,
        case: EvalCase,
        prepared_source: PreparedSource,
        agent: AgentAdapter,
        context: AttemptContext,
        evidence: EvidenceSink,
    ) -> AgentOutcome: ...

    def close(self) -> None: ...


class ValidatorSession(Protocol):
    def evaluate(self, outcome: AgentOutcome) -> PrivateScore: ...

    def close(self) -> None: ...


class ValidatorDriver(Protocol):
    def prepare(
        self,
        *,
        case: EvalCase,
        prepared_source: PreparedSource,
        context: AttemptContext,
        evidence: EvidenceSink,
    ) -> ValidatorSession: ...
