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

"""Backend-neutral Pi code-policy session contracts."""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

from pydantic import BaseModel, ConfigDict, Field

from dimos.benchmark.agent_eval.artifacts import ArtifactReference
from dimos.benchmark.agent_eval.pi_adapter import CodePolicyCallLog, McpBinding


class PiTurn(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)

    final_text: str = ""
    policy_call_count: int = Field(ge=0)


class PiSession(Protocol):
    session_id: str

    def prompt(self, prompt: str, timeout_s: float) -> PiTurn: ...

    def abort(self, timeout_s: float) -> None: ...

    def dispose(self) -> None: ...

    def artifact_references(self) -> tuple[ArtifactReference, ...]: ...


class PiSessionFactory(Protocol):
    def create(
        self,
        *,
        attempt_path: Path,
        public_prompt: str,
        code_policy_session_id: str,
        call_log: CodePolicyCallLog,
        mcp: McpBinding,
    ) -> PiSession: ...
