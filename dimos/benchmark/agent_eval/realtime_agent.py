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

"""External Pi + standalone live CodePolicy worker for real-time attempts."""

from __future__ import annotations

from pathlib import Path
import threading
from typing import cast

from pydantic import JsonValue

from dimos.agents.code_policy_core import CodePolicySessionConfig, LiveDimosEnvironment
from dimos.agents.code_policy_server import StandaloneCodePolicyProcess
from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.benchmark.agent_eval.case import AgentOutcome
from dimos.benchmark.agent_eval.engine import AttemptEvidence
from dimos.benchmark.agent_eval.pi import PiSession, PiSessionFactory, PiTurn
from dimos.benchmark.agent_eval.pi_adapter import CodePolicyCallLog, wait_for_python_exec
from dimos.core.global_config import global_config

NEUTRAL_CONTINUATION = "Continue working on the task."
MAX_TURNS_SAFETY_CEILING = 100


class ExternalPiWorkerFactory:
    def __init__(
        self,
        *,
        pi_factory: PiSessionFactory,
        turn_timeout_seconds: float,
        readiness_timeout_seconds: float = 180.0,
        cancellation_timeout_seconds: float = 5.0,
    ) -> None:
        self.pi_factory = pi_factory
        self.turn_timeout_seconds = turn_timeout_seconds
        self.readiness_timeout_seconds = readiness_timeout_seconds
        self.cancellation_timeout_seconds = cancellation_timeout_seconds

    def create(self) -> ExternalPiWorker:
        return ExternalPiWorker(
            pi_factory=self.pi_factory,
            turn_timeout_seconds=self.turn_timeout_seconds,
            readiness_timeout_seconds=self.readiness_timeout_seconds,
            cancellation_timeout_seconds=self.cancellation_timeout_seconds,
        )


class ExternalPiWorker:
    def __init__(
        self,
        *,
        pi_factory: PiSessionFactory,
        turn_timeout_seconds: float,
        readiness_timeout_seconds: float,
        cancellation_timeout_seconds: float,
    ) -> None:
        self.pi_factory = pi_factory
        self.turn_timeout_seconds = turn_timeout_seconds
        self.readiness_timeout_seconds = readiness_timeout_seconds
        self.cancellation_timeout_seconds = cancellation_timeout_seconds
        self._process: StandaloneCodePolicyProcess | None = None
        self._session: PiSession | None = None
        self._call_log: CodePolicyCallLog | None = None
        self._evidence: AttemptEvidence | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._failure: BaseException | None = None
        self._last_turn = PiTurn(policy_call_count=0)
        self._code_policy_session_id: str | None = None

    def start(
        self,
        *,
        prompt: str,
        memory_path: Path,
        attempt_path: Path,
        evidence: AttemptEvidence,
    ) -> None:
        process = StandaloneCodePolicyProcess(
            CodePolicySessionConfig(
                environment=LiveDimosEnvironment(recording_path=str(memory_path))
            ),
            environment={"DIMOS_TRANSPORT": global_config.transport},
        )
        process.start(self.readiness_timeout_seconds)
        mcp = McpAdapter(process.mcp_url, timeout=int(self.readiness_timeout_seconds))
        evidence.artifact(
            "mcp-inventory.v1.json",
            wait_for_python_exec(
                process.mcp_url,
                mcp,
                self.readiness_timeout_seconds,
            ),
        )
        receipt = process.receipt()
        evidence.artifact("code-policy-session.v1.json", receipt)
        code_policy_session_id = str(receipt["session_id"])
        call_log = CodePolicyCallLog(attempt_path / "code-policy-calls.jsonl")
        session = self.pi_factory.create(
            attempt_path=attempt_path,
            public_prompt=prompt,
            code_policy_session_id=code_policy_session_id,
            call_log=call_log,
            mcp=mcp,
        )
        self._process = process
        self._session = session
        self._call_log = call_log
        self._evidence = evidence
        self._code_policy_session_id = code_policy_session_id
        self._thread = threading.Thread(
            target=self._run,
            args=(prompt,),
            name=f"realtime-pi-{session.session_id}",
            daemon=True,
        )
        self._thread.start()

    def _run(self, prompt: str) -> None:
        assert self._session is not None
        try:
            for turn in range(MAX_TURNS_SAFETY_CEILING):
                if self._stop.is_set():
                    return
                previous_policy_calls = self._last_turn.policy_call_count
                self._last_turn = self._session.prompt(prompt, self.turn_timeout_seconds)
                policy_calls = max(0, self._last_turn.policy_call_count - previous_policy_calls)
                if self._evidence is not None:
                    self._evidence.event(
                        "pi-turn-ended",
                        {
                            "turn": turn + 1,
                            "policy_calls": policy_calls,
                        },
                    )
                if policy_calls == 0:
                    if self._evidence is not None:
                        self._evidence.event("pi-idle")
                    self._stop.wait(self.turn_timeout_seconds)
                    return
                prompt = NEUTRAL_CONTINUATION
                if self._evidence is not None:
                    self._evidence.event("pi-continuation")
            raise RuntimeError("Pi exceeded the live turn safety ceiling")
        except BaseException as exc:
            if not self._stop.is_set():
                self._failure = exc

    def failure(self) -> BaseException | None:
        return self._failure

    def outcome(self, terminal_reason: str) -> AgentOutcome:
        return AgentOutcome(
            final_text=self._last_turn.final_text,
            tool_call_count=self._last_turn.policy_call_count,
            terminal_reason=terminal_reason,
            agent_session_id=(self._session.session_id if self._session is not None else None),
            interaction_session_id=self._code_policy_session_id,
        )

    def abort(self) -> None:
        self._stop.set()
        if self._session is not None:
            self._session.abort(self.cancellation_timeout_seconds)
        if self._process is not None:
            self._process.interrupt()
        if self._thread is not None:
            self._thread.join(timeout=self.cancellation_timeout_seconds)

    def close(self) -> None:
        if self._session is not None:
            self._session.dispose()
            if self._evidence is not None:
                for artifact in self._session.artifact_references():
                    self._evidence.reference(artifact.path)
        if self._call_log is not None:
            self._call_log.close()
            if self._evidence is not None:
                self._evidence.reference("code-policy-calls.jsonl")
        if self._process is not None:
            if self._evidence is not None:
                self._evidence.artifact(
                    "code-policy-records.v1.json",
                    cast("JsonValue", self._process.records()),
                )
            self._process.close()
