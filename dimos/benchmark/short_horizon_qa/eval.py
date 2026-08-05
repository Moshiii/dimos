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

"""Frozen Memory2 QA drivers for the canonical agent-evaluation engine."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import re
from typing import Any, cast

from pydantic import Field, JsonValue

from dimos.agents.code_policy_core import (
    CodePolicySessionConfig,
    FrozenMemoryEnvironment,
)
from dimos.agents.code_policy_server import StandaloneCodePolicyProcess
from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.benchmark.agent_eval.base import BaseEvalModel
from dimos.benchmark.agent_eval.case import (
    AgentCondition,
    AgentOutcome,
    AttemptRequest,
    EvalCase,
    ExactIntegerValidatorRef,
    FrozenCodePolicyInteraction,
    FrozenRecordingSource,
    IntegerQuestionTask,
    Prediction,
    PrivateScore,
    RuntimeBinding,
    SourceSpec,
)
from dimos.benchmark.agent_eval.engine import AttemptEngine, EngineResult
from dimos.benchmark.agent_eval.interfaces import (
    AgentAdapter,
    AttemptContext,
    EvidenceSink,
    PreparedSource,
    ValidatorSession,
)
from dimos.benchmark.agent_eval.pi import PiSession, PiSessionFactory
from dimos.benchmark.agent_eval.pi_adapter import (
    CodePolicyCallLog,
    wait_for_python_exec,
)
from dimos.benchmark.short_horizon_qa.service import load_bundle

_ANSWER_LINE = re.compile(r"(?m)^ANSWER:\s*")
_TERMINAL_INTEGER = re.compile(r"(?:^|\n)ANSWER:\s*(-?\d+)\s*\Z")


class RoomOracleEntry(BaseEvalModel):
    label: str = Field(min_length=1)
    evidence: tuple[str, ...] = Field(min_length=1)


class ExactIntegerOracle(BaseEvalModel):
    expected_count: int = Field(ge=0)
    counting_policy: str = Field(min_length=1)
    rooms: tuple[RoomOracleEntry, ...]
    reviewed_by: tuple[str, ...] = Field(min_length=1)


class FrozenMemorySourceDriver:
    def __init__(self, bundle: Path) -> None:
        self.bundle = bundle

    def prepare(
        self,
        *,
        source: SourceSpec,
        context: AttemptContext,
        evidence: EvidenceSink,
    ) -> PreparedSource:
        del context
        if not isinstance(source, FrozenRecordingSource):
            raise TypeError("frozen source driver requires FrozenRecordingSource")
        manifest, cutoff, source_path, derived_path = load_bundle(
            self.bundle, progress=source.progress
        )
        if source_path.stem != source.recording:
            raise ValueError("prepared recording does not match the authored source")
        if (
            source.bundle_manifest_sha256 is not None
            and source.bundle_manifest_sha256
            != hashlib.sha256((self.bundle / "manifest.v1.json").read_bytes()).hexdigest()
        ):
            raise ValueError("prepared manifest digest does not match the case")
        evidence.artifact("source-manifest.v1.json", manifest)
        receipt = {
            "recording": source.recording,
            "progress": source.progress,
            "cutoff_seconds": cutoff.cutoff_seconds,
            "cutoff_timestamp": cutoff.cutoff_timestamp,
            "source_sha256": manifest.source_sha256,
            "derived_sha256": manifest.derived_sha256,
        }
        return PreparedSource(
            public={"recording": source.recording, "progress": source.progress},
            receipt=receipt,
            private_handle={
                "source_path": str(source_path),
                "derived_path": str(derived_path),
                "cutoff_timestamp": cutoff.cutoff_timestamp,
            },
        )

    def close(self) -> None:
        return None


@dataclass(frozen=True)
class CodePolicyAgentInterface:
    mcp: McpAdapter
    session_id: str
    call_log: CodePolicyCallLog
    evidence: EvidenceSink


class PiCodePolicyAgent(AgentAdapter):
    def __init__(self, factory: PiSessionFactory, *, turn_timeout_s: float = 180.0) -> None:
        self.factory = factory
        self.turn_timeout_s = turn_timeout_s
        self._session: PiSession | None = None
        self._interface: CodePolicyAgentInterface | None = None

    def run(
        self,
        *,
        task: Any,
        context: AttemptContext,
        interface: Any = None,
    ) -> AgentOutcome:
        if not isinstance(task, IntegerQuestionTask):
            raise TypeError("frozen Pi agent requires an integer question task")
        if not isinstance(interface, CodePolicyAgentInterface):
            raise TypeError("frozen Pi agent requires a CodePolicy interface")
        prompt = (
            f"{task.prompt}\n\n"
            "Use the provided python_exec tool and the read-only `memory` API to "
            "answer from the recording. End your final response with exactly "
            f"`{task.answer_marker} <integer>`."
        )
        session = self.factory.create(
            attempt_path=context.path,
            public_prompt=prompt,
            code_policy_session_id=interface.session_id,
            call_log=interface.call_log,
            mcp=interface.mcp,
        )
        self._session = session
        self._interface = interface
        turn = session.prompt(prompt, self.turn_timeout_s)
        return AgentOutcome(
            final_text=turn.final_text,
            tool_call_count=turn.policy_call_count,
            terminal_reason="pi turn completed",
            agent_session_id=session.session_id,
            interaction_session_id=interface.session_id,
        )

    def close(self) -> None:
        session = self._session
        interface = self._interface
        self._session = None
        self._interface = None
        if session is None:
            return
        session.dispose()
        if interface is not None:
            for artifact in session.artifact_references():
                interface.evidence.reference(artifact.path)


class FrozenCodePolicyInteractionDriver:
    def __init__(self, *, readiness_timeout_s: float = 10.0) -> None:
        self.readiness_timeout_s = readiness_timeout_s
        self._process: StandaloneCodePolicyProcess | None = None
        self._call_log: CodePolicyCallLog | None = None
        self._evidence: EvidenceSink | None = None
        self._session_id: str | None = None

    def run(
        self,
        *,
        case: EvalCase,
        prepared_source: PreparedSource,
        agent: AgentAdapter,
        context: AttemptContext,
        evidence: EvidenceSink,
    ) -> AgentOutcome:
        if not isinstance(case.interaction, FrozenCodePolicyInteraction):
            raise TypeError("frozen interaction driver received an incompatible case")
        handle = prepared_source.private_handle
        if not isinstance(handle, dict):
            raise TypeError("frozen source did not provide a private binding")
        config = CodePolicySessionConfig(
            environment=FrozenMemoryEnvironment(
                recording_path=str(handle["source_path"]),
                derived_recording_path=str(handle["derived_path"]),
                memory_cutoff_timestamp=float(handle["cutoff_timestamp"]),
            )
        )
        process = StandaloneCodePolicyProcess(config)
        process.start(self.readiness_timeout_s)
        self._process = process
        self._evidence = evidence
        adapter = McpAdapter(process.mcp_url, timeout=120)
        inventory = wait_for_python_exec(process.mcp_url, adapter, self.readiness_timeout_s)
        evidence.artifact("mcp-inventory.v1.json", inventory)
        receipt = process.receipt()
        session_id = str(receipt["session_id"])
        self._session_id = session_id
        evidence.artifact("code-policy-session.v1.json", receipt)
        call_log = CodePolicyCallLog(context.path / "code-policy-calls.jsonl")
        self._call_log = call_log
        return agent.run(
            task=case.task,
            context=context,
            interface=CodePolicyAgentInterface(
                mcp=adapter,
                session_id=session_id,
                call_log=call_log,
                evidence=evidence,
            ),
        )

    def close(self) -> None:
        process = self._process
        call_log = self._call_log
        evidence = self._evidence
        session_id = self._session_id
        self._process = None
        self._call_log = None
        self._evidence = None
        self._session_id = None
        if call_log is not None:
            call_log.close()
            if evidence is not None:
                evidence.reference("code-policy-calls.jsonl")
        if process is not None:
            try:
                if evidence is not None:
                    evidence.artifact(
                        "code-policy-records.v1.json",
                        cast("JsonValue", process.records(session_id)),
                    )
            finally:
                process.close()


class ExactIntegerValidatorDriver:
    def __init__(self, private_root: Path) -> None:
        self.private_root = private_root.resolve()

    def prepare(
        self,
        *,
        case: EvalCase,
        prepared_source: PreparedSource,
        context: AttemptContext,
        evidence: EvidenceSink,
    ) -> ValidatorSession:
        del prepared_source
        oracle = load_exact_integer_oracle(case, self.private_root)
        reference = cast("ExactIntegerValidatorRef", case.validator)
        evidence.artifact("oracle.private.v1.json", oracle)
        return _ExactIntegerValidatorSession(
            case=case,
            context=context,
            evidence=evidence,
            oracle=oracle,
            revision=reference.revision,
        )


class _ExactIntegerValidatorSession:
    def __init__(
        self,
        *,
        case: EvalCase,
        context: AttemptContext,
        evidence: EvidenceSink,
        oracle: ExactIntegerOracle,
        revision: str,
    ) -> None:
        self.case = case
        self.context = context
        self.evidence = evidence
        self.oracle = oracle
        self.revision = revision

    def evaluate(self, outcome: AgentOutcome) -> PrivateScore:
        if outcome.agent_session_id is None or outcome.interaction_session_id is None:
            raise ValueError("agent outcome is missing session identities")
        prediction = parse_integer_prediction(
            case_id=self.case.case_id,
            attempt_id=self.context.attempt_id,
            agent_session_id=outcome.agent_session_id,
            interaction_session_id=outcome.interaction_session_id,
            final_text=outcome.final_text,
        )
        self.evidence.artifact("prediction.v1.json", prediction)
        passed = (
            prediction.status == "parsed"
            and prediction.integer_answer == self.oracle.expected_count
        )
        return PrivateScore(
            case_id=self.case.case_id,
            attempt_id=self.context.attempt_id,
            validator_revision=self.revision,
            passed=passed,
            prediction_status=prediction.status,
        )

    def close(self) -> None:
        return None


def load_exact_integer_oracle(case: EvalCase, private_root: Path) -> ExactIntegerOracle:
    """Resolve and verify a case-relative private oracle before agent dispatch."""
    reference = case.validator
    if not isinstance(reference, ExactIntegerValidatorRef):
        raise TypeError("exact integer validator requires ExactIntegerValidatorRef")
    root = private_root.resolve()
    path = (root / reference.private_path).resolve()
    if root not in path.parents:
        raise ValueError("private oracle path escapes its case directory")
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != reference.private_sha256:
        raise ValueError("private oracle digest does not match the case")
    return ExactIntegerOracle.model_validate_json(data)


def parse_integer_prediction(
    *,
    case_id: str,
    attempt_id: str,
    agent_session_id: str,
    interaction_session_id: str,
    final_text: str,
) -> Prediction:
    markers = _ANSWER_LINE.findall(final_text)
    match = _TERMINAL_INTEGER.search(final_text)
    if len(markers) != 1 or match is None:
        return Prediction(
            case_id=case_id,
            attempt_id=attempt_id,
            agent_session_id=agent_session_id,
            interaction_session_id=interaction_session_id,
            parser_revision="marked-integer-v1",
            final_text=final_text,
            status="invalid",
            diagnostic="expected exactly one terminal ANSWER: <integer> marker",
        )
    return Prediction(
        case_id=case_id,
        attempt_id=attempt_id,
        agent_session_id=agent_session_id,
        interaction_session_id=interaction_session_id,
        parser_revision="marked-integer-v1",
        final_text=final_text,
        status="parsed",
        integer_answer=int(match.group(1)),
    )


def run_frozen_case(
    *,
    case: EvalCase,
    bundle: Path,
    private_root: Path,
    output_root: Path,
    pi_factory: PiSessionFactory,
    agent_condition: AgentCondition | None = None,
    runtime_binding: RuntimeBinding | None = None,
    turn_timeout_s: float = 180.0,
) -> EngineResult:
    request = AttemptRequest(
        case=case,
        agent=agent_condition
        or AgentCondition(
            agent_id="pi-code-policy",
            adapter="pi-node",
            model="gpt-5.6-luna",
            thinking_level="medium",
        ),
        runtime=runtime_binding or RuntimeBinding(runtime_id="local-standalone-code-policy"),
    )
    return AttemptEngine(
        request=request,
        output_root=output_root,
        source=FrozenMemorySourceDriver(bundle),
        interaction=FrozenCodePolicyInteractionDriver(),
        validator=ExactIntegerValidatorDriver(private_root),
        agent=PiCodePolicyAgent(pi_factory, turn_timeout_s=turn_timeout_s),
    ).run()
