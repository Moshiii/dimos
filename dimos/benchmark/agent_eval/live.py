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

"""Canonical live DimSim source, interaction, and native-validator drivers."""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import FIRST_COMPLETED, Future, ThreadPoolExecutor, wait
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any, Protocol, cast

from pydantic import BaseModel, JsonValue

from dimos.agents.code_policy_core import CodePolicySessionConfig, LiveDimosEnvironment
from dimos.agents.code_policy_server import StandaloneCodePolicyProcess
from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.benchmark.agent_eval.backend import (
    BackendEvaluationRequest,
    BackendResetRequest,
    SimulatorBackend,
)
from dimos.benchmark.agent_eval.case import (
    AgentCondition,
    AgentOutcome,
    AttemptRequest,
    EmbodiedInstructionTask,
    EvalCase,
    LiveCodePolicyInteraction,
    LiveDimosSource,
    NativeValidatorRef,
    PrivateScore,
    RuntimeBinding,
    SourceSpec,
)
from dimos.benchmark.agent_eval.config import SelectedDestination
from dimos.benchmark.agent_eval.engine import AttemptEngine, EngineResult
from dimos.benchmark.agent_eval.interfaces import (
    AgentAdapter,
    AttemptContext,
    EvidenceSink,
    PreparedSource,
    ValidatorSession,
)
from dimos.benchmark.agent_eval.models import (
    BackendEpisodeReference,
    EvaluationHandle,
    InfrastructureTimeouts,
)
from dimos.benchmark.agent_eval.pi import PiSession, PiSessionFactory
from dimos.benchmark.agent_eval.pi_adapter import (
    CodePolicyCallLog,
    McpBinding,
    wait_for_python_exec,
)
from dimos.benchmark.agent_eval.store import new_operation_id

NEUTRAL_CONTINUATION = "Continue working on the task."
MAX_TURNS_SAFETY_CEILING = 100


class MotionControl(Protocol):
    def motion_active(self, timeout_s: float) -> bool: ...

    def cancel_motion(self, timeout_s: float) -> None: ...

    def close(self) -> None: ...


class LivePolicyProcess(Protocol):
    mcp_url: str

    def start(self, timeout_s: float = 10.0) -> None: ...

    def receipt(self) -> dict[str, Any]: ...

    def records(self, session_id: str | None = None) -> list[dict[str, Any]]: ...

    def close(self) -> None: ...


@dataclass
class LiveAttemptState:
    episode: BackendEpisodeReference
    handle: EvaluationHandle | Any | None = None
    native: Any | None = None
    terminal_reason: str = "not_started"


def compile_dimsim_case(
    selected: SelectedDestination,
    *,
    runtime: str = "unitree-go2-dimsim-external-pi-eval",
    timeout_seconds: float = 600.0,
) -> EvalCase:
    """Project one generated destination triple into the canonical case shape."""
    source = selected.contract.source
    return EvalCase.compile(
        case_id=selected.public.task_id,
        source=LiveDimosSource(
            runtime=runtime,
            scene=source.scene_id,
            episode=selected.manifest.release_id,
        ),
        task=EmbodiedInstructionTask(prompt=selected.public.text),
        interaction=LiveCodePolicyInteraction(
            driver_revision="dimsim-live-v1",
            timeout_seconds=timeout_seconds,
        ),
        validator=NativeValidatorRef(
            revision="dimsim-native-v1",
            contract_sha256=selected.contract_sha256,
        ),
    )


class LiveDimSimSourceDriver:
    def __init__(
        self,
        *,
        selected: SelectedDestination,
        backend: SimulatorBackend,
        timeouts: InfrastructureTimeouts,
        episode_timeout_s: float,
    ) -> None:
        self.selected = selected
        self.backend = backend
        self.timeouts = timeouts
        self.episode_timeout_s = episode_timeout_s

    def prepare(
        self,
        *,
        source: SourceSpec,
        context: AttemptContext,
        evidence: EvidenceSink,
    ) -> PreparedSource:
        if not isinstance(source, LiveDimosSource):
            raise TypeError("live DimSim driver requires LiveDimosSource")
        readiness = self.backend.readiness(self.timeouts.readiness_s)
        evidence.artifact("backend-readiness.v1.json", readiness)
        if not readiness.ready:
            raise RuntimeError(readiness.detail)
        episode = BackendEpisodeReference(
            backend=readiness.backend,
            episode_id=f"episode-{context.attempt_id}",
            opaque={"deadline_s": self.episode_timeout_s},
        )
        reset = self.backend.reset(
            BackendResetRequest(
                attempt_id=context.attempt_id,
                operation_id=new_operation_id(),
                task_id=self.selected.public.task_id,
                episode=episode,
                start_pose=self.selected.start_pose,
                source_revisions=_source_revisions(self.selected),
            ),
            self.timeouts.reset_s,
        )
        _validate_reset(self.selected, reset)
        evidence.artifact("source-reset.private.v1.json", reset)
        state = LiveAttemptState(episode=episode)
        return PreparedSource(
            public={
                "runtime": source.runtime,
                "scene": source.scene,
                "episode": source.episode,
            },
            receipt=reset.model_dump(mode="json"),
            private_handle=state,
        )

    def close(self) -> None:
        self.backend.cleanup()


class DimSimNativeValidatorDriver:
    def __init__(
        self,
        *,
        selected: SelectedDestination,
        backend: SimulatorBackend,
        timeouts: InfrastructureTimeouts,
    ) -> None:
        self.selected = selected
        self.backend = backend
        self.timeouts = timeouts

    def prepare(
        self,
        *,
        case: EvalCase,
        prepared_source: PreparedSource,
        context: AttemptContext,
        evidence: EvidenceSink,
    ) -> ValidatorSession:
        reference = case.validator
        state = prepared_source.private_handle
        if not isinstance(reference, NativeValidatorRef):
            raise TypeError("live DimSim validator requires NativeValidatorRef")
        if reference.contract_sha256 != self.selected.contract_sha256:
            raise ValueError("native validator contract digest mismatch")
        if not isinstance(state, LiveAttemptState):
            raise TypeError("live source did not provide attempt state")
        state.handle = self.backend.start_evaluation(
            BackendEvaluationRequest(
                attempt_id=context.attempt_id,
                operation_id=new_operation_id(),
                task_id=self.selected.public.task_id,
                episode=state.episode,
                contract_digest=self.selected.contract_sha256,
                contract_payload=self.selected.contract.contract.model_dump(mode="json"),
            ),
            self.timeouts.evaluation_start_s,
        )
        evidence.event("native-evaluation-started")
        return _DimSimNativeValidatorSession(
            case=case,
            context=context,
            state=state,
            evidence=evidence,
            revision=reference.revision,
            backend=self.backend,
            cancellation_s=self.timeouts.cancellation_s,
        )


class _DimSimNativeValidatorSession:
    def __init__(
        self,
        *,
        case: EvalCase,
        context: AttemptContext,
        state: LiveAttemptState,
        evidence: EvidenceSink,
        revision: str,
        backend: SimulatorBackend,
        cancellation_s: float,
    ) -> None:
        self.case = case
        self.context = context
        self.state = state
        self.evidence = evidence
        self.revision = revision
        self.backend = backend
        self.cancellation_s = cancellation_s

    def evaluate(self, outcome: AgentOutcome) -> PrivateScore:
        del outcome
        if self.state.native is None:
            raise RuntimeError("live interaction did not retain a native terminal result")
        self.evidence.artifact("dimsim-result.v1.json", _native_json(self.state.native))
        return PrivateScore(
            case_id=self.case.case_id,
            attempt_id=self.context.attempt_id,
            validator_revision=self.revision,
            passed=getattr(self.state.native, "passed", False) is True,
            prediction_status="native",
        )

    def close(self) -> None:
        if self.state.handle is not None and self.state.native is None:
            self.backend.cancel(self.state.handle, self.cancellation_s)


@dataclass(frozen=True)
class LiveAgentInterface:
    mcp: McpBinding
    session_id: str
    call_log: CodePolicyCallLog
    evidence: EvidenceSink
    prompt: str
    timeout_s: float


class LivePiAgent(AgentAdapter):
    def __init__(self, factory: PiSessionFactory) -> None:
        self.factory = factory
        self.session: PiSession | None = None
        self.interface: LiveAgentInterface | None = None

    def run(
        self,
        *,
        task: Any,
        context: AttemptContext,
        interface: Any = None,
    ) -> AgentOutcome:
        if not isinstance(task, EmbodiedInstructionTask):
            raise TypeError("live Pi agent requires an embodied instruction")
        if not isinstance(interface, LiveAgentInterface):
            raise TypeError("live Pi agent requires a live CodePolicy interface")
        if self.session is None:
            self.session = self.factory.create(
                attempt_path=context.path,
                public_prompt=task.prompt,
                code_policy_session_id=interface.session_id,
                call_log=interface.call_log,
                mcp=interface.mcp,
            )
        self.interface = interface
        turn = self.session.prompt(interface.prompt, interface.timeout_s)
        return AgentOutcome(
            final_text=turn.final_text,
            tool_call_count=turn.policy_call_count,
            terminal_reason="pi turn completed",
            agent_session_id=self.session.session_id,
            interaction_session_id=interface.session_id,
        )

    def abort(self, timeout_s: float) -> None:
        if self.session is not None:
            self.session.abort(timeout_s)

    def close(self) -> None:
        if self.session is None:
            return
        self.session.dispose()
        if self.interface is not None:
            for artifact in self.session.artifact_references():
                self.interface.evidence.reference(artifact.path)
        self.session = None
        self.interface = None


class LiveCodePolicyInteractionDriver:
    def __init__(
        self,
        *,
        memory_path: str,
        backend: SimulatorBackend,
        motion: MotionControl,
        timeouts: InfrastructureTimeouts,
        episode_timeout_s: float,
        process_factory: Callable[[CodePolicySessionConfig], LivePolicyProcess] = (
            StandaloneCodePolicyProcess
        ),
        mcp_factory: Callable[[str, float], McpBinding] | None = None,
    ) -> None:
        self.memory_path = memory_path
        self.backend = backend
        self.motion = motion
        self.timeouts = timeouts
        self.episode_timeout_s = episode_timeout_s
        self.process_factory = process_factory
        self.mcp_factory = mcp_factory or (
            lambda endpoint, timeout: McpAdapter(endpoint, timeout=max(1, int(timeout)))
        )
        self.process: LivePolicyProcess | None = None
        self.call_log: CodePolicyCallLog | None = None
        self.evidence: EvidenceSink | None = None
        self.agent: LivePiAgent | None = None

    def run(
        self,
        *,
        case: EvalCase,
        prepared_source: PreparedSource,
        agent: AgentAdapter,
        context: AttemptContext,
        evidence: EvidenceSink,
    ) -> AgentOutcome:
        if not isinstance(case.interaction, LiveCodePolicyInteraction):
            raise TypeError("live interaction requires LiveCodePolicyInteraction")
        if not isinstance(agent, LivePiAgent):
            raise TypeError("live interaction requires LivePiAgent")
        state = prepared_source.private_handle
        if not isinstance(state, LiveAttemptState) or state.handle is None:
            raise RuntimeError("native validator did not start before live interaction")
        process = self.process_factory(
            CodePolicySessionConfig(
                environment=LiveDimosEnvironment(recording_path=self.memory_path)
            )
        )
        process.start(self.timeouts.readiness_s)
        self.process = process
        self.evidence = evidence
        self.agent = agent
        mcp = self.mcp_factory(process.mcp_url, self.timeouts.mcp_call_s)
        evidence.artifact(
            "mcp-inventory.v1.json",
            wait_for_python_exec(process.mcp_url, mcp, self.timeouts.readiness_s),
        )
        receipt = process.receipt()
        evidence.artifact("code-policy-session.v1.json", receipt)
        session_id = str(receipt["session_id"])
        call_log = CodePolicyCallLog(context.path / "code-policy-calls.jsonl")
        self.call_log = call_log
        outcome = self._execute(
            case=case,
            state=state,
            agent=agent,
            context=context,
            evidence=evidence,
            mcp=mcp,
            session_id=session_id,
            call_log=call_log,
        )
        return outcome

    def _execute(
        self,
        *,
        case: EvalCase,
        state: LiveAttemptState,
        agent: LivePiAgent,
        context: AttemptContext,
        evidence: EvidenceSink,
        mcp: McpBinding,
        session_id: str,
        call_log: CodePolicyCallLog,
    ) -> AgentOutcome:
        deadline = time.monotonic() + self.episode_timeout_s
        handle = state.handle
        if handle is None:
            raise RuntimeError("live interaction is missing its evaluation handle")
        no_policy_turns = 0
        prompt = case.task.prompt
        last_outcome: AgentOutcome | None = None
        executor = ThreadPoolExecutor(max_workers=2)
        native_future = executor.submit(self.backend.wait_result, handle, self.episode_timeout_s)
        try:
            for turn_index in range(MAX_TURNS_SAFETY_CEILING):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                turn_future = executor.submit(
                    agent.run,
                    task=case.task,
                    context=context,
                    interface=LiveAgentInterface(
                        mcp=mcp,
                        session_id=session_id,
                        call_log=call_log,
                        evidence=evidence,
                        prompt=prompt,
                        timeout_s=remaining,
                    ),
                )
                pending_futures: set[Future[Any]] = {native_future, turn_future}
                done, _ = wait(
                    pending_futures,
                    timeout=remaining,
                    return_when=FIRST_COMPLETED,
                )
                if native_future in done and native_future.result() is not None:
                    state.native = native_future.result()
                    state.terminal_reason = _native_reason(state.native)
                    agent.abort(self.timeouts.cancellation_s)
                    break
                if turn_future not in done:
                    state.terminal_reason = "episode_timeout"
                    break
                before = last_outcome.tool_call_count if last_outcome is not None else 0
                last_outcome = turn_future.result()
                delta = max(0, last_outcome.tool_call_count - before)
                no_policy_turns = no_policy_turns + 1 if delta == 0 else 0
                evidence.event("pi-turn-ended", {"turn": turn_index + 1, "policy_calls": delta})
                if no_policy_turns >= 2:
                    state.terminal_reason = "two_consecutive_no_policy_calls"
                    break
                while self.motion.motion_active(min(1.0, max(0.001, deadline - time.monotonic()))):
                    if native_future.done() and native_future.result() is not None:
                        state.native = native_future.result()
                        state.terminal_reason = _native_reason(state.native)
                        break
                    if time.monotonic() >= deadline:
                        state.terminal_reason = "episode_timeout"
                        break
                if state.native is not None or state.terminal_reason == "episode_timeout":
                    break
                prompt = NEUTRAL_CONTINUATION
                evidence.event("pi-continuation")
            if state.native is None:
                self.backend.cancel(handle, self.timeouts.cancellation_s)
                state.native = self.backend.wait_result(handle, self.timeouts.cancellation_s)
            if state.native is None:
                raise RuntimeError("backend did not retain a native terminal result")
            if state.terminal_reason == "not_started":
                state.terminal_reason = _native_reason(state.native)
            if last_outcome is None:
                last_outcome = AgentOutcome(
                    final_text="",
                    tool_call_count=0,
                    terminal_reason=state.terminal_reason,
                    agent_session_id=(
                        agent.session.session_id
                        if agent.session is not None
                        else "pi-native-terminal"
                    ),
                    interaction_session_id=session_id,
                )
            return last_outcome.model_copy(update={"terminal_reason": state.terminal_reason})
        finally:
            agent.abort(self.timeouts.cancellation_s)
            executor.shutdown(wait=True, cancel_futures=True)

    def close(self) -> None:
        errors: list[Exception] = []
        try:
            self.motion.cancel_motion(self.timeouts.cancellation_s)
        except Exception as exc:
            errors.append(exc)
        if self.call_log is not None:
            self.call_log.close()
            if self.evidence is not None:
                self.evidence.reference("code-policy-calls.jsonl")
        if self.process is not None:
            try:
                if self.evidence is not None:
                    self.evidence.artifact(
                        "code-policy-records.v1.json",
                        cast("JsonValue", self.process.records()),
                    )
            finally:
                self.process.close()
        try:
            self.motion.close()
        except Exception as exc:
            errors.append(exc)
        if errors:
            raise RuntimeError("; ".join(f"{type(exc).__name__}: {exc}" for exc in errors))


def run_live_case(
    *,
    selected: SelectedDestination,
    backend: SimulatorBackend,
    motion: MotionControl,
    memory_path: str,
    output_root: Path,
    pi_factory: PiSessionFactory,
    timeouts: InfrastructureTimeouts,
    episode_timeout_s: float,
    process_factory: Callable[[CodePolicySessionConfig], LivePolicyProcess] = (
        StandaloneCodePolicyProcess
    ),
    mcp_factory: Callable[[str, float], McpBinding] | None = None,
) -> EngineResult:
    case = compile_dimsim_case(selected, timeout_seconds=episode_timeout_s)
    request = AttemptRequest(
        case=case,
        agent=AgentCondition(
            agent_id="pi-code-policy",
            adapter="pi-node",
            model="gpt-5.6-luna",
            thinking_level="medium",
        ),
        runtime=RuntimeBinding(runtime_id="attached-dimsim-standalone-code-policy"),
    )
    return AttemptEngine(
        request=request,
        output_root=output_root,
        source=LiveDimSimSourceDriver(
            selected=selected,
            backend=backend,
            timeouts=timeouts,
            episode_timeout_s=episode_timeout_s,
        ),
        interaction=LiveCodePolicyInteractionDriver(
            memory_path=memory_path,
            backend=backend,
            motion=motion,
            timeouts=timeouts,
            episode_timeout_s=episode_timeout_s,
            process_factory=process_factory,
            mcp_factory=mcp_factory,
        ),
        validator=DimSimNativeValidatorDriver(
            selected=selected,
            backend=backend,
            timeouts=timeouts,
        ),
        agent=LivePiAgent(pi_factory),
    ).run()


def _validate_reset(selected: SelectedDestination, reset: Any) -> None:
    source = selected.contract.source
    expected = {
        "scene_id": source.scene_id,
        "scene_revision": source.scene_revision,
        "reset_revision": source.reset_revision,
        "upstream_revision": source.upstream_revision,
        "profile_revision": source.profile_revision,
    }
    if reset.verified_source_revisions != expected:
        raise ValueError("post-reset source revisions do not match selected task")
    if reset.source_digest != source.oracle_view_digest:
        raise ValueError("post-reset source digest does not match selected task")
    if reset.requested_pose != selected.start_pose:
        raise ValueError("authoritative reset request does not match selected start pose")
    if reset.initial_predicate_satisfied:
        raise ValueError("selected task predicate is already satisfied after reset")


def _source_revisions(selected: SelectedDestination) -> dict[str, str]:
    source = selected.contract.source
    return {
        "scene_id": source.scene_id,
        "profile_revision": source.profile_revision,
        "reset_revision": source.reset_revision,
        "upstream_revision": source.upstream_revision,
    }


def _native_reason(native: Any) -> str:
    value = getattr(native, "reason", None)
    return value if isinstance(value, str) and value else "native_terminal"


def _native_json(native: Any) -> Any:
    if isinstance(native, BaseModel):
        return native.model_dump(mode="json")
    if isinstance(native, dict):
        return native
    raise TypeError("native backend result is not serializable")
