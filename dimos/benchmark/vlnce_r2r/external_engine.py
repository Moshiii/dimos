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

"""Evaluator-owned attempt lifecycle for the external VLN-CE benchmark."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import threading
import time
from typing import Literal, Protocol, cast

from pydantic import JsonValue

from dimos.benchmark.agent_eval.case import (
    AgentOutcome,
    AttemptRequest,
    BenchmarkNativeResultValidatorRef,
    EvalCase,
    EvalOutcome,
    ExternalBenchmarkEpisodeSource,
    LiveCodePolicyInteraction,
    PrivateScore,
)
from dimos.benchmark.agent_eval.engine import AttemptEvidence, EngineResult
from dimos.benchmark.agent_eval.progress import ProgressSink, StatusProgress, emit_progress
from dimos.benchmark.agent_eval.realtime import LiveAgentWorker, LiveAgentWorkerFactory
from dimos.benchmark.agent_eval.store import AttemptStore
from dimos.benchmark.vlnce_r2r.external_runtime import (
    VlnceExternalRuntime,
    preparation_evidence,
)
from dimos.benchmark.vlnce_r2r.native_result import VlnceNativeResult, validate_native_result
from dimos.benchmark.vlnce_r2r.preparation import PreparationReceipt
from dimos.benchmark.vlnce_r2r.prompt import vlnce_task_prompt

RESULT_SCHEMA_PATH = Path(__file__).with_name("result-schema.v1.json")
RUNTIME_EXIT_GRACE_SECONDS = 30.0


class ExternalRuntime(Protocol):
    memory_path: Path
    result_path: Path
    log_path: Path
    render_path: Path

    def start(self) -> dict[str, JsonValue]: ...
    def healthy(self) -> bool: ...
    def result_bytes(self) -> bytes | None: ...
    def public_evidence(self) -> dict[str, JsonValue]: ...
    def cancel_motion(self) -> None: ...
    def render_evidence(self) -> dict[str, JsonValue] | None: ...
    def close(self) -> None: ...


class ExternalRuntimeFactory(Protocol):
    def create(
        self,
        *,
        case: EvalCase,
        attempt_id: str,
        attempt_path: Path,
        preparation: PreparationReceipt,
        image_id: str,
    ) -> ExternalRuntime: ...


class VlnceExternalRuntimeFactory:
    """Construct the production OCI and case-bound DimOS runtime."""

    def __init__(self, *, render: Literal["none", "native"] = "none") -> None:
        self.render = render

    def create(
        self,
        *,
        case: EvalCase,
        attempt_id: str,
        attempt_path: Path,
        preparation: PreparationReceipt,
        image_id: str,
    ) -> ExternalRuntime:
        return VlnceExternalRuntime(
            case=case,
            attempt_id=attempt_id,
            attempt_path=attempt_path,
            preparation=preparation,
            image_id=image_id,
            render=self.render,
        )


class ExternalBenchmarkAttemptEngine:
    """Supervise one native-scored external benchmark attempt."""

    def __init__(
        self,
        *,
        request: AttemptRequest,
        output_root: Path,
        preparation: PreparationReceipt,
        image_id: str,
        runtime_factory: ExternalRuntimeFactory,
        agent_factory: LiveAgentWorkerFactory,
        monotonic: Callable[[], float] = time.monotonic,
        wait: Callable[[float], None] | None = None,
        progress: ProgressSink | None = None,
    ) -> None:
        if not isinstance(request.case.source, ExternalBenchmarkEpisodeSource):
            raise TypeError("external benchmark engine requires an external source")
        if not isinstance(request.case.interaction, LiveCodePolicyInteraction):
            raise TypeError("external benchmark engine requires a live interaction")
        if not isinstance(request.case.validator, BenchmarkNativeResultValidatorRef):
            raise TypeError("external benchmark engine requires a native result validator")
        self.request = request
        self.output_root = output_root
        self.preparation = preparation
        self.image_id = image_id
        self.runtime_factory = runtime_factory
        self.agent_factory = agent_factory
        self.monotonic = monotonic
        self.wait = wait or (lambda seconds: threading.Event().wait(seconds))
        self.progress = progress

    def run(self) -> EngineResult:
        case = self.request.case
        interaction = case.interaction
        validator = case.validator
        assert isinstance(interaction, LiveCodePolicyInteraction)
        assert isinstance(validator, BenchmarkNativeResultValidatorRef)
        store = AttemptStore(self.output_root)
        evidence = AttemptEvidence(store)
        runtime: ExternalRuntime | None = None
        agent: LiveAgentWorker | None = None
        agent_outcome: AgentOutcome | None = None
        native_result: VlnceNativeResult | None = None
        completed = False
        passed = False
        reason = "infrastructure failure"
        cleanup_diagnostics: list[dict[str, str]] = []
        try:
            evidence.event("attempt-created")
            evidence.artifact("case.private.v1.json", case)
            evidence.artifact("case.public.v1.json", case.public_projection())
            evidence.artifact(
                "source-preparation.v1.json",
                preparation_evidence(self.preparation, self.image_id),
            )
            evidence.event("source-prepared")
            self._status("starting external benchmark runtime")
            runtime = self.runtime_factory.create(
                case=case,
                attempt_id=store.attempt_id,
                attempt_path=store.path,
                preparation=self.preparation,
                image_id=self.image_id,
            )
            evidence.artifact("runtime-startup.v1.json", runtime.start())
            evidence.event("runtime-ready")
            self._status("benchmark runtime ready; starting agent")
            agent = self.agent_factory.create()
            agent.start(
                prompt=vlnce_task_prompt(case.task.prompt),
                memory_path=runtime.memory_path,
                attempt_path=store.path,
                evidence=evidence,
            )
            evidence.event("agent-started")
            self._status("agent active; waiting for native terminal result")
            deadline = self.monotonic() + interaction.timeout_seconds + RUNTIME_EXIT_GRACE_SECONDS
            while True:
                failure = agent.failure()
                if failure is not None:
                    raise RuntimeError(
                        f"live agent failed: {type(failure).__name__}: {failure}"
                    ) from failure
                payload = runtime.result_bytes()
                if payload is not None:
                    native_result = validate_native_result(
                        payload,
                        case=case,
                        attempt_id=store.attempt_id,
                        schema_path=RESULT_SCHEMA_PATH,
                    )
                    if runtime.result_path.relative_to(store.path).as_posix() != (
                        validator.result_filename
                    ):
                        raise ValueError("native result path does not match the case contract")
                    evidence.reference(validator.result_filename)
                    evidence.event(
                        "native-result-validated",
                        {
                            "terminal_reason": native_result.terminal_reason,
                            "success": native_result.metrics.SUCCESS,
                        },
                    )
                    evidence.artifact("runtime-public-terminal.v1.json", runtime.public_evidence())
                    completed = True
                    passed = native_result.metrics.SUCCESS == 1.0
                    reason = native_result.terminal_reason
                    self._status("official benchmark result received")
                    break
                if not runtime.healthy():
                    raise RuntimeError("external benchmark runtime became unavailable")
                if self.monotonic() >= deadline:
                    raise TimeoutError("benchmark did not publish its terminal result")
                self.wait(0.1)
        except KeyboardInterrupt:
            reason = "user interrupted"
            evidence.event("attempt-interrupted")
        except Exception as exc:
            reason = f"{type(exc).__name__}: {exc}"[:1024]
            evidence.event("infrastructure-failure", {"diagnostic": reason})
        finally:
            self._status("cleaning up external benchmark attempt")
            if agent is not None:
                self._cleanup_call("agent-abort", agent.abort, evidence, cleanup_diagnostics)
            if runtime is not None:
                self._cleanup_call("motion", runtime.cancel_motion, evidence, cleanup_diagnostics)
            if agent is not None:
                try:
                    agent_outcome = agent.outcome(reason)
                    evidence.artifact("agent-outcome.v1.json", agent_outcome)
                except Exception as exc:
                    self._record_cleanup_failure(
                        "agent-outcome", exc, evidence, cleanup_diagnostics
                    )
                self._cleanup_call("agent", agent.close, evidence, cleanup_diagnostics)
            if runtime is not None:
                self._cleanup_call("runtime", runtime.close, evidence, cleanup_diagnostics)
                self._collect_render_evidence(runtime, evidence)
                if runtime.log_path.is_file():
                    evidence.reference(runtime.log_path.relative_to(store.path).as_posix())

        if completed:
            evidence.artifact(
                "score.private.v1.json",
                PrivateScore(
                    case_id=case.case_id,
                    attempt_id=store.attempt_id,
                    validator_revision=validator.revision,
                    passed=passed,
                    prediction_status="native",
                ),
            )
        evidence.artifact(
            "cleanup.v1.json",
            cast(
                "dict[str, JsonValue]",
                {
                    "schema_version": "1.0",
                    "completed": True,
                    "diagnostics": cleanup_diagnostics,
                },
            ),
        )
        outcome = EvalOutcome(
            attempt_id=store.attempt_id,
            attempt_status="completed" if completed else "failed",
            task_result=("passed" if passed else "failed") if completed else "not_evaluated",
            reason=reason,
        )
        evidence.reference("events.jsonl")
        evidence.artifact(
            "attempt-manifest.v1.json",
            {
                "schema_version": "1.0",
                "attempt_id": store.attempt_id,
                "case_id": case.case_id,
                "case_fingerprint": case.fingerprint,
                "agent": self.request.agent.model_dump(mode="json"),
                "runtime": self.request.runtime.model_dump(mode="json"),
                "agent_session_id": (
                    agent_outcome.agent_session_id if agent_outcome is not None else None
                ),
                "interaction_session_id": (
                    agent_outcome.interaction_session_id if agent_outcome is not None else None
                ),
                "native_metrics": (
                    native_result.metrics.model_dump(mode="json")
                    if native_result is not None
                    else None
                ),
                "artifacts": [artifact.model_dump(mode="json") for artifact in evidence.artifacts],
            },
        )
        evidence.artifacts.append(store.write_eval_outcome(outcome))
        store.close()
        return EngineResult(
            attempt_path=store.path,
            outcome=outcome,
            artifacts=tuple(evidence.artifacts),
        )

    def _status(self, message: str) -> None:
        emit_progress(self.progress, StatusProgress(channel="eval", message=message))

    def _collect_render_evidence(
        self,
        runtime: ExternalRuntime,
        evidence: AttemptEvidence,
    ) -> None:
        try:
            payload = runtime.render_evidence()
        except Exception as error:
            payload = {
                "schema_version": "native-render.v1",
                "status": "failed",
                "diagnostic": f"could not collect renderer metadata: {type(error).__name__}",
            }
        if payload is None:
            return
        if payload.get("schema_version") != "native-render.v1":
            payload = {
                "schema_version": "native-render.v1",
                "status": "failed",
                "diagnostic": "renderer metadata schema was invalid",
            }
        completed = payload.get("status") == "completed" and runtime.render_path.is_file()
        if completed:
            evidence.reference(runtime.render_path.relative_to(evidence.store.path).as_posix())
            evidence.event("native-render-completed")
        else:
            payload["status"] = "failed"
            evidence.event(
                "native-render-failed",
                {"diagnostic": str(payload.get("diagnostic", "native video was unavailable"))},
            )
            self._status("warning: native render was not produced; official scoring is unchanged")
        evidence.artifact("native-render.v1.json", payload)

    @staticmethod
    def _cleanup_call(
        resource: str,
        action: Callable[[], None],
        evidence: AttemptEvidence,
        diagnostics: list[dict[str, str]],
    ) -> None:
        try:
            action()
        except Exception as exc:
            ExternalBenchmarkAttemptEngine._record_cleanup_failure(
                resource, exc, evidence, diagnostics
            )

    @staticmethod
    def _record_cleanup_failure(
        resource: str,
        error: Exception,
        evidence: AttemptEvidence,
        diagnostics: list[dict[str, str]],
    ) -> None:
        diagnostic = f"{type(error).__name__}: {error}"[:1024]
        diagnostics.append({"resource": resource, "diagnostic": diagnostic})
        evidence.event(
            "cleanup-failure",
            {"resource": resource, "diagnostic": diagnostic},
        )
