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

"""Evaluator-owned control loop for real-time simulator cases."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import threading
import time
from typing import Protocol, cast

from pydantic import JsonValue

from dimos.benchmark.agent_eval.case import (
    AgentOutcome,
    AttemptRequest,
    EvalOutcome,
    LiveCodePolicyInteraction,
    PrivateScore,
    SemanticObjectProximityGoal,
    SimulatorSceneSource,
    SourcePreparationRef,
)
from dimos.benchmark.agent_eval.engine import AttemptEvidence, EngineResult
from dimos.benchmark.agent_eval.periodic_goal import (
    SemanticGoalObservation,
    observe_semantic_proximity,
)
from dimos.benchmark.agent_eval.progress import ProgressSink, StatusProgress, emit_progress
from dimos.benchmark.agent_eval.store import AttemptStore
from dimos.e2e_tests.scene_contract import PlanarBounds


class RealtimeRuntime(Protocol):
    memory_path: Path

    def start(self) -> dict[str, JsonValue]: ...
    def prepare(self, recipe: SourcePreparationRef | None) -> dict[str, JsonValue]: ...
    def robot_position(self) -> tuple[float, float]: ...
    def semantic_object_bounds(self, query: str) -> PlanarBounds: ...
    def healthy(self) -> bool: ...
    def cancel_motion(self) -> None: ...
    def close(self) -> None: ...


class RealtimeRuntimeFactory(Protocol):
    def create(self, *, source: SimulatorSceneSource, attempt_path: Path) -> RealtimeRuntime: ...


class LiveAgentWorker(Protocol):
    def start(
        self,
        *,
        prompt: str,
        memory_path: Path,
        attempt_path: Path,
        evidence: AttemptEvidence,
    ) -> None: ...
    def failure(self) -> BaseException | None: ...
    def outcome(self, terminal_reason: str) -> AgentOutcome: ...
    def abort(self) -> None: ...
    def close(self) -> None: ...


class LiveAgentWorkerFactory(Protocol):
    def create(self) -> LiveAgentWorker: ...


class RealtimeAttemptEngine:
    """Supervise runtime, external agent, private polling, deadline, and cleanup."""

    def __init__(
        self,
        *,
        request: AttemptRequest,
        private_goal: SemanticObjectProximityGoal,
        output_root: Path,
        runtime_factory: RealtimeRuntimeFactory,
        agent_factory: LiveAgentWorkerFactory,
        monotonic: Callable[[], float] = time.monotonic,
        wait: Callable[[float], None] | None = None,
        progress: ProgressSink | None = None,
    ) -> None:
        if not isinstance(request.case.source, SimulatorSceneSource):
            raise TypeError("real-time engine requires a simulator-scene source")
        if not isinstance(request.case.interaction, LiveCodePolicyInteraction):
            raise TypeError("real-time engine requires a live CodePolicy interaction")
        self.request = request
        self.private_goal = private_goal
        self.output_root = output_root
        self.runtime_factory = runtime_factory
        self.agent_factory = agent_factory
        self.monotonic = monotonic
        self.wait = wait or (lambda seconds: threading.Event().wait(seconds))
        self.progress = progress

    def run(self) -> EngineResult:
        source = self.request.case.source
        interaction = self.request.case.interaction
        assert isinstance(source, SimulatorSceneSource)
        assert isinstance(interaction, LiveCodePolicyInteraction)
        store = AttemptStore(self.output_root)
        evidence = AttemptEvidence(store)
        runtime: RealtimeRuntime | None = None
        agent: LiveAgentWorker | None = None
        agent_outcome: AgentOutcome | None = None
        observations: list[SemanticGoalObservation] = []
        completed = False
        passed = False
        reason = "infrastructure failure"
        cleanup_diagnostics: list[dict[str, str]] = []
        try:
            evidence.event("attempt-created")
            evidence.artifact("case.private.v1.json", self.request.case)
            evidence.artifact("case.public.v1.json", self.request.case.public_projection())
            runtime = self.runtime_factory.create(
                source=source,
                attempt_path=store.path,
            )
            evidence.event("runtime-starting")
            self._status("starting case runtime")
            evidence.artifact("runtime-startup.v1.json", runtime.start())
            evidence.event("runtime-ready")
            self._status("case runtime ready")
            evidence.event("source-preparation-starting")
            self._status("preparing simulator source")
            evidence.artifact(
                "source-preparation.v1.json",
                runtime.prepare(source.preparation),
            )
            evidence.event("source-prepared")
            self._status("simulator source prepared")
            agent = self.agent_factory.create()
            deadline = self.monotonic() + interaction.timeout_seconds
            agent.start(
                prompt=self.request.case.task.prompt,
                memory_path=runtime.memory_path,
                attempt_path=store.path,
                evidence=evidence,
            )
            evidence.event("agent-started")
            self._status("agent active; monitoring goal")
            bounds = runtime.semantic_object_bounds(self.private_goal.semantic_query)
            while True:
                failure = agent.failure()
                if failure is not None:
                    raise RuntimeError(
                        f"live agent failed: {type(failure).__name__}: {failure}"
                    ) from failure
                if not runtime.healthy():
                    raise RuntimeError("live runtime became unavailable")
                observation = self._observe(runtime, bounds)
                observations.append(observation)
                evidence.event("goal-observed", {"available": True})
                if observation.passed:
                    completed = True
                    passed = True
                    reason = "goal_reached"
                    self._status("goal reached")
                    break
                remaining = deadline - self.monotonic()
                if remaining <= 0:
                    # Deliberately re-sample at the boundary before classifying timeout.
                    final_observation = self._observe(runtime, bounds)
                    observations.append(final_observation)
                    if final_observation.passed:
                        completed = True
                        passed = True
                        reason = "goal_reached"
                        self._status("goal reached")
                    else:
                        completed = True
                        reason = "episode_timeout"
                        self._status("episode deadline reached")
                    break
                self.wait(min(self.private_goal.poll_interval_seconds, remaining))
        except KeyboardInterrupt:
            reason = "user interrupted"
            evidence.event("attempt-interrupted")
        except Exception as exc:
            reason = f"{type(exc).__name__}: {exc}"[:1024]
            evidence.event("infrastructure-failure", {"diagnostic": reason})
        finally:
            self._status("cleaning up live attempt")
            if agent is not None:
                try:
                    agent.abort()
                except Exception as exc:
                    cleanup_diagnostics.append({"resource": "agent-abort", "diagnostic": str(exc)})
                    evidence.event(
                        "cleanup-failure", {"resource": "agent-abort", "diagnostic": str(exc)}
                    )
            if runtime is not None:
                try:
                    runtime.cancel_motion()
                except Exception as exc:
                    cleanup_diagnostics.append({"resource": "motion", "diagnostic": str(exc)})
                    evidence.event(
                        "cleanup-failure", {"resource": "motion", "diagnostic": str(exc)}
                    )
            if agent is not None:
                try:
                    agent_outcome = agent.outcome(reason)
                    evidence.artifact("agent-outcome.v1.json", agent_outcome)
                except Exception as exc:
                    cleanup_diagnostics.append(
                        {"resource": "agent-outcome", "diagnostic": str(exc)}
                    )
                    evidence.event(
                        "cleanup-failure", {"resource": "agent-outcome", "diagnostic": str(exc)}
                    )
                try:
                    agent.close()
                except Exception as exc:
                    cleanup_diagnostics.append({"resource": "agent", "diagnostic": str(exc)})
                    evidence.event("cleanup-failure", {"resource": "agent", "diagnostic": str(exc)})
            if runtime is not None:
                try:
                    runtime.close()
                except Exception as exc:
                    cleanup_diagnostics.append({"resource": "runtime", "diagnostic": str(exc)})
                    evidence.event(
                        "cleanup-failure", {"resource": "runtime", "diagnostic": str(exc)}
                    )

        if observations:
            evidence.artifact(
                "goal-observations.private.v1.json",
                [observation.model_dump(mode="json") for observation in observations],
            )
        if completed:
            score = PrivateScore(
                case_id=self.request.case.case_id,
                attempt_id=store.attempt_id,
                validator_revision=self.request.case.validator.revision,
                passed=passed,
                prediction_status="native",
            )
            evidence.artifact("score.private.v1.json", score)
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
                "case_id": self.request.case.case_id,
                "case_fingerprint": self.request.case.fingerprint,
                "agent": self.request.agent.model_dump(mode="json"),
                "runtime": self.request.runtime.model_dump(mode="json"),
                "agent_session_id": (
                    agent_outcome.agent_session_id if agent_outcome is not None else None
                ),
                "interaction_session_id": (
                    agent_outcome.interaction_session_id if agent_outcome is not None else None
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

    def _observe(self, runtime: RealtimeRuntime, bounds: PlanarBounds) -> SemanticGoalObservation:
        x, y = runtime.robot_position()
        return observe_semantic_proximity(
            goal=self.private_goal,
            bounds=bounds,
            robot_x=x,
            robot_y=y,
            observed_at_monotonic=self.monotonic(),
        )
