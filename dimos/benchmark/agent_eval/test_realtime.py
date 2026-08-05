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

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from typer.testing import CliRunner

from dimos.benchmark.agent_eval.case import (
    AgentCondition,
    AgentOutcome,
    AttemptRequest,
    EmbodiedInstructionTask,
    EvalCase,
    LiveCodePolicyInteraction,
    PeriodicGoalValidatorRef,
    RuntimeBinding,
    SemanticObjectProximityGoal,
    SimulatorSceneSource,
)
from dimos.benchmark.agent_eval.realtime import RealtimeAttemptEngine
import dimos.benchmark.agent_eval.single_case as single_case
from dimos.cli.dimos import main
from dimos.e2e_tests.scene_contract import PlanarBounds


class Clock:
    def __init__(self) -> None:
        self.value = 0.0

    def monotonic(self) -> float:
        return self.value

    def wait(self, seconds: float) -> None:
        self.value += seconds


class FakeRuntime:
    def __init__(self, positions: list[tuple[float, float]]) -> None:
        self.memory_path = Path("/tmp/fake-live-memory.db")
        self.positions = positions
        self.started = False
        self.prepared = False
        self.cancelled = False
        self.closed = False
        self.healthy_value = True
        self.cancel_error = False
        self.start_error = False
        self.prepare_error = False

    def start(self) -> dict[str, Any]:
        if self.start_error:
            raise RuntimeError("startup failed")
        self.started = True
        return {"schema_version": "1.0", "ready": True}

    def prepare(self, recipe: Any) -> dict[str, Any]:
        if self.prepare_error:
            raise RuntimeError("preparation failed")
        self.prepared = True
        return {"schema_version": "1.0", "completed": True}

    def robot_position(self) -> tuple[float, float]:
        if not self.positions:
            raise RuntimeError("odometry unavailable")
        return self.positions.pop(0) if len(self.positions) > 1 else self.positions[0]

    def semantic_object_bounds(self, query: str) -> PlanarBounds:
        assert query in {"private bed", "queen size bed"}
        return PlanarBounds(min_x=0.0, min_y=0.0, max_x=1.0, max_y=1.0)

    def healthy(self) -> bool:
        return self.healthy_value

    def cancel_motion(self) -> None:
        self.cancelled = True
        if self.cancel_error:
            raise RuntimeError("cancel failed")

    def close(self) -> None:
        self.closed = True


class FakeRuntimeFactory:
    def __init__(self, runtime: FakeRuntime) -> None:
        self.runtime = runtime

    def create(self, **kwargs: Any) -> FakeRuntime:
        return self.runtime


class FakeAgent:
    def __init__(self) -> None:
        self.started = False
        self.aborted = False
        self.closed = False
        self.failure_value: BaseException | None = None

    def start(self, **kwargs: Any) -> None:
        assert kwargs["prompt"] == "Go to the bed"
        self.started = True

    def failure(self) -> BaseException | None:
        return self.failure_value

    def outcome(self, terminal_reason: str) -> AgentOutcome:
        return AgentOutcome(
            final_text="",
            tool_call_count=1,
            terminal_reason=terminal_reason,
            agent_session_id="pi-session",
            interaction_session_id="code-policy-session",
        )

    def abort(self) -> None:
        self.aborted = True

    def close(self) -> None:
        self.closed = True


class FakeAgentFactory:
    def __init__(self, agent: FakeAgent) -> None:
        self.agent = agent

    def create(self) -> FakeAgent:
        return self.agent


def _request(timeout: float = 2.0) -> AttemptRequest:
    case = EvalCase.compile(
        case_id="go-to-bed",
        source=SimulatorSceneSource(
            scene="apartment",
            simulation_provider="pimsim",
            robot="unitree-go2",
            dimos_blueprint="unitree-go2",
        ),
        task=EmbodiedInstructionTask(prompt="Go to the bed"),
        interaction=LiveCodePolicyInteraction(driver_revision="v1", timeout_seconds=timeout),
        validator=PeriodicGoalValidatorRef(
            revision="v1",
            private_path="private/goal.json",
            private_sha256="a" * 64,
        ),
    )
    return AttemptRequest(
        case=case,
        agent=AgentCondition(
            agent_id="pi", adapter="node", model="gpt-5.6-luna", thinking_level="medium"
        ),
        runtime=RuntimeBinding(runtime_id="case-bound"),
    )


def _run(
    tmp_path: Path,
    positions: list[tuple[float, float]],
    *,
    timeout: float = 2.0,
    configure: Any = None,
    wait: Any = None,
):
    runtime = FakeRuntime(positions)
    agent = FakeAgent()
    clock = Clock()
    if configure is not None:
        configure(runtime, agent, clock)
    engine = RealtimeAttemptEngine(
        request=_request(timeout),
        private_goal=SemanticObjectProximityGoal(
            semantic_query="private bed",
            maximum_distance_metres=1.0,
            poll_interval_seconds=1.0,
        ),
        output_root=tmp_path,
        runtime_factory=FakeRuntimeFactory(runtime),
        agent_factory=FakeAgentFactory(agent),
        monotonic=clock.monotonic,
        wait=wait or clock.wait,
    )
    return engine.run(), runtime, agent


def test_goal_success_while_agent_is_active_is_evaluator_owned(tmp_path: Path) -> None:
    result, runtime, agent = _run(tmp_path, [(5.0, 5.0), (1.5, 1.0)])
    assert (result.outcome.attempt_status, result.outcome.task_result) == (
        "completed",
        "passed",
    )
    assert agent.aborted and runtime.cancelled and runtime.closed


def test_early_agent_return_does_not_end_monitoring_and_timeout_is_task_failure(
    tmp_path: Path,
) -> None:
    result, _, agent = _run(tmp_path, [(5.0, 5.0)])
    assert agent.failure() is None
    assert result.outcome.reason == "episode_timeout"
    assert (result.outcome.attempt_status, result.outcome.task_result) == (
        "completed",
        "failed",
    )


def test_final_deadline_check_can_observe_edge_success(tmp_path: Path) -> None:
    result, _, _ = _run(
        tmp_path,
        [(5.0, 5.0), (5.0, 5.0), (5.0, 5.0), (1.0, 1.0)],
    )
    assert result.outcome.task_result == "passed"


def test_unavailable_goal_state_is_infrastructure_failure(tmp_path: Path) -> None:
    result, _, _ = _run(tmp_path, [])
    assert result.outcome.attempt_status == "failed"
    assert result.outcome.task_result == "not_evaluated"
    assert "odometry unavailable" in result.outcome.reason


def test_startup_or_preparation_failure_never_starts_agent(tmp_path: Path) -> None:
    for failure in ("start_error", "prepare_error"):

        def configure(
            runtime: FakeRuntime,
            agent: FakeAgent,
            clock: Clock,
            selected: str = failure,
        ) -> None:
            setattr(runtime, selected, True)

        result, runtime, agent = _run(
            tmp_path / failure,
            [(5.0, 5.0)],
            configure=configure,
        )
        assert result.outcome.task_result == "not_evaluated"
        assert agent.started is False
        assert runtime.closed is True


def test_interruption_and_cancellation_failure_retain_primary_outcome(tmp_path: Path) -> None:
    def configure(runtime: FakeRuntime, agent: FakeAgent, clock: Clock) -> None:
        runtime.cancel_error = True

    def interrupt(seconds: float) -> None:
        raise KeyboardInterrupt

    result, _, _ = _run(
        tmp_path,
        [(5.0, 5.0)],
        configure=configure,
        wait=interrupt,
    )
    assert result.outcome.reason == "user interrupted"
    assert result.outcome.task_result == "not_evaluated"
    events = (result.attempt_path / "events.jsonl").read_text()
    assert "cleanup-failure" in events


def test_private_goal_details_never_enter_public_artifacts(tmp_path: Path) -> None:
    result, _, _ = _run(tmp_path, [(1.0, 1.0)])
    public = (result.attempt_path / "case.public.v1.json").read_text()
    outcome = (result.attempt_path / "outcome.v1.json").read_text()
    assert "private bed" not in public + outcome
    private = json.loads((result.attempt_path / "goal-observations.private.v1.json").read_text())
    assert private[0]["passed"] is True


def test_checked_in_case_runs_end_to_end_through_canonical_cli_with_fakes(
    tmp_path: Path, monkeypatch
) -> None:
    runtime = FakeRuntime([(1.0, 1.0)])
    agent = FakeAgent()

    class RuntimeFactory:
        def preflight(self, source: SimulatorSceneSource) -> None:
            assert source.simulation_provider == "pimsim"

        def create(self, **kwargs: Any) -> FakeRuntime:
            return runtime

    class AgentFactory:
        def __init__(self, **kwargs: Any) -> None: ...

        def create(self) -> FakeAgent:
            return agent

    monkeypatch.setattr(single_case, "DimosSimulatorRuntimeFactory", RuntimeFactory)
    monkeypatch.setattr(single_case, "ExternalPiWorkerFactory", AgentFactory)
    auth = tmp_path / "auth.json"
    auth.write_text("{}")
    case = (
        Path(__file__).parents[1]
        / "realtime_sim"
        / "cases"
        / "go2-apartment-go-to-bed"
        / "case.json"
    )
    output = tmp_path / "attempts"

    result = CliRunner().invoke(
        main,
        [
            "eval",
            "run",
            str(case),
            f"--agent.auth.path={auth}",
            f"--output={output}",
            "--quiet",
        ],
    )

    assert result.exit_code == 0, result.output
    assert "Evaluation passed" in result.output
    assert "Task       Go to the bed" in result.output
    attempts = list(output.glob("attempt_*"))
    assert len(attempts) == 1
    outcome = json.loads((attempts[0] / "outcome.v1.json").read_text())
    assert outcome["task_result"] == "passed"
    assert runtime.prepared and runtime.closed and agent.aborted
