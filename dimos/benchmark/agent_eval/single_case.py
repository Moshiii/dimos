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

"""One immutable case bound to typed local agent execution configuration."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import time
from typing import Annotated, Literal, TypeVar

from pydantic import Field

from dimos.benchmark.agent_eval.base import BaseEvalModel
from dimos.benchmark.agent_eval.case import (
    AgentCondition,
    AgentOutcome,
    AttemptRequest,
    EvalCase,
    FrozenRecordingSource,
    LiveCodePolicyInteraction,
    Prediction,
    RuntimeBinding,
    SimulatorSceneSource,
)
from dimos.benchmark.agent_eval.config import RuntimeCredential
from dimos.benchmark.agent_eval.periodic_goal import load_periodic_goal
from dimos.benchmark.agent_eval.pi_adapter import credential_binding_sha256
from dimos.benchmark.agent_eval.pi_process import NodePiSessionFactory
from dimos.benchmark.agent_eval.progress import (
    CaseHeaderProgress,
    ProgressSink,
    StatusProgress,
    emit_progress,
)
from dimos.benchmark.agent_eval.realtime import RealtimeAttemptEngine
from dimos.benchmark.agent_eval.realtime_agent import ExternalPiWorkerFactory
from dimos.benchmark.agent_eval.realtime_runtime import DimosSimulatorRuntimeFactory
from dimos.benchmark.short_horizon_qa.eval import (
    load_exact_integer_oracle,
    run_frozen_case,
)
from dimos.benchmark.short_horizon_qa.prepare import prepare_bundle
from dimos.constants import CACHE_DIR, STATE_DIR

DEFAULT_MODEL: Literal["gpt-5.6-luna"] = "gpt-5.6-luna"
DEFAULT_OUTPUT_ROOT = STATE_DIR / "evals"
DEFAULT_CODEX_AUTH_PATH = Path.home() / ".pi" / "agent" / "auth.json"
DEFAULT_OPENAI_API_KEY_ENV = "OPENAI_API_KEY"
SINGLE_CASE_TURN_TIMEOUT_SECONDS = 600.0
EvalModelT = TypeVar("EvalModelT", bound=BaseEvalModel)


class CodexOAuthConfig(BaseEvalModel):
    mode: Literal["codex-oauth"] = "codex-oauth"
    path: Path | None = None


class OpenAIApiKeyConfig(BaseEvalModel):
    mode: Literal["openai-api-key"] = "openai-api-key"
    env: str = Field(default=DEFAULT_OPENAI_API_KEY_ENV, min_length=1)


AgentAuthConfig = Annotated[
    CodexOAuthConfig | OpenAIApiKeyConfig,
    Field(discriminator="mode"),
]


class PiAgentConfig(BaseEvalModel):
    backend: Literal["pi"] = "pi"
    model: Literal["gpt-5.6-luna"] = DEFAULT_MODEL
    thinking_level: Literal["medium"] = "medium"
    auth: AgentAuthConfig = Field(default_factory=CodexOAuthConfig)


class EvalRunConfig(BaseEvalModel):
    agent: PiAgentConfig = Field(default_factory=PiAgentConfig)


class CompactEvalResult(BaseEvalModel):
    attempt_id: str
    case_id: str
    source: str
    source_kind: Literal["frozen_memory", "simulator_scene"] = "frozen_memory"
    progress: float | None
    question: str
    attempt_status: Literal["completed", "failed"]
    task_result: Literal["passed", "failed", "not_evaluated"]
    reason: str
    prediction_status: Literal["parsed", "invalid"] | None = None
    integer_answer: int | None = None
    agent: AgentCondition
    tool_call_count: int = Field(ge=0)
    duration_seconds: float = Field(ge=0)
    artifact_path: Path


def execute_single_case(
    case_path: Path,
    *,
    config: EvalRunConfig,
    output_root: Path = DEFAULT_OUTPUT_ROOT,
    progress: ProgressSink | None = None,
) -> CompactEvalResult:
    """Preflight and execute exactly one canonical static or real-time case."""
    path = case_path.expanduser().resolve()
    emit_progress(progress, StatusProgress(channel="eval", message="loading case"))
    case = EvalCase.model_validate_json(path.read_bytes())
    task = case.task
    source_name = (
        case.source.recording
        if isinstance(case.source, FrozenRecordingSource)
        else case.source.scene
        if isinstance(case.source, SimulatorSceneSource)
        else case.source.runtime
    )
    emit_progress(
        progress,
        CaseHeaderProgress(
            case_id=case.case_id,
            source=source_name,
            progress=(
                case.source.progress if isinstance(case.source, FrozenRecordingSource) else None
            ),
            question=getattr(task, "prompt", ""),
        ),
    )

    # Resolve all private case material before starting Pi.
    emit_progress(progress, StatusProgress(channel="eval", message="verifying validator"))
    private_goal = None
    runtime_factory = None
    if isinstance(case.source, FrozenRecordingSource):
        load_exact_integer_oracle(case, path.parent)
        emit_progress(progress, StatusProgress(channel="eval", message="preparing frozen memory"))
        bundle = _materialize_frozen_memory(case)
        emit_progress(progress, StatusProgress(channel="eval", message="frozen memory ready"))
    elif isinstance(case.source, SimulatorSceneSource):
        private_goal = load_periodic_goal(case, path.parent)
        runtime_factory = DimosSimulatorRuntimeFactory()
        emit_progress(progress, StatusProgress(channel="eval", message="checking simulator"))
        runtime_factory.preflight(case.source)
        emit_progress(progress, StatusProgress(channel="eval", message="simulator ready"))
    else:
        raise ValueError("single-case CLI does not support legacy live_dimos sources")
    credential, binding_digest = _resolve_credential(config.agent.auth)
    adapter = _adapter_entrypoint()
    condition = AgentCondition(
        agent_id="pi-code-policy",
        adapter="pi-node",
        model=config.agent.model,
        thinking_level=config.agent.thinking_level,
    )
    runtime = RuntimeBinding(
        runtime_id=(
            "local-standalone-code-policy"
            if isinstance(case.source, FrozenRecordingSource)
            else "case-bound-simulator-scene"
        ),
        parameters={
            "auth_mode": config.agent.auth.mode,
            "credential_binding_sha256": binding_digest,
            "turn_timeout_seconds": SINGLE_CASE_TURN_TIMEOUT_SECONDS,
        },
    )
    factory = NodePiSessionFactory(
        command=("node", str(adapter)),
        credential=credential,
        model=config.agent.model,
        thinking_level=config.agent.thinking_level,
        startup_timeout_s=180.0,
        progress=progress,
    )
    emit_progress(progress, StatusProgress(channel="eval", message="starting attempt"))
    started = time.monotonic()
    if isinstance(case.source, FrozenRecordingSource):
        engine_result = run_frozen_case(
            case=case,
            bundle=bundle,
            private_root=path.parent,
            output_root=output_root.expanduser(),
            pi_factory=factory,
            agent_condition=condition,
            runtime_binding=runtime,
            turn_timeout_s=SINGLE_CASE_TURN_TIMEOUT_SECONDS,
        )
    else:
        assert (
            isinstance(case.interaction, LiveCodePolicyInteraction)
            and private_goal is not None
            and runtime_factory is not None
        )
        request = AttemptRequest(case=case, agent=condition, runtime=runtime)
        engine_result = RealtimeAttemptEngine(
            request=request,
            private_goal=private_goal,
            output_root=output_root.expanduser(),
            runtime_factory=runtime_factory,
            agent_factory=ExternalPiWorkerFactory(
                pi_factory=factory,
                turn_timeout_seconds=case.interaction.timeout_seconds,
            ),
            progress=progress,
        ).run()
    duration = time.monotonic() - started
    emit_progress(progress, StatusProgress(channel="eval", message="attempt finished"))
    prediction = _optional_model(engine_result.attempt_path / "prediction.v1.json", Prediction)
    agent_outcome = _optional_model(
        engine_result.attempt_path / "agent-outcome.v1.json", AgentOutcome
    )
    return CompactEvalResult(
        attempt_id=engine_result.outcome.attempt_id,
        case_id=case.case_id,
        source=source_name,
        source_kind=(
            "frozen_memory" if isinstance(case.source, FrozenRecordingSource) else "simulator_scene"
        ),
        progress=(case.source.progress if isinstance(case.source, FrozenRecordingSource) else None),
        question=getattr(task, "prompt", ""),
        attempt_status=engine_result.outcome.attempt_status,
        task_result=engine_result.outcome.task_result,
        reason=engine_result.outcome.reason,
        prediction_status=prediction.status if prediction is not None else None,
        integer_answer=prediction.integer_answer if prediction is not None else None,
        agent=condition,
        tool_call_count=agent_outcome.tool_call_count if agent_outcome is not None else 0,
        duration_seconds=duration,
        artifact_path=engine_result.attempt_path,
    )


def _resolve_credential(auth: AgentAuthConfig) -> tuple[RuntimeCredential, str]:
    if isinstance(auth, CodexOAuthConfig):
        configured = auth.path or Path(
            os.environ.get("PI_SPATIAL_AUTH_PATH", DEFAULT_CODEX_AUTH_PATH)
        )
        path = configured.expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(
                f"Codex OAuth credential not found at {path}; use --agent.auth.path"
            )
        material = path.read_bytes()
        return (
            RuntimeCredential(auth_mode="subscription", binding_name=str(path), value=None),
            credential_binding_sha256("subscription", str(path), material),
        )
    value = os.environ.get(auth.env)
    if not value:
        raise ValueError(f"credential environment variable {auth.env!r} is unset")
    return (
        RuntimeCredential(auth_mode="environment", binding_name=auth.env, value=value),
        credential_binding_sha256("environment", auth.env, value),
    )


def _materialize_frozen_memory(case: EvalCase) -> Path:
    source = case.source
    assert isinstance(source, FrozenRecordingSource)
    key = hashlib.sha256(source.model_dump_json().encode()).hexdigest()[:24]
    bundle = CACHE_DIR / "agent_eval" / "frozen_memory" / key
    if not (bundle / "manifest.v1.json").is_file():
        bundle.parent.mkdir(parents=True, exist_ok=True)
        prepare_bundle(source.recording, [], bundle, progress=[source.progress])
    return bundle


def _adapter_entrypoint() -> Path:
    path = (
        Path(__file__).resolve().parents[3]
        / "packages"
        / "pi-spatial-adapter"
        / "dist"
        / "code-policy-main.js"
    )
    if not path.is_file():
        raise FileNotFoundError(
            "Pi adapter is not built; run npm run build in packages/pi-spatial-adapter"
        )
    return path


def _optional_model(path: Path, model: type[EvalModelT]) -> EvalModelT | None:
    return model.model_validate_json(path.read_bytes()) if path.is_file() else None
