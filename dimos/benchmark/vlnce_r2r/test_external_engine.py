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

import hashlib
import json
from pathlib import Path
from typing import Any

import pytest

from dimos.benchmark.agent_eval.case import (
    AgentCondition,
    AgentOutcome,
    AttemptRequest,
    BenchmarkInstructionTask,
    BenchmarkNativeResultValidatorRef,
    EvalCase,
    ExternalAssetFileRef,
    ExternalAssetRef,
    ExternalBenchmarkEpisodeSource,
    ExternalBenchmarkPreparationRef,
    ExternalOciImageRef,
    LiveCodePolicyInteraction,
    RuntimeBinding,
)
from dimos.benchmark.vlnce_r2r.external_engine import (
    RESULT_SCHEMA_PATH,
    ExternalBenchmarkAttemptEngine,
)
from dimos.benchmark.vlnce_r2r.preparation import (
    EpisodeBinding,
    PreparationReceipt,
    PreparedAsset,
)
from dimos.core.global_config import global_config

INSTRUCTION = "Exit the bedroom, then stop by the toilet."
IMAGE_DIGEST = "c" * 64


class FakeRuntime:
    def __init__(
        self,
        *,
        case: EvalCase,
        attempt_id: str,
        attempt_path: Path,
        success: float,
        malformed: bool = False,
        foreign: bool = False,
        missing: bool = False,
        start_error: bool = False,
        healthy: bool = True,
        close_error: bool = False,
        render_status: str | None = None,
    ) -> None:
        self.case = case
        self.attempt_id = attempt_id
        self.memory_path = attempt_path / "recording.db"
        self.result_path = attempt_path / "terminal-private" / "vlnce-result.v1.json"
        self.log_path = attempt_path / "oci-runtime.log"
        self.success = success
        self.malformed = malformed
        self.foreign = foreign
        self.missing = missing
        self.start_error = start_error
        self.healthy_value = healthy
        self.close_error = close_error
        self.render_status = render_status
        self.render_path = attempt_path / "native-render.mp4"
        self.cancelled = False
        self.closed = False

    def start(self) -> dict[str, Any]:
        if self.start_error:
            raise RuntimeError("runtime startup failed")
        self.memory_path.touch()
        self.result_path.parent.mkdir()
        self.log_path.write_text("runtime\n")
        return {"schema_version": "1.0", "ready": True}

    def healthy(self) -> bool:
        return self.healthy_value

    def result_bytes(self) -> bytes | None:
        if self.missing:
            return None
        source = self.case.source
        assert isinstance(source, ExternalBenchmarkEpisodeSource)
        if self.malformed:
            payload = b"not-json"
        else:
            document = {
                "schema_version": "vlnce-result.v1",
                "attempt_id": "foreign" if self.foreign else self.attempt_id,
                "case_id": self.case.case_id,
                "case_fingerprint": self.case.fingerprint,
                "benchmark": "vlnce_r2r",
                "dataset_revision": source.dataset_revision,
                "split": source.split,
                "episode_id": source.episode_id,
                "scene_id": source.scene_id,
                "upstream_revision": source.upstream_revision,
                "runtime_image_digest": source.preparation.image.image_digest,
                "protocol_revision": source.protocol_revision,
                "result_schema_revision": source.result_schema_revision,
                "condition_label": source.condition_label,
                "terminal_reason": "submitted",
                "duration_seconds": 1.0,
                "trajectory": {"sha256": "9" * 64, "points": 2},
                "metrics": {
                    "DISTANCE_TO_GOAL": 0.5,
                    "SUCCESS": self.success,
                    "SPL": self.success,
                    "NDTW": 0.8,
                    "PATH_LENGTH": 2.0,
                    "ORACLE_SUCCESS": self.success,
                    "STEPS_TAKEN": 3.0,
                },
                "runtime": {"engine": "fake"},
            }
            payload = json.dumps(document, sort_keys=True).encode()
        self.result_path.write_bytes(payload)
        return payload

    def public_evidence(self) -> dict[str, Any]:
        return {
            "schema_version": "vlnce-public-diagnostics.v1",
            "state": "submitted",
            "observation_count": 2,
            "accepted_control_count": 1,
            "route_submitted": True,
        }

    def cancel_motion(self) -> None:
        self.cancelled = True

    def close(self) -> None:
        self.closed = True
        if self.render_status == "completed":
            self.render_path.write_bytes(b"video")
        if self.close_error:
            raise RuntimeError("runtime cleanup failed")

    def render_evidence(self) -> dict[str, Any] | None:
        if self.render_status is None:
            return None
        return {
            "schema_version": "native-render.v1",
            "status": self.render_status,
            "diagnostic": "synthetic writer failure" if self.render_status == "failed" else None,
        }


class FakeRuntimeFactory:
    def __init__(self, success: float, **options: Any) -> None:
        self.success = success
        self.options = options
        self.runtime: FakeRuntime | None = None

    def create(self, **kwargs: Any) -> FakeRuntime:
        self.runtime = FakeRuntime(
            case=kwargs["case"],
            attempt_id=kwargs["attempt_id"],
            attempt_path=kwargs["attempt_path"],
            success=self.success,
            **self.options,
        )
        return self.runtime


class FakeAgent:
    def __init__(
        self,
        failure: BaseException | None = None,
        *,
        start_error: bool = False,
    ) -> None:
        self.failure_value = failure
        self.start_error = start_error
        self.prompt = ""
        self.aborted = False
        self.closed = False

    def start(self, **kwargs: Any) -> None:
        if self.start_error:
            raise RuntimeError("agent startup failed")
        self.prompt = kwargs["prompt"]

    def failure(self) -> BaseException | None:
        return self.failure_value

    def outcome(self, terminal_reason: str) -> AgentOutcome:
        return AgentOutcome(
            final_text="done",
            tool_call_count=2,
            terminal_reason=terminal_reason,
            agent_session_id="pi",
            interaction_session_id="policy",
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


def _request() -> AttemptRequest:
    instruction_digest = hashlib.sha256(INSTRUCTION.encode()).hexdigest()
    schema_digest = hashlib.sha256(RESULT_SCHEMA_PATH.read_bytes()).hexdigest()
    asset = ExternalAssetRef(
        asset_id="public",
        url="https://example.com/public.zip",
        archive_sha256="a" * 64,
        archive_bytes=1,
        cache_subdir="public",
        required_files=(ExternalAssetFileRef(path="scene.glb", sha256="b" * 64),),
    )
    source = ExternalBenchmarkEpisodeSource(
        upstream_revision="f" * 40,
        dataset_revision="R2R_VLNCE_v1-3",
        split="train",
        episode_id="515",
        episode_sha256="2" * 64,
        scene_id="mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",
        episode_asset_id="public",
        episode_path="train/train.json.gz",
        scene_asset_id="public",
        scene_path="scene.glb",
        navmesh_path="scene.navmesh",
        robot_profile="vlnce-cylinder-dimos-planar-v1",
        dimos_blueprint="vlnce-r2r-go2-eval",
        protocol_revision="vlnce-public-v1",
        result_schema_revision="vlnce-result.v1",
        condition_label="dimos_geometry_training_scene_development",
        preparation=ExternalBenchmarkPreparationRef(
            revision="v1",
            cache_namespace="vlnce",
            assets=(asset,),
            image=ExternalOciImageRef(
                image_name="localhost/dimos-vlnce-r2r",
                image_digest=IMAGE_DIGEST,
                build_context="dimos/benchmark/vlnce_r2r",
                build_recipe_sha256="d" * 64,
                base_image="example/base",
                base_image_digest="e" * 64,
            ),
        ),
    )
    case = EvalCase.compile(
        case_id="vlnce-test",
        source=source,
        task=BenchmarkInstructionTask(
            prompt=INSTRUCTION,
            instruction_sha256=instruction_digest,
            submission_guidance="Call submit_route() once.",
        ),
        interaction=LiveCodePolicyInteraction(driver_revision="v1", timeout_seconds=2.0),
        validator=BenchmarkNativeResultValidatorRef(
            revision="v1",
            result_filename="terminal-private/vlnce-result.v1.json",
            result_schema_sha256=schema_digest,
            identity_fields=("attempt_id", "case_id", "episode_id", "scene_id"),
        ),
    )
    return AttemptRequest(
        case=case,
        agent=AgentCondition(
            agent_id="pi", adapter="node", model="gpt-5.6-luna", thinking_level="medium"
        ),
        runtime=RuntimeBinding(runtime_id="external-vlnce-r2r"),
    )


def _preparation(tmp_path: Path) -> PreparationReceipt:
    root = tmp_path / "asset"
    root.mkdir(parents=True, exist_ok=True)
    return PreparationReceipt(
        assets={
            "public": PreparedAsset(
                asset_id="public",
                root=root,
                archive_sha256="a" * 64,
                required_file_sha256={"scene.glb": "b" * 64},
                cache_hit=True,
            )
        },
        episode=EpisodeBinding(episode={}, episode_sha256="2" * 64),
    )


def _run(
    tmp_path: Path,
    *,
    success: float = 1.0,
    malformed: bool = False,
    foreign: bool = False,
    agent_failure: BaseException | None = None,
    runtime_missing: bool = False,
    runtime_start_error: bool = False,
    runtime_healthy: bool = True,
    runtime_close_error: bool = False,
    render_status: str | None = None,
    agent_start_error: bool = False,
    wait: Any = None,
):
    runtime_factory = FakeRuntimeFactory(
        success,
        malformed=malformed,
        foreign=foreign,
        missing=runtime_missing,
        start_error=runtime_start_error,
        healthy=runtime_healthy,
        close_error=runtime_close_error,
        render_status=render_status,
    )
    agent = FakeAgent(agent_failure, start_error=agent_start_error)
    clock = FakeClock()
    result = ExternalBenchmarkAttemptEngine(
        request=_request(),
        output_root=tmp_path / "attempts",
        preparation=_preparation(tmp_path),
        image_id=f"sha256:{IMAGE_DIGEST}",
        runtime_factory=runtime_factory,
        agent_factory=FakeAgentFactory(agent),
        monotonic=clock.monotonic,
        wait=wait or clock.wait,
    ).run()
    return result, runtime_factory.runtime, agent


class FakeClock:
    def __init__(self) -> None:
        self.value = 0.0

    def monotonic(self) -> float:
        return self.value

    def wait(self, seconds: float) -> None:
        self.value += seconds


def test_official_success_is_the_only_pass_signal_and_native_bytes_are_unchanged(
    tmp_path: Path,
) -> None:
    result, runtime, agent = _run(tmp_path)

    assert (result.outcome.attempt_status, result.outcome.task_result) == ("completed", "passed")
    assert runtime is not None and runtime.cancelled and runtime.closed
    assert agent.aborted and agent.closed
    assert INSTRUCTION in agent.prompt and "submit_route()" in agent.prompt
    retained = result.attempt_path / "terminal-private" / "vlnce-result.v1.json"
    assert retained.read_bytes() == runtime.result_path.read_bytes()
    public_terminal = json.loads(
        (result.attempt_path / "runtime-public-terminal.v1.json").read_text()
    )
    assert public_terminal["route_submitted"] is True
    assert "metrics" not in public_terminal


def test_official_failure_is_completed_task_failure(tmp_path: Path) -> None:
    result, _, _ = _run(tmp_path, success=0.0)
    assert (result.outcome.attempt_status, result.outcome.task_result) == ("completed", "failed")


def test_native_render_is_registered_without_becoming_a_score_signal(tmp_path: Path) -> None:
    completed, _, _ = _run(tmp_path / "completed", render_status="completed")
    failed, _, _ = _run(tmp_path / "failed", render_status="failed")

    assert completed.outcome.task_result == "passed"
    assert (completed.attempt_path / "native-render.mp4").read_bytes() == b"video"
    completed_metadata = json.loads((completed.attempt_path / "native-render.v1.json").read_text())
    assert completed_metadata["status"] == "completed"
    assert failed.outcome.task_result == "passed"
    failed_metadata = json.loads((failed.attempt_path / "native-render.v1.json").read_text())
    assert failed_metadata["status"] == "failed"


def test_malformed_foreign_or_agent_failure_is_not_evaluated(tmp_path: Path) -> None:
    variants = (
        {"malformed": True},
        {"foreign": True},
        {"agent_failure": RuntimeError("agent crashed")},
    )
    for index, kwargs in enumerate(variants):
        result, _, _ = _run(tmp_path / str(index), **kwargs)
        assert (result.outcome.attempt_status, result.outcome.task_result) == (
            "failed",
            "not_evaluated",
        )


def test_startup_runtime_loss_and_missing_result_are_not_evaluated(tmp_path: Path) -> None:
    variants = (
        {"runtime_start_error": True},
        {"agent_start_error": True},
        {"runtime_missing": True, "runtime_healthy": False},
        {"runtime_missing": True},
    )
    for index, kwargs in enumerate(variants):
        result, runtime, agent = _run(tmp_path / str(index), **kwargs)
        assert result.outcome.task_result == "not_evaluated"
        assert runtime is not None and runtime.closed
        if kwargs.get("runtime_start_error"):
            assert agent.prompt == ""


def test_interruption_and_cleanup_failure_preserve_primary_classification(tmp_path: Path) -> None:
    def interrupt(_seconds: float) -> None:
        raise KeyboardInterrupt

    interrupted, _, _ = _run(
        tmp_path / "interrupted",
        runtime_missing=True,
        wait=interrupt,
    )
    assert interrupted.outcome.reason == "user interrupted"
    assert interrupted.outcome.task_result == "not_evaluated"

    completed, _, _ = _run(tmp_path / "cleanup", runtime_close_error=True)
    assert completed.outcome.task_result == "passed"
    cleanup = json.loads((completed.attempt_path / "cleanup.v1.json").read_text())
    assert cleanup["diagnostics"][0]["resource"] == "runtime"


def test_repeated_runs_never_overwrite_an_attempt(tmp_path: Path) -> None:
    first, _, _ = _run(tmp_path)
    second, _, _ = _run(tmp_path)

    assert first.attempt_path != second.attempt_path
    assert first.attempt_path.is_dir() and second.attempt_path.is_dir()


@pytest.mark.parametrize("viewer", ["none", "rerun"])
def test_viewer_choice_does_not_change_native_scoring_or_cleanup(
    tmp_path: Path, viewer: str
) -> None:
    original = global_config.model_dump()
    try:
        global_config.update(viewer=viewer)
        result, runtime, agent = _run(tmp_path)
    finally:
        global_config.update(**original)

    assert (result.outcome.attempt_status, result.outcome.task_result) == (
        "completed",
        "passed",
    )
    assert runtime is not None and runtime.cancelled and runtime.closed
    assert agent.aborted and agent.closed
    cleanup = json.loads((result.attempt_path / "cleanup.v1.json").read_text())
    assert cleanup["diagnostics"] == []
