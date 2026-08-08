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

from pydantic import ValidationError
import pytest

from dimos.benchmark.agent_eval.case import (
    AgentCondition,
    AttemptRequest,
    BenchmarkInstructionTask,
    BenchmarkNativeResultValidatorRef,
    EmbodiedInstructionTask,
    EvalCase,
    ExternalAssetFileRef,
    ExternalAssetRef,
    ExternalBenchmarkEpisodeSource,
    ExternalBenchmarkPreparationRef,
    ExternalOciImageRef,
    LiveCodePolicyInteraction,
    PeriodicGoalValidatorRef,
    RuntimeBinding,
)

INSTRUCTION = "Exit the bedroom, enter the bathroom, wait at the toilet. "


def _asset(asset_id: str = "scene") -> ExternalAssetRef:
    return ExternalAssetRef(
        asset_id=asset_id,
        url=f"https://example.com/{asset_id}.zip",
        archive_sha256="a" * 64,
        archive_bytes=123,
        cache_subdir=f"assets/{asset_id}",
        required_files=(ExternalAssetFileRef(path="17DRP5sb8fy/17DRP5sb8fy.glb", sha256="b" * 64),),
    )


def _image() -> ExternalOciImageRef:
    return ExternalOciImageRef(
        image_name="localhost/dimos-vlnce-r2r",
        image_digest="c" * 64,
        build_context="dimos/benchmark/vlnce_r2r",
        build_recipe_sha256="d" * 64,
        base_image="docker.io/continuumio/miniconda3",
        base_image_digest="e" * 64,
    )


def _source(**updates: object) -> ExternalBenchmarkEpisodeSource:
    values = {
        "upstream_revision": "f" * 40,
        "dataset_revision": "R2R_VLNCE_v1-3",
        "split": "train",
        "episode_id": "515",
        "episode_sha256": "2" * 64,
        "scene_id": "mp3d/17DRP5sb8fy/17DRP5sb8fy.glb",
        "episode_asset_id": "scene",
        "episode_path": "R2R_VLNCE_v1-3/train/train.json.gz",
        "scene_asset_id": "scene",
        "scene_path": "17DRP5sb8fy/17DRP5sb8fy.glb",
        "navmesh_path": "17DRP5sb8fy/17DRP5sb8fy.navmesh",
        "robot_profile": "vlnce-cylinder-dimos-planar-v1",
        "dimos_blueprint": "vlnce-r2r-go2-eval",
        "protocol_revision": "vlnce-public-v1",
        "result_schema_revision": "vlnce-result-v1",
        "condition_label": "dimos_geometry_training_scene_development",
        "preparation": ExternalBenchmarkPreparationRef(
            revision="v1",
            cache_namespace="agent_eval/vlnce_r2r",
            assets=(_asset(),),
            image=_image(),
        ),
    }
    values.update(updates)
    return ExternalBenchmarkEpisodeSource.model_validate(values)


def _task(prompt: str = INSTRUCTION) -> BenchmarkInstructionTask:
    return BenchmarkInstructionTask(
        prompt=prompt,
        instruction_sha256=hashlib.sha256(prompt.encode()).hexdigest(),
        submission_guidance="Call submit_route() once when the described route is complete.",
    )


def _validator() -> BenchmarkNativeResultValidatorRef:
    return BenchmarkNativeResultValidatorRef(
        revision="v1",
        result_filename="vlnce-result.v1.json",
        result_schema_sha256="1" * 64,
        identity_fields=("attempt_id", "case_id", "episode_id", "scene_id"),
    )


def _case(source: ExternalBenchmarkEpisodeSource | None = None) -> EvalCase:
    return EvalCase.compile(
        case_id="vlnce-r2r-17DRP5sb8fy-episode-515",
        source=source or _source(),
        task=_task(),
        interaction=LiveCodePolicyInteraction(
            driver_revision="external-benchmark-v1", timeout_seconds=60.0
        ),
        validator=_validator(),
    )


def test_external_case_binds_runtime_assets_protocol_and_result_contract() -> None:
    case = _case()
    changed = _source(protocol_revision="vlnce-public-v2")

    assert case.fingerprint != _case(changed).fingerprint
    public = case.public_projection().model_dump(mode="json")
    assert public["source"]["episode_id"] == "515"
    assert public["task"]["prompt"] == INSTRUCTION
    assert "validator" not in public


def test_external_case_runtime_binding_does_not_override_case_identity() -> None:
    case = _case()
    request = AttemptRequest(
        case=case,
        agent=AgentCondition(
            agent_id="pi", adapter="pi-node", model="gpt-5.6-luna", thinking_level="medium"
        ),
        runtime=RuntimeBinding(runtime_id="local", parameters={"viewer": "none"}),
    )

    assert request.case.fingerprint == case.fingerprint


@pytest.mark.parametrize(
    "field",
    [
        "upstream_revision",
        "dataset_revision",
        "split",
        "episode_id",
        "episode_sha256",
        "scene_id",
        "episode_asset_id",
        "episode_path",
        "scene_asset_id",
        "scene_path",
        "navmesh_path",
        "robot_profile",
        "dimos_blueprint",
        "protocol_revision",
        "result_schema_revision",
        "condition_label",
        "preparation",
    ],
)
def test_external_source_rejects_missing_required_field(field: str) -> None:
    values = _source().model_dump()
    del values[field]

    with pytest.raises(ValidationError):
        ExternalBenchmarkEpisodeSource.model_validate(values)


@pytest.mark.parametrize(
    ("updates", "match"),
    [
        ({"url": "http://example.com/scene.zip"}, "HTTPS"),
        ({"url": "https://user:secret@example.com/scene.zip"}, "unauthenticated"),
        ({"cache_subdir": "../scene"}, "safe relative"),
    ],
)
def test_external_asset_rejects_unsafe_location(updates: dict[str, object], match: str) -> None:
    values = _asset().model_dump()
    values.update(updates)

    with pytest.raises(ValidationError, match=match):
        ExternalAssetRef.model_validate(values)


def test_benchmark_task_rejects_mismatched_instruction_digest() -> None:
    with pytest.raises(ValidationError, match="instruction digest"):
        BenchmarkInstructionTask(
            prompt=INSTRUCTION,
            instruction_sha256="0" * 64,
            submission_guidance="Call submit_route().",
        )


def test_external_source_rejects_unknown_or_unsafe_asset_reference() -> None:
    with pytest.raises(ValidationError, match="unknown asset"):
        _source(scene_asset_id="missing")

    with pytest.raises(ValidationError, match="safe relative"):
        _source(scene_path="../scene.glb")


def test_external_source_rejects_incompatible_task_and_validator() -> None:
    with pytest.raises(ValidationError, match="benchmark instruction"):
        EvalCase.compile(
            case_id="bad",
            source=_source(),
            task=EmbodiedInstructionTask(prompt=INSTRUCTION),
            interaction=LiveCodePolicyInteraction(driver_revision="v1", timeout_seconds=1.0),
            validator=_validator(),
        )

    with pytest.raises(ValidationError, match="benchmark-native"):
        EvalCase.compile(
            case_id="bad",
            source=_source(),
            task=_task(),
            interaction=LiveCodePolicyInteraction(driver_revision="v1", timeout_seconds=1.0),
            validator=PeriodicGoalValidatorRef(
                revision="v1", private_path="private/goal.json", private_sha256="a" * 64
            ),
        )


def test_native_result_contract_rejects_unsafe_path_and_duplicate_identities() -> None:
    values = _validator().model_dump()
    values["result_filename"] = "../result.json"
    with pytest.raises(ValidationError, match="safe relative"):
        BenchmarkNativeResultValidatorRef.model_validate(values)

    values = _validator().model_dump()
    values["identity_fields"] = ("case_id", "case_id")
    with pytest.raises(ValidationError, match="unique"):
        BenchmarkNativeResultValidatorRef.model_validate(values)
