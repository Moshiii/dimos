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

"""Canonical source/task/interaction/validator contracts for agent evaluation."""

from __future__ import annotations

import hashlib
import math
from pathlib import PurePosixPath
from typing import Annotated, Literal
from urllib.parse import urlsplit

from pydantic import Field, JsonValue, model_validator

from dimos.benchmark.agent_eval.base import BaseEvalModel
from dimos.benchmark.spatial.utilities import canonical_json

NonEmpty = Annotated[str, Field(min_length=1)]
Sha256 = Annotated[str, Field(pattern=r"^[0-9a-f]{64}$")]
GitSha = Annotated[str, Field(pattern=r"^[0-9a-f]{40}$")]
NormalizedProgress = Annotated[float, Field(ge=0.0, le=1.0, allow_inf_nan=False)]
PositiveFinite = Annotated[float, Field(gt=0.0, allow_inf_nan=False)]
PositiveInteger = Annotated[int, Field(gt=0)]


class SourcePreparationRef(BaseEvalModel):
    """Versioned public recipe used to establish a simulator task's start state."""

    kind: Literal["apartment_spatial_memory"] = "apartment_spatial_memory"
    revision: NonEmpty
    exploration_route: tuple[tuple[float, float], ...] = Field(min_length=1)
    final_start_pose: tuple[float, float, float]
    step_timeout_seconds: PositiveFinite
    odometry_timeout_seconds: PositiveFinite
    start_tolerance_metres: PositiveFinite

    @model_validator(mode="after")
    def coordinates_are_finite(self) -> SourcePreparationRef:
        coordinates = (
            coordinate
            for point in (*self.exploration_route, self.final_start_pose)
            for coordinate in point
        )
        if not all(math.isfinite(coordinate) for coordinate in coordinates):
            raise ValueError("source preparation coordinates must be finite")
        return self


class FrozenRecordingSource(BaseEvalModel):
    kind: Literal["frozen_memory"] = "frozen_memory"
    recording: NonEmpty
    progress: NormalizedProgress
    bundle_manifest_sha256: Sha256 | None = None

    @model_validator(mode="after")
    def progress_is_finite(self) -> FrozenRecordingSource:
        if not math.isfinite(self.progress):
            raise ValueError("recording progress must be finite")
        return self


class LiveDimosSource(BaseEvalModel):
    kind: Literal["live_dimos"] = "live_dimos"
    runtime: NonEmpty
    scene: NonEmpty | None = None
    episode: NonEmpty | None = None


class SimulatorSceneSource(BaseEvalModel):
    kind: Literal["simulator_scene"] = "simulator_scene"
    scene: NonEmpty
    simulation_provider: NonEmpty
    robot: NonEmpty
    dimos_blueprint: NonEmpty
    preparation: SourcePreparationRef | None = None


class ExternalAssetFileRef(BaseEvalModel):
    path: NonEmpty
    sha256: Sha256

    @model_validator(mode="after")
    def path_is_relative(self) -> ExternalAssetFileRef:
        _validate_relative_path(self.path, label="asset path")
        return self


class ExternalAssetRef(BaseEvalModel):
    asset_id: NonEmpty
    url: NonEmpty
    archive_sha256: Sha256
    archive_bytes: PositiveInteger
    cache_subdir: NonEmpty
    required_files: tuple[ExternalAssetFileRef, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def fields_are_safe_and_unique(self) -> ExternalAssetRef:
        parsed = urlsplit(self.url)
        if parsed.scheme != "https" or not parsed.netloc or parsed.username or parsed.password:
            raise ValueError("external asset URL must be an unauthenticated HTTPS URL")
        _validate_relative_path(self.cache_subdir, label="asset cache_subdir")
        paths = [item.path for item in self.required_files]
        if len(paths) != len(set(paths)):
            raise ValueError("external asset required file paths must be unique")
        return self


class ExternalOciImageRef(BaseEvalModel):
    image_name: NonEmpty
    image_digest: Sha256
    build_context: NonEmpty
    build_recipe_sha256: Sha256
    base_image: NonEmpty
    base_image_digest: Sha256

    @model_validator(mode="after")
    def build_context_is_relative(self) -> ExternalOciImageRef:
        _validate_relative_path(self.build_context, label="OCI build_context")
        return self


class ExternalBenchmarkPreparationRef(BaseEvalModel):
    kind: Literal["vlnce_public_assets"] = "vlnce_public_assets"
    revision: NonEmpty
    cache_namespace: NonEmpty
    assets: tuple[ExternalAssetRef, ...] = Field(min_length=1)
    image: ExternalOciImageRef

    @model_validator(mode="after")
    def assets_are_unique(self) -> ExternalBenchmarkPreparationRef:
        asset_ids = [asset.asset_id for asset in self.assets]
        if len(asset_ids) != len(set(asset_ids)):
            raise ValueError("external preparation asset ids must be unique")
        _validate_relative_path(self.cache_namespace, label="cache namespace")
        return self


class ExternalBenchmarkEpisodeSource(BaseEvalModel):
    kind: Literal["external_benchmark_episode"] = "external_benchmark_episode"
    benchmark: Literal["vlnce_r2r"] = "vlnce_r2r"
    upstream_revision: GitSha
    dataset_revision: NonEmpty
    split: NonEmpty
    episode_id: NonEmpty
    episode_sha256: Sha256
    scene_id: NonEmpty
    episode_asset_id: NonEmpty
    episode_path: NonEmpty
    scene_asset_id: NonEmpty
    scene_path: NonEmpty
    navmesh_path: NonEmpty
    robot_profile: NonEmpty
    dimos_blueprint: NonEmpty
    protocol_revision: NonEmpty
    result_schema_revision: NonEmpty
    condition_label: NonEmpty
    preparation: ExternalBenchmarkPreparationRef

    @model_validator(mode="after")
    def asset_references_are_resolvable(self) -> ExternalBenchmarkEpisodeSource:
        assets = {asset.asset_id: asset for asset in self.preparation.assets}
        for asset_id in (self.episode_asset_id, self.scene_asset_id):
            if asset_id not in assets:
                raise ValueError(f"external source references unknown asset {asset_id!r}")
        for value, label in (
            (self.episode_path, "episode_path"),
            (self.scene_path, "scene_path"),
            (self.navmesh_path, "navmesh_path"),
        ):
            _validate_relative_path(value, label=label)
        return self


SourceSpec = Annotated[
    FrozenRecordingSource | LiveDimosSource | SimulatorSceneSource | ExternalBenchmarkEpisodeSource,
    Field(discriminator="kind"),
]


class IntegerQuestionTask(BaseEvalModel):
    kind: Literal["integer_question"] = "integer_question"
    prompt: NonEmpty
    answer_marker: Literal["ANSWER:"] = "ANSWER:"


class EmbodiedInstructionTask(BaseEvalModel):
    kind: Literal["embodied_instruction"] = "embodied_instruction"
    prompt: NonEmpty


class BenchmarkInstructionTask(BaseEvalModel):
    kind: Literal["benchmark_instruction"] = "benchmark_instruction"
    prompt: NonEmpty
    instruction_sha256: Sha256
    submission_guidance: NonEmpty

    @model_validator(mode="after")
    def instruction_digest_matches(self) -> BenchmarkInstructionTask:
        digest = hashlib.sha256(self.prompt.encode()).hexdigest()
        if digest != self.instruction_sha256:
            raise ValueError("benchmark instruction digest does not match prompt")
        return self


TaskSpec = Annotated[
    IntegerQuestionTask | EmbodiedInstructionTask | BenchmarkInstructionTask,
    Field(discriminator="kind"),
]


class FrozenCodePolicyInteraction(BaseEvalModel):
    kind: Literal["frozen_code_policy"] = "frozen_code_policy"
    driver_revision: NonEmpty
    session_lifetime: Literal["one_attempt"] = "one_attempt"


class LiveCodePolicyInteraction(BaseEvalModel):
    kind: Literal["live_code_policy"] = "live_code_policy"
    driver_revision: NonEmpty
    session_lifetime: Literal["one_attempt"] = "one_attempt"
    completion: Literal["agent_or_native_terminal"] = "agent_or_native_terminal"
    timeout_seconds: PositiveFinite


InteractionSpec = Annotated[
    FrozenCodePolicyInteraction | LiveCodePolicyInteraction,
    Field(discriminator="kind"),
]


class ExactIntegerValidatorRef(BaseEvalModel):
    kind: Literal["exact_integer"] = "exact_integer"
    revision: NonEmpty
    private_path: NonEmpty
    private_sha256: Sha256

    @model_validator(mode="after")
    def private_path_is_relative(self) -> ExactIntegerValidatorRef:
        path = PurePosixPath(self.private_path)
        if path.is_absolute() or not path.parts or ".." in path.parts:
            raise ValueError("validator private_path must be a safe relative path")
        return self


class NativeValidatorRef(BaseEvalModel):
    kind: Literal["native"] = "native"
    revision: NonEmpty
    contract_sha256: Sha256


class PeriodicGoalValidatorRef(BaseEvalModel):
    kind: Literal["periodic_goal"] = "periodic_goal"
    revision: NonEmpty
    private_path: NonEmpty
    private_sha256: Sha256

    @model_validator(mode="after")
    def private_path_is_relative(self) -> PeriodicGoalValidatorRef:
        _validate_private_path(self.private_path)
        return self


class SemanticObjectProximityGoal(BaseEvalModel):
    kind: Literal["semantic_object_proximity"] = "semantic_object_proximity"
    semantic_query: NonEmpty
    maximum_distance_metres: PositiveFinite
    poll_interval_seconds: PositiveFinite


class BenchmarkNativeResultValidatorRef(BaseEvalModel):
    kind: Literal["benchmark_native_result"] = "benchmark_native_result"
    revision: NonEmpty
    result_filename: NonEmpty
    result_schema_sha256: Sha256
    success_metric: Literal["SUCCESS"] = "SUCCESS"
    identity_fields: tuple[NonEmpty, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def result_contract_is_safe(self) -> BenchmarkNativeResultValidatorRef:
        _validate_relative_path(self.result_filename, label="benchmark result filename")
        if len(self.identity_fields) != len(set(self.identity_fields)):
            raise ValueError("benchmark result identity fields must be unique")
        return self


ValidatorRef = Annotated[
    ExactIntegerValidatorRef
    | NativeValidatorRef
    | PeriodicGoalValidatorRef
    | BenchmarkNativeResultValidatorRef,
    Field(discriminator="kind"),
]


class PublicEvalCase(BaseEvalModel):
    """Agent-safe case projection; it intentionally cannot carry a validator."""

    case_id: NonEmpty
    source: SourceSpec
    task: TaskSpec
    interaction: InteractionSpec


class EvalCase(BaseEvalModel):
    """Compiled private case binding all four semantic contracts."""

    case_id: NonEmpty
    source: SourceSpec
    task: TaskSpec
    interaction: InteractionSpec
    validator: ValidatorRef
    fingerprint: Sha256

    @model_validator(mode="after")
    def contracts_are_compatible(self) -> EvalCase:
        external = isinstance(self.source, ExternalBenchmarkEpisodeSource)
        benchmark_task = isinstance(self.task, BenchmarkInstructionTask)
        benchmark_validator = isinstance(self.validator, BenchmarkNativeResultValidatorRef)
        if external:
            if not benchmark_task:
                raise ValueError("external benchmark source requires a benchmark instruction task")
            if not isinstance(self.interaction, LiveCodePolicyInteraction):
                raise ValueError("external benchmark source requires live CodePolicy interaction")
            if not benchmark_validator:
                raise ValueError(
                    "external benchmark source requires benchmark-native result validation"
                )
        elif benchmark_task or benchmark_validator:
            raise ValueError(
                "benchmark instruction and native result validator require an external source"
            )
        return self

    @classmethod
    def compile(
        cls,
        *,
        case_id: str,
        source: SourceSpec,
        task: TaskSpec,
        interaction: InteractionSpec,
        validator: ValidatorRef,
    ) -> EvalCase:
        payload = _case_payload(case_id, source, task, interaction, validator)
        return cls(
            case_id=case_id,
            source=source,
            task=task,
            interaction=interaction,
            validator=validator,
            fingerprint=hashlib.sha256(canonical_json(payload)).hexdigest(),
        )

    @model_validator(mode="after")
    def fingerprint_matches_payload(self) -> EvalCase:
        payload = _case_payload(
            self.case_id,
            self.source,
            self.task,
            self.interaction,
            self.validator,
        )
        expected = hashlib.sha256(canonical_json(payload)).hexdigest()
        if self.fingerprint != expected:
            raise ValueError("evaluation case fingerprint does not match its contracts")
        return self

    def public_projection(self) -> PublicEvalCase:
        return PublicEvalCase(
            case_id=self.case_id,
            source=self.source,
            task=self.task,
            interaction=self.interaction,
        )


class AgentCondition(BaseEvalModel):
    agent_id: NonEmpty
    adapter: NonEmpty
    model: NonEmpty
    thinking_level: NonEmpty


class RuntimeBinding(BaseEvalModel):
    runtime_id: NonEmpty
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class AttemptRequest(BaseEvalModel):
    case: EvalCase
    agent: AgentCondition
    runtime: RuntimeBinding
    seed: int | None = None


class AgentOutcome(BaseEvalModel):
    final_text: str
    tool_call_count: int = Field(ge=0)
    terminal_reason: NonEmpty
    agent_session_id: NonEmpty | None = None
    interaction_session_id: NonEmpty | None = None


class Prediction(BaseEvalModel):
    case_id: NonEmpty
    attempt_id: NonEmpty
    agent_session_id: NonEmpty
    interaction_session_id: NonEmpty
    parser_revision: NonEmpty
    final_text: str
    status: Literal["parsed", "invalid"]
    integer_answer: int | None = None
    diagnostic: NonEmpty | None = None

    @model_validator(mode="after")
    def answer_matches_status(self) -> Prediction:
        if self.status == "parsed" and (self.integer_answer is None or self.diagnostic is not None):
            raise ValueError("parsed prediction requires only integer_answer")
        if self.status == "invalid" and (
            self.integer_answer is not None or self.diagnostic is None
        ):
            raise ValueError("invalid prediction requires only diagnostic")
        return self


class PrivateScore(BaseEvalModel):
    case_id: NonEmpty
    attempt_id: NonEmpty
    validator_revision: NonEmpty
    passed: bool
    prediction_status: Literal["parsed", "invalid", "native"]


class EvalOutcome(BaseEvalModel):
    attempt_id: NonEmpty
    attempt_status: Literal["completed", "failed"]
    task_result: Literal["passed", "failed", "not_evaluated"]
    reason: NonEmpty

    @model_validator(mode="after")
    def states_are_consistent(self) -> EvalOutcome:
        if self.attempt_status == "failed" and self.task_result != "not_evaluated":
            raise ValueError("failed infrastructure cannot claim a task result")
        if self.attempt_status == "completed" and self.task_result == "not_evaluated":
            raise ValueError("completed evaluation must report passed or failed")
        return self


def _case_payload(
    case_id: str,
    source: SourceSpec,
    task: TaskSpec,
    interaction: InteractionSpec,
    validator: ValidatorRef,
) -> dict[str, JsonValue]:
    return {
        "case_id": case_id,
        "source": source.model_dump(mode="json"),
        "task": task.model_dump(mode="json"),
        "interaction": interaction.model_dump(mode="json"),
        "validator": validator.model_dump(mode="json"),
    }


def _validate_private_path(value: str) -> None:
    _validate_relative_path(value, label="validator private_path")


def _validate_relative_path(value: str, *, label: str) -> None:
    path = PurePosixPath(value)
    if path.is_absolute() or not path.parts or ".." in path.parts or path == PurePosixPath("."):
        raise ValueError(f"{label} must be a safe relative path")
