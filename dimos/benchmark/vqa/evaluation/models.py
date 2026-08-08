# Copyright 2026 Dimensional Inc.
"""Public and private contracts for standalone single-frame VQA evaluation."""

from __future__ import annotations

import math
from pathlib import PurePosixPath
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

NonEmpty = Annotated[str, Field(min_length=1)]


class BaseEvaluationModel(BaseModel):
    """Strict immutable base for VQA evaluator records."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    schema_version: Literal["1.0"] = "1.0"


class EvaluationCase(BaseEvaluationModel):
    """Public metadata shared by every evaluator case."""

    case_id: NonEmpty


class SingleFrameVqaEvaluationCase(EvaluationCase):
    """Public image, question, and answer policy for one VQA case."""

    kind: Literal["single_frame_vqa"] = "single_frame_vqa"
    image_path: NonEmpty
    question: NonEmpty
    answer_kind: Literal["choice", "numeric"] = "choice"
    allowed_answers: tuple[NonEmpty, ...] | None = None
    unit: NonEmpty | None = None
    tolerance: float | None = Field(default=None, ge=0, allow_inf_nan=False)
    answer_marker: Literal["ANSWER:"] = "ANSWER:"

    @field_validator("allowed_answers", mode="before")
    @classmethod
    def deserialize_allowed_answers(cls, value: Any) -> Any:
        """Accept JSON arrays while retaining immutable tuple values in Python."""
        return tuple(value) if isinstance(value, list) else value

    @model_validator(mode="after")
    def validate_public_inputs(self) -> SingleFrameVqaEvaluationCase:
        path = PurePosixPath(self.image_path)
        if path.is_absolute() or not path.parts or ".." in path.parts:
            raise ValueError("image_path must be a safe relative path")
        if self.answer_kind == "choice":
            if self.allowed_answers is None or len(self.allowed_answers) < 2:
                raise ValueError("choice cases require at least two allowed_answers")
            if self.unit is not None or self.tolerance is not None:
                raise ValueError("choice cases cannot define numeric unit or tolerance")
            normalized = [answer.lower() for answer in self.allowed_answers]
            if len(set(normalized)) != len(normalized):
                raise ValueError("allowed_answers must be unique ignoring case")
        elif self.allowed_answers is not None:
            raise ValueError("numeric cases cannot define allowed_answers")
        elif self.unit is None or self.tolerance is None:
            raise ValueError("numeric cases require unit and tolerance")
        return self


class EvaluationOracle(BaseEvaluationModel):
    """Private metadata shared by evaluator oracles."""

    case_id: NonEmpty
    validator_revision: NonEmpty


class SingleFrameVqaOracle(EvaluationOracle):
    """Private expected answer and grounding provenance for one VQA case."""

    kind: Literal["single_frame_vqa"] = "single_frame_vqa"
    expected_answer: NonEmpty | float
    tolerance: float | None = Field(default=None, ge=0, allow_inf_nan=False)
    ground_truth_revision: NonEmpty

    @model_validator(mode="after")
    def validate_expected_answer(self) -> SingleFrameVqaOracle:
        if isinstance(self.expected_answer, float):
            if not math.isfinite(self.expected_answer):
                raise ValueError("numeric expected_answer must be finite")
            if self.tolerance is None:
                raise ValueError("numeric expected_answer requires tolerance provenance")
        elif self.tolerance is not None:
            raise ValueError("choice expected_answer cannot define tolerance")
        return self


class LangChainVisionEvaluationConfig(BaseEvaluationModel):
    """Model configuration for the no-tools LangChain vision evaluator."""

    model: NonEmpty = "gpt-5.6-luna"
    thinking_level: Literal["medium"] = "medium"


class EvaluationResult(BaseEvaluationModel):
    """Common persisted result fields for one evaluated case."""

    case_id: NonEmpty
    model: NonEmpty
    raw_response: str = ""
    normalized_answer: str | float | None = None
    passed: bool | None = None
    infra_error: str | None = None


class SingleFrameVqaEvaluationResult(EvaluationResult):
    """Result structure for one standalone single-frame VQA evaluation."""

    kind: Literal["single_frame_vqa"] = "single_frame_vqa"
    validator_revision: NonEmpty | None = None

    @model_validator(mode="after")
    def validate_normalized_answer(self) -> SingleFrameVqaEvaluationResult:
        if isinstance(self.normalized_answer, float) and not math.isfinite(self.normalized_answer):
            raise ValueError("numeric normalized_answer must be finite")
        return self


class EvaluationDatasetIdentity(BaseEvaluationModel):
    """Public identity of the dataset evaluated by a run."""

    resolved_path: NonEmpty
    public_digest: NonEmpty
    case_count: int = Field(ge=0)
    public_manifest_digest: NonEmpty | None = None


class EvaluationRunConfig(BaseEvaluationModel):
    """Configuration that must remain stable when resuming a run."""

    model: NonEmpty
    langchain: LangChainVisionEvaluationConfig
    workers: int = Field(ge=1)


class EvaluationRunManifest(BaseEvaluationModel):
    """Public lifecycle record for an evaluator run."""

    run_id: NonEmpty
    status: Literal["running", "completed", "failed"]
    created_at: NonEmpty
    updated_at: NonEmpty
    completed_at: NonEmpty | None = None
    dataset: EvaluationDatasetIdentity
    config: EvaluationRunConfig


class EvaluationProgressEvent(BaseEvaluationModel):
    """Base public event emitted by an evaluator run."""

    run_id: NonEmpty
    timestamp: NonEmpty


class RunStartedEvent(EvaluationProgressEvent):
    event_type: Literal["run_started"] = "run_started"


class CaseStartedEvent(EvaluationProgressEvent):
    event_type: Literal["case_started"] = "case_started"
    index: int = Field(ge=0)
    case_id: NonEmpty


class CaseCompletedEvent(EvaluationProgressEvent):
    event_type: Literal["case_completed"] = "case_completed"
    index: int = Field(ge=0)
    case_id: NonEmpty
    result: SingleFrameVqaEvaluationResult


class InfrastructureFailureEvent(EvaluationProgressEvent):
    event_type: Literal["infrastructure_failure"] = "infrastructure_failure"
    index: int | None = Field(default=None, ge=0)
    case_id: NonEmpty | None = None
    error: NonEmpty


class RunCompletedEvent(EvaluationProgressEvent):
    event_type: Literal["run_completed"] = "run_completed"
    summary: dict[str, int | float | None]


class RunFailedEvent(EvaluationProgressEvent):
    event_type: Literal["run_failed"] = "run_failed"
    error: NonEmpty


class EvaluationCheckpoint(BaseEvaluationModel):
    """Terminal, public-only result checkpoint for one inventory index."""

    index: int = Field(ge=0)
    case_id: NonEmpty
    result: SingleFrameVqaEvaluationResult
