# Copyright 2026 Dimensional Inc.
"""Public and private contracts for standalone single-frame VQA evaluation."""

from __future__ import annotations

from pathlib import PurePosixPath
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

NonEmpty = Annotated[str, Field(min_length=1)]


class BaseEvaluationModel(BaseModel):
    """Strict immutable base for VQA evaluator records."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    schema_version: Literal["1.0"] = "1.0"


class EvaluationCase(BaseEvaluationModel):
    """Public metadata shared by every evaluator case."""

    case_id: NonEmpty


class SingleFrameVqaEvaluationCase(EvaluationCase):
    """Public image, question, and closed answer policy for one VQA case."""

    kind: Literal["single_frame_vqa"] = "single_frame_vqa"
    image_path: NonEmpty
    question: NonEmpty
    allowed_answers: tuple[NonEmpty, ...] = Field(min_length=2)
    answer_marker: Literal["ANSWER:"] = "ANSWER:"

    @model_validator(mode="after")
    def validate_public_inputs(self) -> SingleFrameVqaEvaluationCase:
        path = PurePosixPath(self.image_path)
        if path.is_absolute() or not path.parts or ".." in path.parts:
            raise ValueError("image_path must be a safe relative path")
        normalized = [answer.lower() for answer in self.allowed_answers]
        if len(set(normalized)) != len(normalized):
            raise ValueError("allowed_answers must be unique ignoring case")
        return self


class EvaluationOracle(BaseEvaluationModel):
    """Private metadata shared by evaluator oracles."""

    case_id: NonEmpty
    validator_revision: NonEmpty


class SingleFrameVqaOracle(EvaluationOracle):
    """Private expected answer and grounding provenance for one VQA case."""

    kind: Literal["single_frame_vqa"] = "single_frame_vqa"
    expected_answer: NonEmpty
    ground_truth_revision: NonEmpty


class LangChainVisionEvaluationConfig(BaseEvaluationModel):
    """Model configuration for the no-tools LangChain vision evaluator."""

    model: NonEmpty = "gpt-5.6-luna"
    thinking_level: Literal["medium"] = "medium"


class EvaluationResult(BaseEvaluationModel):
    """Common persisted result fields for one evaluated case."""

    case_id: NonEmpty
    model: NonEmpty
    raw_response: str = ""
    normalized_answer: str | None = None
    passed: bool | None = None
    infra_error: str | None = None


class SingleFrameVqaEvaluationResult(EvaluationResult):
    """Result structure for one standalone single-frame VQA evaluation."""

    kind: Literal["single_frame_vqa"] = "single_frame_vqa"
    validator_revision: NonEmpty | None = None
