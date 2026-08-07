# Copyright 2026 Dimensional Inc.

from __future__ import annotations

import pytest

from dimos.benchmark.vqa.evaluation.models import (
    LangChainVisionEvaluationConfig,
    SingleFrameVqaEvaluationCase,
    SingleFrameVqaEvaluationResult,
    SingleFrameVqaOracle,
)


def test_single_frame_vqa_case_has_only_public_evaluation_inputs() -> None:
    case = SingleFrameVqaEvaluationCase(
        case_id="go2-1-chair-direction",
        image_path="public/image.jpg",
        question="Where is the nearest chair: left, center, or right?",
        allowed_answers=("left", "center", "right"),
    )

    assert case.model_dump() == {
        "schema_version": "1.0",
        "case_id": "go2-1-chair-direction",
        "kind": "single_frame_vqa",
        "image_path": "public/image.jpg",
        "question": "Where is the nearest chair: left, center, or right?",
        "allowed_answers": ("left", "center", "right"),
        "answer_marker": "ANSWER:",
    }


@pytest.mark.parametrize("image_path", ("/image.jpg", "../image.jpg", "images/../../image.jpg"))
def test_single_frame_vqa_case_rejects_unsafe_image_paths(image_path: str) -> None:
    with pytest.raises(ValueError, match="safe relative path"):
        SingleFrameVqaEvaluationCase(
            case_id="case",
            image_path=image_path,
            question="Is there a chair?",
            allowed_answers=("yes", "no"),
        )


def test_single_frame_vqa_case_rejects_duplicate_answer_choices() -> None:
    with pytest.raises(ValueError, match="unique ignoring case"):
        SingleFrameVqaEvaluationCase(
            case_id="case",
            image_path="public/image.jpg",
            question="Is there a chair?",
            allowed_answers=("yes", "YES"),
        )


def test_private_oracle_and_result_are_separate_from_public_case() -> None:
    oracle = SingleFrameVqaOracle(
        case_id="case",
        validator_revision="v1",
        expected_answer="left",
        ground_truth_revision="grounding-v1",
    )
    result = SingleFrameVqaEvaluationResult(
        case_id="case",
        model=LangChainVisionEvaluationConfig().model,
        raw_response="ANSWER: left",
        normalized_answer="left",
        passed=True,
    )

    assert oracle.expected_answer == "left"
    assert result.passed is True
