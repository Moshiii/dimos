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
        "answer_kind": "choice",
        "allowed_answers": ("left", "center", "right"),
        "unit": None,
        "tolerance": None,
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


def test_numeric_case_requires_public_unit_and_non_negative_tolerance() -> None:
    case = SingleFrameVqaEvaluationCase(
        case_id="chair-height",
        image_path="image.jpg",
        question="How tall is the chair?",
        answer_kind="numeric",
        unit="m",
        tolerance=0.1,
    )

    assert case.allowed_answers is None
    assert case.unit == "m"
    assert case.tolerance == 0.1
    with pytest.raises(ValueError, match="cannot define allowed_answers"):
        SingleFrameVqaEvaluationCase(
            case_id="chair-height",
            image_path="image.jpg",
            question="How tall is the chair?",
            answer_kind="numeric",
            allowed_answers=("short", "tall"),
            unit="m",
            tolerance=0.1,
        )
    with pytest.raises(ValueError, match="require unit and tolerance"):
        SingleFrameVqaEvaluationCase(
            case_id="chair-height",
            image_path="image.jpg",
            question="How tall is the chair?",
            answer_kind="numeric",
        )
    with pytest.raises(ValueError):
        SingleFrameVqaEvaluationCase(
            case_id="chair-height",
            image_path="image.jpg",
            question="How tall is the chair?",
            answer_kind="numeric",
            unit="m",
            tolerance=-0.1,
        )


def test_numeric_oracle_and_result_accept_finite_normalized_answers() -> None:
    oracle = SingleFrameVqaOracle(
        case_id="chair-height",
        validator_revision="v1",
        expected_answer=1.2,
        tolerance=0.1,
        ground_truth_revision="height-v1",
    )
    result = SingleFrameVqaEvaluationResult(
        case_id="chair-height",
        model="model",
        normalized_answer=1.15,
        passed=True,
    )

    assert oracle.expected_answer == 1.2
    assert result.normalized_answer == 1.15
    with pytest.raises(ValueError, match="finite"):
        SingleFrameVqaEvaluationResult(
            case_id="chair-height",
            model="model",
            normalized_answer=float("inf"),
        )


def test_public_case_rejects_unknown_json_fields() -> None:
    with pytest.raises(ValueError):
        SingleFrameVqaEvaluationCase.model_validate(
            {
                "case_id": "case",
                "image_path": "image.jpg",
                "question": "Is there a chair?",
                "allowed_answers": ["yes", "no"],
                "private_answer": "yes",
            }
        )
