# Copyright 2026 Dimensional Inc.

from __future__ import annotations

import pytest

from dimos.benchmark.vqa.evaluation.models import (
    SingleFrameVqaEvaluationCase,
    SingleFrameVqaOracle,
)
from dimos.benchmark.vqa.evaluation.scoring import score_vqa_response


@pytest.fixture
def numeric_case() -> tuple[SingleFrameVqaEvaluationCase, SingleFrameVqaOracle]:
    return (
        SingleFrameVqaEvaluationCase(
            case_id="chair-height",
            image_path="image.jpg",
            question="How tall is the chair?",
            answer_kind="numeric",
            unit="m",
            tolerance=0.1,
        ),
        SingleFrameVqaOracle(
            case_id="chair-height",
            validator_revision="v1",
            expected_answer=1.2,
            tolerance=0.1,
            ground_truth_revision="height-v1",
        ),
    )


@pytest.mark.parametrize(
    ("response", "passed", "normalized"),
    [
        ("ANSWER: 1.2 m", True, 1.2),
        ("Measured from the image.\nANSWER: 1.15 m", True, 1.15),
        ("ANSWER: 1.31 m", False, 1.31),
        ("ANSWER: 1.2 cm", False, None),
        ("ANSWER: 1.2 m approximately", False, None),
        ("ANSWER: 1e309 m", False, None),
    ],
)
def test_numeric_scoring_requires_strict_final_number_and_unit(
    numeric_case: tuple[SingleFrameVqaEvaluationCase, SingleFrameVqaOracle],
    response: str,
    passed: bool,
    normalized: float | None,
) -> None:
    case, oracle = numeric_case

    result = score_vqa_response(case, oracle, response, "model")

    assert result.passed is passed
    assert result.normalized_answer == normalized


def test_choice_scoring_keeps_invalid_format_rejection() -> None:
    case = SingleFrameVqaEvaluationCase(
        case_id="chair-side",
        image_path="image.jpg",
        question="Which side is the chair on?",
        allowed_answers=("left", "right"),
    )
    oracle = SingleFrameVqaOracle(
        case_id="chair-side",
        validator_revision="v1",
        expected_answer="left",
        ground_truth_revision="v1",
    )

    result = score_vqa_response(case, oracle, "ANSWER: left\nExplanation", "model")

    assert result.normalized_answer is None
    assert result.passed is False
