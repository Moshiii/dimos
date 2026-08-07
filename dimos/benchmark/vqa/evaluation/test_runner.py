# Copyright 2026 Dimensional Inc.

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from dimos.benchmark.vqa.evaluation.models import (
    SingleFrameVqaEvaluationCase,
    SingleFrameVqaOracle,
)
from dimos.benchmark.vqa.evaluation.runner import evaluate_case, evaluate_dataset
from dimos.msgs.sensor_msgs.Image import Image


class _Answerer:
    def __init__(self, response: str) -> None:
        self._response = response
        self.case: SingleFrameVqaEvaluationCase | None = None
        self.image: Image | None = None

    def answer(self, image: Image, case: SingleFrameVqaEvaluationCase) -> str:
        self.image = image
        self.case = case
        return self._response


def _write_case(root: Path, expected_answer: str = "left") -> Path:
    private = root / "private"
    private.mkdir(parents=True)
    assert cv2.imwrite(str(root / "image.jpg"), np.zeros((4, 4, 3), dtype=np.uint8))
    case = SingleFrameVqaEvaluationCase(
        case_id="case-1",
        image_path="image.jpg",
        question="Which chair is closer?",
        allowed_answers=("left", "right"),
    )
    oracle = SingleFrameVqaOracle(
        case_id="case-1",
        validator_revision="v1",
        expected_answer=expected_answer,
        ground_truth_revision="grounding-v1",
    )
    (root / "case.json").write_text(case.model_dump_json())
    (private / "oracle.json").write_text(oracle.model_dump_json())
    return root


def test_evaluate_case_sends_only_public_case_to_answerer(tmp_path: Path) -> None:
    answerer = _Answerer("ANSWER: LEFT")
    case_path = _write_case(tmp_path / "case")

    result = evaluate_case(case_path, answerer, "gpt-5.6-luna")

    assert result.passed is True
    assert result.normalized_answer == "left"
    assert answerer.image is not None
    assert answerer.case is not None
    assert "expected_answer" not in answerer.case.model_dump()


def test_evaluate_case_rejects_invalid_closed_answer(tmp_path: Path) -> None:
    result = evaluate_case(_write_case(tmp_path / "case"), _Answerer("ANSWER: center"), "model")

    assert result.normalized_answer is None
    assert result.passed is False


def test_evaluate_case_rejects_answer_marker_followed_by_explanation(tmp_path: Path) -> None:
    result = evaluate_case(
        _write_case(tmp_path / "case"),
        _Answerer("ANSWER: left\nThe chair on the left is closer."),
        "model",
    )

    assert result.normalized_answer is None
    assert result.passed is False


def test_evaluate_dataset_loads_exported_case_directories(tmp_path: Path) -> None:
    case_path = _write_case(tmp_path / "frame-000001" / "cases" / "case-1")
    answerer = _Answerer("ANSWER: left")

    results = evaluate_dataset(tmp_path, answerer, "model")

    assert len(results) == 1
    assert results[0].case_id == "case-1"
    assert json.loads((case_path / "case.json").read_text())["question"] == "Which chair is closer?"
