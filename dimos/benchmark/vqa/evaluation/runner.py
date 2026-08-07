# Copyright 2026 Dimensional Inc.
"""Trusted loading and scoring for standalone single-frame VQA cases."""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

import cv2

from dimos.benchmark.vqa.evaluation.models import (
    SingleFrameVqaEvaluationCase,
    SingleFrameVqaEvaluationResult,
    SingleFrameVqaOracle,
)
from dimos.benchmark.vqa.evaluation.scoring import score_vqa_response
from dimos.msgs.sensor_msgs.Image import Image


class SingleFrameVqaAnswerer(Protocol):
    """Answers a public VQA case from its image and public case record."""

    def answer(self, image: Image, case: SingleFrameVqaEvaluationCase) -> str: ...


def evaluate_case(
    case_path: Path, answerer: SingleFrameVqaAnswerer, model: str
) -> SingleFrameVqaEvaluationResult:
    """Run a public VQA case, then load its private oracle for deterministic scoring."""
    root = case_path.expanduser().resolve()
    case = SingleFrameVqaEvaluationCase.model_validate_json((root / "case.json").read_bytes())
    image_path = _case_path(root, case.image_path)
    image_data = cv2.imread(str(image_path))
    if image_data is None:
        raise ValueError(f"failed to read case image: {image_path}")
    response = answerer.answer(Image.from_numpy(image_data), case)
    oracle = SingleFrameVqaOracle.model_validate_json(
        (root / "private" / "oracle.json").read_bytes()
    )
    return score_vqa_response(case, oracle, response, model)


def evaluate_dataset(
    dataset: Path, answerer: SingleFrameVqaAnswerer, model: str
) -> list[SingleFrameVqaEvaluationResult]:
    """Evaluate every exported VQA case in a generated dataset."""
    root = dataset.expanduser().resolve()
    case_paths = sorted(path.parent for path in root.glob("frame-*/cases/*/case.json"))
    if not case_paths:
        raise ValueError(f"no VQA cases found in {root}")
    return [evaluate_case(path, answerer, model) for path in case_paths]


def _case_path(root: Path, relative_path: str) -> Path:
    path = (root / relative_path).resolve()
    if root not in path.parents:
        raise ValueError("case image path escapes the case directory")
    return path
