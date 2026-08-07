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

"""Image-only visual-question-answering evaluation."""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Protocol

from dimos.benchmark.vqa.evaluation.models import (
    SingleFrameVqaEvaluationCase,
    SingleFrameVqaEvaluationResult,
    SingleFrameVqaOracle,
)
from dimos.benchmark.vqa.models import VqaExample
from dimos.msgs.sensor_msgs.Image import Image


@dataclass(frozen=True)
class VqaEvaluation:
    """The result of asking an image-only agent one generated question."""

    example_id: str
    expected_answer: str
    raw_response: str
    normalized_response: str | None
    passed: bool


class VisualQuestionAnswerer(Protocol):
    """Answers a question from an image without access to ground truth."""

    def answer(self, image: Image, question: str) -> str: ...


def evaluate_examples(
    image: Image, examples: list[VqaExample], answerer: VisualQuestionAnswerer
) -> list[VqaEvaluation]:
    """Ask an agent questions from an image and compare closed answers."""
    evaluations: list[VqaEvaluation] = []
    for example in examples:
        raw_response = answerer.answer(image, example.question)
        normalized = _normalize_response(raw_response, example.allowed_answers)
        evaluations.append(
            VqaEvaluation(
                example_id=example.id,
                expected_answer=example.expected_answer,
                raw_response=raw_response,
                normalized_response=normalized,
                passed=normalized == example.expected_answer,
            )
        )
    return evaluations


def _normalize_response(response: str, allowed_answers: tuple[str, ...]) -> str | None:
    tokens = re.findall(r"[a-z]+", response.lower())
    matches = [answer for answer in allowed_answers if answer.lower() in tokens]
    return matches[0] if len(matches) == 1 else None


def score_vqa_response(
    case: SingleFrameVqaEvaluationCase,
    oracle: SingleFrameVqaOracle,
    response: str,
    model: str,
) -> SingleFrameVqaEvaluationResult:
    """Parse one closed answer using public choices and score it privately."""
    if oracle.case_id != case.case_id:
        raise ValueError("oracle case_id must match the public case")
    expected = _canonical_answer(oracle.expected_answer, case.allowed_answers)
    if expected is None:
        raise ValueError("oracle expected_answer must be an allowed public answer")
    answer = _parse_marked_answer(response, case.answer_marker, case.allowed_answers)
    return SingleFrameVqaEvaluationResult(
        case_id=case.case_id,
        model=model,
        raw_response=response,
        normalized_answer=answer,
        passed=answer == expected,
        validator_revision=oracle.validator_revision,
    )


def _parse_marked_answer(
    response: str, marker: str, allowed_answers: tuple[str, ...]
) -> str | None:
    lines = [line.strip() for line in response.splitlines() if line.strip()]
    marked = [line for line in lines if line.startswith(marker)]
    if len(marked) != 1 or not lines or lines[-1] != marked[0]:
        return None
    return _canonical_answer(marked[0].removeprefix(marker), allowed_answers)


def _canonical_answer(value: str, allowed_answers: tuple[str, ...]) -> str | None:
    normalized = value.strip().lower()
    return next((answer for answer in allowed_answers if answer.lower() == normalized), None)
