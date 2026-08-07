# Copyright 2026 Dimensional Inc.

from __future__ import annotations

from types import SimpleNamespace
from typing import cast

import numpy as np

from dimos.benchmark.vqa.evaluation.langchain import LangChainVisionQuestionAnswerer
from dimos.benchmark.vqa.evaluation.models import (
    LangChainVisionEvaluationConfig,
    SingleFrameVqaEvaluationCase,
)
from dimos.msgs.sensor_msgs.Image import Image


class _Model:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def invoke(self, messages: list[object]) -> SimpleNamespace:
        self.messages = messages
        return SimpleNamespace(content="ANSWER: left")


def test_langchain_answerer_sends_only_public_image_and_case() -> None:
    model = _Model()
    answerer = LangChainVisionQuestionAnswerer(
        LangChainVisionEvaluationConfig(), model=cast("object", model)
    )
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    case = SingleFrameVqaEvaluationCase(
        case_id="case-1",
        image_path="image.jpg",
        question="Which chair is closer?",
        allowed_answers=("left", "right"),
    )

    response = answerer.answer(image, case)

    assert response == "ANSWER: left"
    assert len(model.messages) == 2
    content = model.messages[1].content
    assert isinstance(content, list)
    assert content[0]["text"].startswith("Which chair is closer?")
    assert content[1]["type"] == "image_url"
    assert "expected_answer" not in str(content)
