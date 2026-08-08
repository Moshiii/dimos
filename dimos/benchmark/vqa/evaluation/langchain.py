# Copyright 2026 Dimensional Inc.
"""LangChain execution boundary for standalone single-frame VQA evaluation."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.benchmark.vqa.evaluation.models import (
    LangChainVisionEvaluationConfig,
    SingleFrameVqaEvaluationCase,
)
from dimos.msgs.sensor_msgs.Image import Image

if TYPE_CHECKING:
    from langchain_core.language_models.chat_models import BaseChatModel


class LangChainVisionQuestionAnswerer:
    """No-tools LangChain adapter for a public image and VQA case."""

    def __init__(
        self, config: LangChainVisionEvaluationConfig, model: BaseChatModel | None = None
    ) -> None:
        self._config = config
        self._model = model or _create_model(config)

    def answer(self, image: Image, case: SingleFrameVqaEvaluationCase) -> str:
        """Return the model response for the supplied public image and case."""
        from langchain_core.messages import HumanMessage, SystemMessage

        if case.answer_kind == "choice":
            assert case.allowed_answers is not None
            answer_instruction = (
                f"Allowed answers: {', '.join(case.allowed_answers)}. "
                f"End with exactly `{case.answer_marker} <answer>` using one allowed answer."
            )
        else:
            assert case.unit is not None
            answer_instruction = (
                f"End with exactly `{case.answer_marker} <number> {case.unit}` on the final line."
            )
        message = HumanMessage(
            content=[
                {
                    "type": "text",
                    "text": (f"{case.question}\n\n{answer_instruction}"),
                },
                *image.agent_encode(),
            ]
        )
        response = self._model.invoke(
            [
                SystemMessage(
                    "Answer the visual question using only the supplied image. "
                    "You have no tools and must not infer unavailable information."
                ),
                message,
            ]
        )
        return _response_text(response.content)


def _create_model(config: LangChainVisionEvaluationConfig) -> BaseChatModel:
    if config.model.startswith("gpt-5"):
        from langchain_openai import ChatOpenAI

        return ChatOpenAI(
            model=config.model,
            use_responses_api=True,
            reasoning={"effort": config.thinking_level, "summary": "auto"},
        )
    from langchain.chat_models import init_chat_model

    return init_chat_model(config.model)


def _response_text(content: Any) -> str:
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "".join(str(item.get("text", "")) for item in content if isinstance(item, dict))
    return str(content)
