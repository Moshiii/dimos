# Copyright 2026 Dimensional Inc.
"""Single-frame point-cloud-grounded VQA commands."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re

import typer

from dimos.benchmark.vqa.evaluation.langchain import LangChainVisionQuestionAnswerer
from dimos.benchmark.vqa.evaluation.models import LangChainVisionEvaluationConfig
from dimos.benchmark.vqa.evaluation.runner import evaluate_dataset
from dimos.benchmark.vqa.generation.adapters import (
    EdgeTamObjectSegmenter,
    MoondreamObjectDetector,
)
from dimos.benchmark.vqa.generation.dataset import write_dataset_manifest, write_frame_record
from dimos.benchmark.vqa.generation.ground_truth_generator import VqaGroundTruthGenerator
from dimos.benchmark.vqa.generation.question_agent import OpenAIQuestionAgent
from dimos.benchmark.vqa.generation.recording import load_go2_frame
from dimos.benchmark.vqa.models import GroundingConfig, QuestionIntent
from dimos.constants import STATE_DIR
from dimos.models.base import default_local_model_device
from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.models.vl.openai import OpenAIVlModel
from dimos.utils.data import resolve_named_path

app = typer.Typer(help="Generate point-cloud-grounded VQA benchmark examples")


@app.command("single-frame")
def single_frame(
    recording: str = typer.Option(..., "--recording"),
    frame_index: int = typer.Option(0, "--frame-index"),
    query: list[str] = typer.Option([], "--query"),
    propose_questions: bool = typer.Option(False, "--propose-questions"),
    question_model: str = typer.Option("gpt-4o-mini", "--question-model"),
    min_mask_area_px: int = typer.Option(128, "--min-mask-area-px"),
    min_foreground_points: int = typer.Option(3, "--min-foreground-points"),
    output: Path | None = typer.Option(None, "--output"),
) -> None:
    """Generate private-grounded questions for one Go2 recording frame."""
    output = output or (
        STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frame-{frame_index:06d}"
    )
    if output.exists() or (not query and not propose_questions):
        raise typer.BadParameter("output must not already exist")
    _require_edgetam_cuda()
    frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
    model = MoondreamVlModel()
    model.start()
    question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=question_model))
    try:
        intents = (
            question_agent.propose(frame.image)
            if propose_questions
            else [
                QuestionIntent(
                    kind=kind,
                    object_query=item,
                    threshold_m=3.0 if kind == "within_distance" else None,
                )
                for item in query
                for kind in (
                    "presence",
                    "horizontal_direction",
                    "within_distance",
                    "compare_nearest_by_side",
                )
            ]
        )
        ground_truth = VqaGroundTruthGenerator(
            detector := MoondreamObjectDetector(model),
            segmenter := EdgeTamObjectSegmenter(EdgeTAMImageSegmenter()),
            localizer=detector,
            point_segmenter=segmenter,
            config=GroundingConfig(
                min_mask_area_px=min_mask_area_px,
                min_foreground_points=min_foreground_points,
            ),
        )
        results = [ground_truth.answer(frame, intent) for intent in intents]
        examples = [result.question for result in results if result.status == "answered"]
    finally:
        model.stop()
    write_frame_record(
        output,
        frame,
        recording,
        frame_index,
        intents,
        results,
        ground_truth,
        {
            "question_source": "openai_image_agent" if propose_questions else "explicit_queries",
            "question_model": question_model if propose_questions else None,
            "grounding": {
                "min_mask_area_px": min_mask_area_px,
                "min_foreground_points": min_foreground_points,
            },
        },
    )
    typer.echo(f"Wrote {len(examples)} examples to {output}")


@app.command("generate")
def generate(
    recording: str = typer.Option(..., "--recording"),
    start_index: int = typer.Option(0, "--start-index"),
    stop_index: int = typer.Option(..., "--stop-index"),
    stride: int = typer.Option(1, "--stride"),
    query: list[str] = typer.Option([], "--query"),
    propose_questions: bool = typer.Option(False, "--propose-questions"),
    question_model: str = typer.Option("gpt-4o-mini", "--question-model"),
    min_mask_area_px: int = typer.Option(128, "--min-mask-area-px"),
    min_foreground_points: int = typer.Option(3, "--min-foreground-points"),
    output: Path | None = typer.Option(None, "--output"),
) -> None:
    """Generate a resumable VQA dataset from sampled Go2 recording frames."""
    if (
        start_index < 0
        or stop_index <= start_index
        or stride < 1
        or (not query and not propose_questions)
    ):
        raise typer.BadParameter(
            "provide valid frame bounds and either --query or --propose-questions"
        )
    output = output or (STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frames")
    output.mkdir(parents=True, exist_ok=True)
    _require_edgetam_cuda()
    model = MoondreamVlModel()
    model.start()
    try:
        detector = MoondreamObjectDetector(model)
        segmenter = EdgeTamObjectSegmenter(EdgeTAMImageSegmenter())
        question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=question_model))
        for frame_index in range(start_index, stop_index, stride):
            frame_output = output / f"frame-{frame_index:06d}"
            if (frame_output / "frame.json").is_file():
                continue
            frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
            intents = (
                question_agent.propose(frame.image)
                if propose_questions
                else [
                    QuestionIntent(
                        kind=kind,
                        object_query=item,
                        threshold_m=3.0 if kind == "within_distance" else None,
                    )
                    for item in query
                    for kind in (
                        "presence",
                        "horizontal_direction",
                        "within_distance",
                        "compare_nearest_by_side",
                    )
                ]
            )
            ground_truth = VqaGroundTruthGenerator(
                detector,
                segmenter,
                localizer=detector,
                point_segmenter=segmenter,
                config=GroundingConfig(
                    min_mask_area_px=min_mask_area_px, min_foreground_points=min_foreground_points
                ),
            )
            results = [ground_truth.answer(frame, intent) for intent in intents]
            write_frame_record(
                frame_output,
                frame,
                recording,
                frame_index,
                intents,
                results,
                ground_truth,
                {
                    "question_source": "openai_image_agent"
                    if propose_questions
                    else "explicit_queries",
                    "question_model": question_model if propose_questions else None,
                    "grounding": {
                        "min_mask_area_px": min_mask_area_px,
                        "min_foreground_points": min_foreground_points,
                    },
                },
            )
            typer.echo(f"Generated {frame_output}")
    finally:
        model.stop()
    summary = write_dataset_manifest(output)
    typer.echo(f"Dataset manifest: {summary}")


@app.command("evaluate")
def evaluate(
    dataset: Path = typer.Option(..., "--dataset"),
    model: str = typer.Option("gpt-5.6-luna", "--model"),
    output: Path | None = typer.Option(None, "--output"),
) -> None:
    """Evaluate generated VQA cases with a no-tools LangChain vision model."""
    dataset = dataset.expanduser().resolve()
    if not dataset.is_dir():
        raise typer.BadParameter(f"dataset must be an existing directory: {dataset}")
    if model.startswith("gpt-5") and not os.environ.get("OPENAI_API_KEY"):
        raise typer.BadParameter("OPENAI_API_KEY must be set to evaluate with an OpenAI model")
    output = output or dataset / "evaluations" / _safe_model_path(model)
    if output.exists():
        raise typer.BadParameter(f"output must not already exist: {output}")
    config = LangChainVisionEvaluationConfig(model=model)
    results = evaluate_dataset(dataset, LangChainVisionQuestionAnswerer(config), model)
    output.mkdir(parents=True)
    (output / "results.json").write_text(
        json.dumps([result.model_dump(mode="json") for result in results], indent=2) + "\n"
    )
    summary = {
        "model": model,
        "case_count": len(results),
        "valid_answer_count": sum(result.normalized_answer is not None for result in results),
        "passed_count": sum(result.passed is True for result in results),
        "accuracy": (sum(result.passed is True for result in results) / len(results))
        if results
        else None,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    typer.echo(f"Evaluation summary: {summary}")


def _safe_model_path(model: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", model).strip("-")


def _require_edgetam_cuda() -> None:
    if default_local_model_device() != "cuda":
        raise typer.BadParameter(
            "VQA generation requires an installed PyTorch CUDA build that supports this GPU for EdgeTAM"
        )
