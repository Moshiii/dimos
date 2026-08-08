# Copyright 2026 Dimensional Inc.
"""Single-frame point-cloud-grounded VQA commands."""

from __future__ import annotations

import os
from pathlib import Path
import re
import time
from typing import Any, cast

import typer

from dimos.benchmark.vqa.evaluation.langchain import LangChainVisionQuestionAnswerer
from dimos.benchmark.vqa.evaluation.models import (
    CaseCompletedEvent,
    CaseStartedEvent,
    EvaluationProgressEvent,
    EvaluationRunConfig,
    InfrastructureFailureEvent,
    LangChainVisionEvaluationConfig,
    RunCompletedEvent,
    RunFailedEvent,
)
from dimos.benchmark.vqa.evaluation.persistence import EvaluationRunStore
from dimos.benchmark.vqa.evaluation.runner import evaluate_run
from dimos.benchmark.vqa.evaluation.viewer import start_viewer
from dimos.benchmark.vqa.generation.adapters import (
    EdgeTamObjectSegmenter,
    MoondreamObjectDetector,
)
from dimos.benchmark.vqa.generation.dataset import write_dataset_manifest, write_frame_record
from dimos.benchmark.vqa.generation.ground_truth_generator import VqaGroundTruthGenerator
from dimos.benchmark.vqa.generation.oracle import create_openai_oracle
from dimos.benchmark.vqa.generation.oracle_tools import LocalOracleToolRegistry
from dimos.benchmark.vqa.generation.question_agent import (
    OpenAIFreeformQuestionAuthor,
    OpenAIQuestionAgent,
)
from dimos.benchmark.vqa.generation.recording import load_go2_frame
from dimos.benchmark.vqa.models import (
    AcceptedOracleResult,
    CalibratedFrame,
    GroundingConfig,
    GroundTruthResult,
    QuestionIntent,
    QuestionProposal,
    RejectedOracleResult,
)
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
    question_mode: str = typer.Option("constrained", "--question-mode"),
    question_model: str = typer.Option("gpt-4o-mini", "--question-model"),
    oracle_model: str = typer.Option("gpt-4o-mini", "--oracle-model"),
    min_mask_area_px: int = typer.Option(128, "--min-mask-area-px"),
    min_foreground_points: int = typer.Option(3, "--min-foreground-points"),
    output: Path | None = typer.Option(None, "--output"),
) -> None:
    """Generate private-grounded questions for one Go2 recording frame."""
    output = output or (
        STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frame-{frame_index:06d}"
    )
    if output.exists() or (not query and not propose_questions and question_mode != "agentic"):
        raise typer.BadParameter("output must not already exist")
    _validate_question_mode(question_mode)
    _require_openai_for_agentic(question_mode)
    _require_edgetam_cuda()
    typer.echo(f"Loading frame {frame_index} from {recording}")
    frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
    model = MoondreamVlModel()
    typer.echo("Loading private MoonDream model")
    model.start()
    question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=question_model))
    try:
        if propose_questions:
            typer.echo(f"Proposing questions with {question_model}")
        intents: list[QuestionIntent] | list[QuestionProposal] = (
            OpenAIFreeformQuestionAuthor(OpenAIVlModel(model_name=question_model)).propose(
                frame.image
            )
            if question_mode == "agentic"
            else question_agent.propose(frame.image)
            if propose_questions
            else [
                QuestionIntent(
                    kind=cast("Any", kind),
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
        typer.echo(f"Grounding {len(intents)} questions for frame {frame_index}")
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
        results: list[GroundTruthResult] | list[AcceptedOracleResult | RejectedOracleResult] = (
            _answer_agentic(
                ground_truth, frame, cast("list[QuestionProposal]", intents), oracle_model
            )
            if question_mode == "agentic"
            else _answer_intents(
                ground_truth, frame, cast("list[QuestionIntent]", intents), f"Frame {frame_index}"
            )
        )
        examples = [
            result
            for result in results
            if isinstance(result, AcceptedOracleResult)
            or (isinstance(result, GroundTruthResult) and result.status == "answered")
        ]
    finally:
        model.stop()
    write_frame_record(
        output,
        frame,
        recording,
        frame_index,
        cast("list[QuestionIntent | QuestionProposal]", intents),
        cast("list[GroundTruthResult | AcceptedOracleResult | RejectedOracleResult]", results),
        ground_truth,
        {
            "question_source": "agentic_image_author"
            if question_mode == "agentic"
            else "openai_image_agent"
            if propose_questions
            else "explicit_queries",
            "question_model": question_model
            if propose_questions or question_mode == "agentic"
            else None,
            "oracle_model": oracle_model if question_mode == "agentic" else None,
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
    question_mode: str = typer.Option("constrained", "--question-mode"),
    question_model: str = typer.Option("gpt-4o-mini", "--question-model"),
    oracle_model: str = typer.Option("gpt-4o-mini", "--oracle-model"),
    min_mask_area_px: int = typer.Option(128, "--min-mask-area-px"),
    min_foreground_points: int = typer.Option(3, "--min-foreground-points"),
    output: Path | None = typer.Option(None, "--output"),
) -> None:
    """Generate a resumable VQA dataset from sampled Go2 recording frames."""
    if (
        start_index < 0
        or stop_index <= start_index
        or stride < 1
        or (not query and not propose_questions and question_mode != "agentic")
    ):
        raise typer.BadParameter(
            "provide valid frame bounds and either --query or --propose-questions"
        )
    output = output or (STATE_DIR / "datasets" / "vqa" / f"{Path(recording).stem}-frames")
    _validate_question_mode(question_mode)
    _require_openai_for_agentic(question_mode)
    output.mkdir(parents=True, exist_ok=True)
    _require_edgetam_cuda()
    frame_indices = range(start_index, stop_index, stride)
    typer.echo(f"Generating {len(frame_indices)} sampled frames from {recording} into {output}")
    model = MoondreamVlModel()
    typer.echo("Loading private MoonDream model")
    model.start()
    try:
        detector = MoondreamObjectDetector(model)
        segmenter = EdgeTamObjectSegmenter(EdgeTAMImageSegmenter())
        question_agent = OpenAIQuestionAgent(OpenAIVlModel(model_name=question_model))
        for frame_number, frame_index in enumerate(frame_indices, start=1):
            frame_output = output / f"frame-{frame_index:06d}"
            if (frame_output / "frame.json").is_file():
                typer.echo(
                    f"Skipping completed frame {frame_number}/{len(frame_indices)}: {frame_index}"
                )
                continue
            typer.echo(
                f"Frame {frame_number}/{len(frame_indices)}: loading recording index {frame_index}"
            )
            frame = load_go2_frame(str(resolve_named_path(recording, ".db")), frame_index)
            if propose_questions:
                typer.echo(f"Frame {frame_index}: proposing questions with {question_model}")
            intents: list[QuestionIntent] | list[QuestionProposal] = (
                OpenAIFreeformQuestionAuthor(OpenAIVlModel(model_name=question_model)).propose(
                    frame.image
                )
                if question_mode == "agentic"
                else question_agent.propose(frame.image)
                if propose_questions
                else [
                    QuestionIntent(
                        kind=cast("Any", kind),
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
            typer.echo(f"Frame {frame_index}: grounding {len(intents)} questions")
            ground_truth = VqaGroundTruthGenerator(
                detector,
                segmenter,
                localizer=detector,
                point_segmenter=segmenter,
                config=GroundingConfig(
                    min_mask_area_px=min_mask_area_px, min_foreground_points=min_foreground_points
                ),
            )
            results: list[GroundTruthResult] | list[AcceptedOracleResult | RejectedOracleResult] = (
                _answer_agentic(
                    ground_truth, frame, cast("list[QuestionProposal]", intents), oracle_model
                )
                if question_mode == "agentic"
                else _answer_intents(
                    ground_truth,
                    frame,
                    cast("list[QuestionIntent]", intents),
                    f"Frame {frame_index}",
                )
            )
            write_frame_record(
                frame_output,
                frame,
                recording,
                frame_index,
                cast("list[QuestionIntent | QuestionProposal]", intents),
                cast(
                    "list[GroundTruthResult | AcceptedOracleResult | RejectedOracleResult]", results
                ),
                ground_truth,
                {
                    "question_source": "agentic_image_author"
                    if question_mode == "agentic"
                    else "openai_image_agent"
                    if propose_questions
                    else "explicit_queries",
                    "question_model": question_model
                    if propose_questions or question_mode == "agentic"
                    else None,
                    "oracle_model": oracle_model if question_mode == "agentic" else None,
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
    workers: int = typer.Option(1, "--workers", min=1),
    resume: bool = typer.Option(False, "--resume/--no-resume"),
    viewer: bool = typer.Option(True, "--viewer/--no-viewer"),
) -> None:
    """Evaluate generated VQA cases with a no-tools LangChain vision model."""
    dataset = dataset.expanduser().resolve()
    if not dataset.is_dir():
        raise typer.BadParameter(f"dataset must be an existing directory: {dataset}")
    if model.startswith("gpt-5") and not os.environ.get("OPENAI_API_KEY"):
        raise typer.BadParameter("OPENAI_API_KEY must be set to evaluate with an OpenAI model")
    output = output or dataset / "evaluations" / _safe_model_path(model)
    config = LangChainVisionEvaluationConfig(model=model)
    run_config = EvaluationRunConfig(model=model, langchain=config, workers=workers)
    try:
        store = (
            EvaluationRunStore.resume(output, dataset, run_config)
            if resume
            else EvaluationRunStore.create(output, dataset, run_config)
        )
    except ValueError as error:
        raise typer.BadParameter(str(error)) from error
    typer.echo(f"Evaluating VQA cases from {dataset} with {model}")
    typer.echo(f"Run {store.manifest.run_id} started with {workers} workers")
    if viewer:
        typer.echo(f"VQA review viewer: {start_viewer(dataset, store.root).url}")
    try:
        evaluate_run(
            dataset,
            store,
            lambda: LangChainVisionQuestionAnswerer(config),
            on_event=lambda event: typer.echo(_render_event(event)),
        )
        if viewer:
            # Allow the browser to poll the atomically published final result.
            time.sleep(1)
        typer.echo(f"Evaluation summary: {(store.root / 'summary.json').read_text().strip()}")
    finally:
        store.close()


def _safe_model_path(model: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "-", model).strip("-")


def _render_event(event: EvaluationProgressEvent) -> str:
    if isinstance(event, CaseStartedEvent):
        return f"Case {event.index + 1}: sending {event.case_id}"
    if isinstance(event, InfrastructureFailureEvent):
        return f"Case {event.index + 1 if event.index is not None else '?'}: {event.error}"
    if isinstance(event, CaseCompletedEvent):
        if event.result.infra_error:
            state = "infrastructure error"
        elif event.result.passed:
            state = "passed"
        else:
            state = "failed"
        return f"Case {event.index + 1}: {state} ({event.case_id})"
    if isinstance(event, RunCompletedEvent):
        return "Run completed"
    if isinstance(event, RunFailedEvent):
        return f"Run failed: {event.error}"
    return "Run event"


def _answer_intents(
    ground_truth: VqaGroundTruthGenerator,
    frame: CalibratedFrame,
    intents: list[QuestionIntent],
    label: str,
) -> list[GroundTruthResult]:
    results: list[GroundTruthResult] = []
    for number, intent in enumerate(intents, start=1):
        typer.echo(
            f"{label}: grounding question {number}/{len(intents)}: "
            f"{intent.kind} ({intent.object_query})"
        )
        result = ground_truth.answer(frame, intent)
        results.append(result)
        typer.echo(f"{label}: question {number}/{len(intents)} {result.status}")
    return results


def _answer_agentic(
    ground_truth: VqaGroundTruthGenerator,
    frame: CalibratedFrame,
    proposals: list[QuestionProposal],
    oracle_model: str,
) -> list[AcceptedOracleResult | RejectedOracleResult]:
    oracle = create_openai_oracle(oracle_model)
    results: list[AcceptedOracleResult | RejectedOracleResult] = []
    for number, proposal in enumerate(proposals, start=1):
        typer.echo(f"Agentic question {number}/{len(proposals)}: {proposal.question}")
        result = oracle.answer(proposal, LocalOracleToolRegistry(frame, ground_truth))
        results.append(result)
        if isinstance(result, AcceptedOracleResult):
            typer.echo(f"Agentic question {number}/{len(proposals)} accepted")
        else:
            typer.echo(f"Agentic question {number}/{len(proposals)} rejected: {result.reason}")
    return results


def _validate_question_mode(question_mode: str) -> None:
    if question_mode not in ("constrained", "agentic"):
        raise typer.BadParameter("question mode must be constrained or agentic")


def _require_openai_for_agentic(question_mode: str) -> None:
    if question_mode == "agentic" and not os.environ.get("OPENAI_API_KEY"):
        raise typer.BadParameter("OPENAI_API_KEY must be set for agentic question mode")


def _require_edgetam_cuda() -> None:
    if default_local_model_device() != "cuda":
        raise typer.BadParameter(
            "VQA generation requires an installed PyTorch CUDA build that supports this GPU for EdgeTAM"
        )
