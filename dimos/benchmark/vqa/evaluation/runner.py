# Copyright 2026 Dimensional Inc.
"""Trusted loading, scoring, and run-oriented VQA evaluation execution."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from concurrent.futures import FIRST_COMPLETED, Future, ThreadPoolExecutor, wait
from datetime import UTC, datetime
from pathlib import Path
from typing import Protocol

import cv2

from dimos.benchmark.vqa.evaluation.models import (
    CaseCompletedEvent,
    CaseStartedEvent,
    EvaluationCheckpoint,
    EvaluationProgressEvent,
    InfrastructureFailureEvent,
    SingleFrameVqaEvaluationCase,
    SingleFrameVqaEvaluationResult,
    SingleFrameVqaOracle,
)
from dimos.benchmark.vqa.evaluation.persistence import EvaluationRunStore
from dimos.benchmark.vqa.evaluation.scoring import score_vqa_response
from dimos.msgs.sensor_msgs.Image import Image


class SingleFrameVqaAnswerer(Protocol):
    """Answers a public VQA case from its image and public case record."""

    def answer(self, image: Image, case: SingleFrameVqaEvaluationCase) -> str: ...


AnswererFactory = Callable[[], SingleFrameVqaAnswerer]


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


def evaluate_run(
    dataset: Path,
    store: EvaluationRunStore,
    answerer_factory: AnswererFactory,
    on_event: Callable[[EvaluationProgressEvent], None] | None = None,
) -> list[SingleFrameVqaEvaluationResult]:
    """Evaluate a deterministic inventory, checkpointing terminal case results."""
    case_paths = inventory(dataset)
    if not case_paths:
        raise ValueError(f"no VQA cases found in {dataset.expanduser().resolve()}")
    existing = store.load_checkpoints()
    checkpoints: dict[int, EvaluationCheckpoint] = dict(existing)
    pending = [(index, path) for index, path in enumerate(case_paths) if index not in existing]
    workers = store.manifest.config.workers

    def emit(event: EvaluationProgressEvent) -> None:
        store.record_event(event)
        if on_event is not None:
            on_event(event)

    try:
        with ThreadPoolExecutor(max_workers=workers) as executor:
            futures: dict[Future[SingleFrameVqaEvaluationResult], tuple[int, str]] = {}
            pending_iter = iter(pending)

            def submit_next() -> bool:
                try:
                    index, path = next(pending_iter)
                except StopIteration:
                    return False
                case_id = _case_id(path)
                emit(
                    CaseStartedEvent(
                        run_id=store.manifest.run_id,
                        timestamp=_timestamp(),
                        index=index,
                        case_id=case_id,
                    )
                )
                futures[
                    executor.submit(
                        _evaluate_one, path, answerer_factory, store.manifest.config.model
                    )
                ] = (
                    index,
                    case_id,
                )
                return True

            for _ in range(min(workers, len(pending))):
                submit_next()
            while futures:
                completed, _ = wait(futures, return_when=FIRST_COMPLETED)
                for future in completed:
                    index, case_id = futures.pop(future)
                    try:
                        result = future.result()
                    except Exception as error:
                        message = f"evaluation worker failed ({type(error).__name__})"
                        result = SingleFrameVqaEvaluationResult(
                            case_id=case_id, model=store.manifest.config.model, infra_error=message
                        )
                        emit(
                            InfrastructureFailureEvent(
                                run_id=store.manifest.run_id,
                                timestamp=_timestamp(),
                                index=index,
                                case_id=case_id,
                                error=message,
                            )
                        )
                    checkpoint = EvaluationCheckpoint(index=index, case_id=case_id, result=result)
                    store.write_checkpoint(checkpoint)
                    checkpoints[index] = checkpoint
                    emit(
                        CaseCompletedEvent(
                            run_id=store.manifest.run_id,
                            timestamp=_timestamp(),
                            index=index,
                            case_id=case_id,
                            result=result,
                        )
                    )
                    submit_next()
        ordered = [checkpoints[index] for index in range(len(case_paths))]
        summary = evaluation_summary(item.result for item in ordered)
        completed_event = store.publish_final(ordered, summary)
        if on_event is not None:
            on_event(completed_event)
        return [item.result for item in ordered]
    except Exception:
        failed_event = store.fail("run coordinator failed")
        if on_event is not None:
            on_event(failed_event)
        raise


def inventory(dataset: Path) -> list[Path]:
    """Return the stable public case inventory for a generated dataset."""
    root = dataset.expanduser().resolve()
    index_path = root / "public_cases.parquet"
    if index_path.is_file():
        try:
            import pyarrow.parquet as pq
        except ImportError:
            pass
        else:
            rows = pq.read_table(index_path, columns=["case_path"]).to_pylist()
            case_paths = [(root / row["case_path"]).resolve() for row in rows]
            if any(root not in path.parents for path in case_paths):
                raise ValueError("public case index contains a path outside the dataset")
            return sorted(case_paths)
    return sorted(path.parent for path in root.glob("frame-*/cases/*/case.json"))


def evaluation_summary(
    results: Iterable[SingleFrameVqaEvaluationResult],
) -> dict[str, int | float | None]:
    """Summarize valid scores separately from infrastructure failures."""
    items = list(results)
    valid = sum(item.normalized_answer is not None for item in items)
    passed = sum(item.passed is True for item in items)
    failed = sum(item.passed is False for item in items)
    infra_errors = sum(item.infra_error is not None for item in items)
    scored = passed + failed
    return {
        "case_count": len(items),
        "valid_answer_count": valid,
        "passed_count": passed,
        "failed_count": failed,
        "infra_error_count": infra_errors,
        "accuracy": passed / scored if scored else None,
    }


def _evaluate_one(
    path: Path, answerer_factory: AnswererFactory, model: str
) -> SingleFrameVqaEvaluationResult:
    return evaluate_case(path, answerer_factory(), model)


def _case_id(path: Path) -> str:
    try:
        return SingleFrameVqaEvaluationCase.model_validate_json(
            (path / "case.json").read_bytes()
        ).case_id
    except Exception:
        return path.name


def _case_path(root: Path, relative_path: str) -> Path:
    path = (root / relative_path).resolve()
    if root not in path.parents:
        raise ValueError("case image path escapes the case directory")
    return path


def _timestamp() -> str:
    return datetime.now(UTC).isoformat()
