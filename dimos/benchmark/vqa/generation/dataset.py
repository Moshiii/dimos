# Copyright 2026 Dimensional Inc.
"""Persist generated single-frame VQA records and dataset manifests."""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
import shutil
from typing import Any

import cv2

from dimos.benchmark.vqa.evaluation.models import (
    SingleFrameVqaEvaluationCase,
    SingleFrameVqaOracle,
)
from dimos.benchmark.vqa.generation.ground_truth_generator import VqaGroundTruthGenerator
from dimos.benchmark.vqa.models import (
    AcceptedOracleResult,
    BooleanAnswerContract,
    CalibratedFrame,
    ChoiceAnswerContract,
    GroundTruthResult,
    NumericAnswerContract,
    QuestionIntent,
    QuestionProposal,
    RejectedOracleResult,
)


def write_frame_record(
    output: Path,
    frame: CalibratedFrame,
    recording: str,
    frame_index: int,
    intents: list[QuestionIntent | QuestionProposal],
    results: list[GroundTruthResult | AcceptedOracleResult | RejectedOracleResult],
    ground_truth: VqaGroundTruthGenerator,
    metadata: dict[str, Any],
) -> None:
    """Write one self-contained frame record and its private evidence."""
    output.mkdir(parents=True, exist_ok=False)
    image_path = output / "image.jpg"
    if not cv2.imwrite(str(image_path), frame.image.data):
        raise RuntimeError(f"failed to write {image_path}")
    original_image_path = output / "original_image.jpg"
    if frame.original_image is not None and not cv2.imwrite(
        str(original_image_path), frame.original_image.data
    ):
        raise RuntimeError(f"failed to write {original_image_path}")
    overlay_path = output / "grounding_overlay.jpg"
    ground_truth.write_overlay(frame, str(overlay_path))
    accepted: list[GroundTruthResult | AcceptedOracleResult] = []
    for result in results:
        if isinstance(result, RejectedOracleResult):
            continue
        if isinstance(result, GroundTruthResult) and result.status != "answered":
            continue
        accepted.append(result)
    frame_meta = {
        "schema_version": "1.0",
        "frame_id": frame.id,
        "recording": recording,
        "frame_index": frame_index,
        "image": image_path.name,
        "original_image": original_image_path.name if frame.original_image is not None else None,
        "grounding_overlay": overlay_path.name,
        "question_count": len(intents),
        "accepted_question_count": len(accepted),
        "rejected_question_count": len(results) - len(accepted),
        **metadata,
    }
    _write_json(output / "frame.json", frame_meta)
    _write_json(output / "intents.json", [asdict(item) for item in intents])
    _write_json(
        output / "examples.json", [_public_example(result, frame.id) for result in accepted]
    )
    _write_json(output / "ground_truth.json", [_private_result(item) for item in results])
    _write_evaluation_cases(output, image_path, frame.id, accepted)


def write_dataset_manifest(output: Path) -> dict[str, int]:
    """Rebuild aggregate manifests from completed frame record directories."""
    frames = sorted(path for path in output.glob("frame-*") if (path / "frame.json").is_file())
    frame_rows: list[dict[str, Any]] = []
    public_case_rows: list[dict[str, Any]] = []
    private_oracle_rows: list[dict[str, Any]] = []
    accepted = 0
    rejected = 0
    with (
        (output / "frames.jsonl").open("w") as frame_file,
        (output / "ground_truth.jsonl").open("w") as gt_file,
    ):
        for path in frames:
            frame = json.loads((path / "frame.json").read_text())
            frame_rows.append(frame)
            frame_file.write(json.dumps(frame) + "\n")
            for result in json.loads((path / "ground_truth.json").read_text()):
                gt_file.write(json.dumps({"frame_id": frame["frame_id"], **result}) + "\n")
                if result["status"] == "answered":
                    accepted += 1
                else:
                    rejected += 1
            for case_json_path in sorted(path.glob("cases/*/case.json")):
                case_dir = case_json_path.parent
                oracle_path = case_dir / "private" / "oracle.json"
                public_case_rows.append(_public_case_row(output, frame["frame_id"], case_json_path))
                private_oracle_rows.append(
                    _private_oracle_row(output, frame["frame_id"], oracle_path)
                )
    _write_jsonl(output / "public_cases.jsonl", public_case_rows)
    _write_jsonl(output / "private_oracles.jsonl", private_oracle_rows)
    _write_parquet(output / "frames.parquet", frame_rows)
    _write_parquet(output / "public_cases.parquet", public_case_rows)
    _write_parquet(output / "private_oracles.parquet", private_oracle_rows)
    summary = {
        "frame_count": len(frames),
        "accepted_question_count": accepted,
        "rejected_question_count": rejected,
    }
    _write_json(output / "manifest.json", summary)
    return summary


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(json.dumps(payload, indent=2) + "\n")


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text("".join(f"{_json_line(row)}\n" for row in rows))


def _json_line(payload: dict[str, Any]) -> str:
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def _write_parquet(path: Path, rows: list[dict[str, Any]]) -> None:
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ImportError as error:
        raise RuntimeError("PyArrow is required to write VQA aggregate indexes") from error
    pq.write_table(pa.Table.from_pylist(rows), path)


def _public_case_row(output: Path, frame_id: str, case_json_path: Path) -> dict[str, Any]:
    case = json.loads(case_json_path.read_text())
    case_dir = case_json_path.parent
    return {
        "frame_id": frame_id,
        "case_path": str(case_dir.relative_to(output)),
        "case_json_path": str(case_json_path.relative_to(output)),
        "image_path": case["image_path"],
        "image_artifact_path": str((case_dir / case["image_path"]).relative_to(output)),
        "schema_version": case["schema_version"],
        "case_id": case["case_id"],
        "kind": case["kind"],
        "question": case["question"],
        "answer_kind": case["answer_kind"],
        "allowed_answers_json": json.dumps(case.get("allowed_answers"), separators=(",", ":")),
        "unit": case.get("unit"),
        "tolerance": case.get("tolerance"),
        "answer_marker": case["answer_marker"],
    }


def _private_oracle_row(output: Path, frame_id: str, oracle_path: Path) -> dict[str, Any]:
    oracle = json.loads(oracle_path.read_text())
    case_dir = oracle_path.parent.parent
    return {
        "frame_id": frame_id,
        "case_id": oracle["case_id"],
        "oracle_json_path": str(oracle_path.relative_to(output)),
        "ground_truth_path": str((oracle_path.parent / "ground_truth.json").relative_to(output)),
        "expected_answer_json": json.dumps(oracle["expected_answer"], separators=(",", ":")),
        "tolerance": oracle.get("tolerance"),
        "validator_revision": oracle["validator_revision"],
        "ground_truth_revision": oracle["ground_truth_revision"],
        "case_path": str(case_dir.relative_to(output)),
    }


def _public_example(
    result: GroundTruthResult | AcceptedOracleResult, frame_id: str
) -> dict[str, Any]:
    if isinstance(result, AcceptedOracleResult):
        proposal = result.proposal
        return {
            "case_id": _agentic_case_id(frame_id, proposal.id),
            "question": proposal.question,
            "answer_contract": asdict(proposal.answer_contract),
            "object_queries": proposal.object_queries,
        }
    example = result.question
    return {
        "case_id": example.id,
        "question": example.question,
        "allowed_answers": example.allowed_answers,
        "answer_type": example.answer_type,
        "object_ids": example.object_ids,
    }


def _private_result(
    result: GroundTruthResult | AcceptedOracleResult | RejectedOracleResult,
) -> dict[str, Any]:
    if isinstance(result, AcceptedOracleResult):
        return {
            "status": "answered",
            "answer": result.answer,
            "proposal": asdict(result.proposal),
            "evidence_ids": result.evidence_ids,
            "tool_results": [asdict(item) for item in result.tool_results],
            "trace": [asdict(item) for item in result.trace],
        }
    if isinstance(result, RejectedOracleResult):
        return {
            "status": "rejected",
            "reason": result.reason,
            "proposal": asdict(result.proposal),
            "tool_results": [asdict(item) for item in result.tool_results],
            "trace": [asdict(item) for item in result.trace],
        }
    return asdict(result)


def _write_evaluation_cases(
    output: Path,
    image_path: Path,
    frame_id: str,
    results: list[GroundTruthResult | AcceptedOracleResult],
) -> None:
    cases_root = output / "cases"
    cases_root.mkdir()
    for result in results:
        if isinstance(result, AcceptedOracleResult):
            _write_agentic_case(cases_root, image_path, frame_id, result)
            continue
        example = result.question
        if not example.allowed_answers or result.answer is None:
            raise ValueError(
                f"accepted example {example.id} must define allowed answers and an answer"
            )
        case_dir = (cases_root / example.id).resolve()
        if cases_root.resolve() not in case_dir.parents:
            raise ValueError(f"example id must be a safe case path: {example.id}")
        private_dir = case_dir / "private"
        private_dir.mkdir(parents=True)
        shutil.copy2(image_path, case_dir / "image.jpg")
        case = SingleFrameVqaEvaluationCase(
            case_id=example.id,
            image_path="image.jpg",
            question=example.question,
            allowed_answers=example.allowed_answers,
        )
        oracle = SingleFrameVqaOracle(
            case_id=example.id,
            validator_revision="v1",
            expected_answer=result.answer,
            ground_truth_revision="v1",
        )
        _write_json(case_dir / "case.json", case.model_dump(mode="json"))
        _write_json(private_dir / "oracle.json", oracle.model_dump(mode="json"))
        _write_json(private_dir / "ground_truth.json", asdict(result))


def _write_agentic_case(
    cases_root: Path, image_path: Path, frame_id: str, result: AcceptedOracleResult
) -> None:
    proposal = result.proposal
    case_id = _agentic_case_id(frame_id, proposal.id)
    case_dir = (cases_root / proposal.id).resolve()
    if cases_root.resolve() not in case_dir.parents:
        raise ValueError(f"proposal id must be a safe case path: {proposal.id}")
    contract = proposal.answer_contract
    if isinstance(contract, BooleanAnswerContract):
        allowed_answers: tuple[str, ...] = ("yes", "no")
    elif isinstance(contract, ChoiceAnswerContract):
        allowed_answers = contract.choices
    elif isinstance(contract, NumericAnswerContract):
        allowed_answers = None
    else:
        raise ValueError("unsupported answer contract")
    private_dir = case_dir / "private"
    private_dir.mkdir(parents=True)
    shutil.copy2(image_path, case_dir / "image.jpg")
    if isinstance(contract, NumericAnswerContract):
        if not isinstance(result.answer, float):
            raise ValueError("numeric oracle result must contain a numeric answer")
        case = SingleFrameVqaEvaluationCase(
            case_id=case_id,
            image_path="image.jpg",
            question=proposal.question,
            answer_kind="numeric",
            unit=contract.unit,
            tolerance=contract.tolerance,
        )
        oracle = SingleFrameVqaOracle(
            case_id=case_id,
            validator_revision="v1",
            expected_answer=result.answer,
            tolerance=contract.tolerance,
            ground_truth_revision="agentic-v1",
        )
    else:
        assert allowed_answers is not None
        case = SingleFrameVqaEvaluationCase(
            case_id=case_id,
            image_path="image.jpg",
            question=proposal.question,
            allowed_answers=allowed_answers,
        )
        if not isinstance(result.answer, str):
            raise ValueError("choice oracle result must contain a string answer")
        oracle = SingleFrameVqaOracle(
            case_id=case_id,
            validator_revision="v1",
            expected_answer=result.answer,
            ground_truth_revision="agentic-v1",
        )
    _write_json(case_dir / "case.json", case.model_dump(mode="json", exclude_none=True))
    _write_json(private_dir / "oracle.json", oracle.model_dump(mode="json", exclude_none=True))
    _write_json(private_dir / "ground_truth.json", asdict(result))


def _agentic_case_id(frame_id: str, proposal_id: str) -> str:
    return f"{frame_id}-{proposal_id}"
