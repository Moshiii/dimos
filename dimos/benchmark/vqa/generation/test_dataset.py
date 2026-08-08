# Copyright 2026 Dimensional Inc.

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from dimos.benchmark.vqa.evaluation.models import SingleFrameVqaEvaluationCase, SingleFrameVqaOracle
from dimos.benchmark.vqa.generation.dataset import write_dataset_manifest, write_frame_record
from dimos.benchmark.vqa.models import (
    AcceptedOracleResult,
    BooleanAnswerContract,
    CalibratedFrame,
    GroundTruthResult,
    NumericAnswerContract,
    OracleEvidence,
    OracleToolResult,
    OracleTrace,
    QuestionIntent,
    QuestionProposal,
    VqaExample,
)
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class _GroundTruth:
    def write_overlay(self, frame: CalibratedFrame, path: str) -> None:
        assert cv2.imwrite(path, frame.image.data)


def test_frame_record_exports_public_case_and_private_oracle(tmp_path: Path) -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    frame = CalibratedFrame(
        id="frame-1",
        image=image,
        pointcloud=PointCloud2.from_numpy(np.array([[0.0, 0.0, 1.0]], dtype=np.float32)),
        camera_info=CameraInfo.from_intrinsics(2.0, 2.0, 2.0, 2.0, 4, 4),
        pointcloud_to_camera=Transform.identity(),
        image_is_rectified=True,
    )
    example = VqaExample(
        "frame-1-chair-presence",
        "Is there a chair in the image?",
        "yes",
        "boolean",
        (),
        ("yes", "no"),
    )
    result = GroundTruthResult(
        QuestionIntent(kind="presence", object_query="chair"),
        example,
        "answered",
        "yes",
        None,
        (),
        (),
    )

    write_frame_record(
        tmp_path / "frame-000001", frame, "recording", 1, [], [result], _GroundTruth(), {}
    )

    case_dir = tmp_path / "frame-000001" / "cases" / example.id
    public_case = json.loads((case_dir / "case.json").read_text())
    private_oracle = json.loads((case_dir / "private" / "oracle.json").read_text())
    public_examples = json.loads((tmp_path / "frame-000001" / "examples.json").read_text())
    assert public_case["allowed_answers"] == ["yes", "no"]
    assert "expected_answer" not in public_case
    assert private_oracle["expected_answer"] == "yes"
    assert "expected_answer" not in public_examples[0]


def test_dataset_manifest_writes_matching_public_and_private_aggregate_indexes(
    tmp_path: Path,
) -> None:
    pytest.importorskip("pyarrow")
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    frame = CalibratedFrame(
        id="frame-1",
        image=image,
        pointcloud=PointCloud2.from_numpy(np.array([[0.0, 0.0, 1.0]], dtype=np.float32)),
        camera_info=CameraInfo.from_intrinsics(2.0, 2.0, 2.0, 2.0, 4, 4),
        pointcloud_to_camera=Transform.identity(),
        image_is_rectified=True,
    )
    example = VqaExample(
        "frame-1-chair-presence",
        "Is there a chair in the image?",
        "yes",
        "boolean",
        (),
        ("yes", "no"),
    )
    result = GroundTruthResult(
        QuestionIntent(kind="presence", object_query="chair"),
        example,
        "answered",
        "yes",
        None,
        (),
        (),
    )
    write_frame_record(
        tmp_path / "frame-000001", frame, "recording", 1, [], [result], _GroundTruth(), {}
    )

    assert write_dataset_manifest(tmp_path) == {
        "frame_count": 1,
        "accepted_question_count": 1,
        "rejected_question_count": 0,
    }

    import pyarrow.parquet as pq

    frames = pq.read_table(tmp_path / "frames.parquet").to_pylist()
    public_rows = pq.read_table(tmp_path / "public_cases.parquet").to_pylist()
    private_rows = pq.read_table(tmp_path / "private_oracles.parquet").to_pylist()
    assert frames == [
        json.loads(line) for line in (tmp_path / "frames.jsonl").read_text().splitlines()
    ]
    assert public_rows == [
        json.loads(line) for line in (tmp_path / "public_cases.jsonl").read_text().splitlines()
    ]
    assert private_rows == [
        json.loads(line) for line in (tmp_path / "private_oracles.jsonl").read_text().splitlines()
    ]
    assert public_rows[0]["frame_id"] == "frame-1"
    assert public_rows[0]["allowed_answers_json"] == '["yes","no"]'
    assert public_rows[0]["image_path"] == "image.jpg"
    assert (
        public_rows[0]["image_artifact_path"]
        == "frame-000001/cases/frame-1-chair-presence/image.jpg"
    )
    assert "expected_answer_json" not in public_rows[0]
    assert private_rows[0]["expected_answer_json"] == '"yes"'
    assert private_rows[0]["validator_revision"] == "v1"
    assert private_rows[0]["ground_truth_revision"] == "v1"


def test_agentic_frame_record_never_leaks_private_answer_or_evidence(tmp_path: Path) -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    frame = CalibratedFrame(
        id="frame-1",
        image=image,
        pointcloud=PointCloud2.from_numpy(np.array([[0.0, 0.0, 1.0]], dtype=np.float32)),
        camera_info=CameraInfo.from_intrinsics(2.0, 2.0, 2.0, 2.0, 4, 4),
        pointcloud_to_camera=Transform.identity(),
        image_is_rectified=True,
    )
    proposal = QuestionProposal("chair-presence", "Is there a chair?", BooleanAnswerContract())
    result = AcceptedOracleResult(
        proposal,
        "yes",
        ("grounding:v1:chair-1",),
        (
            OracleToolResult(
                "ground_semantic_object",
                "chair",
                (OracleEvidence("grounding:v1:chair-1", "v1", "chair-1", "chair", 1.0, "left", 3),),
            ),
        ),
        (OracleTrace("tool", "ground_semantic_object"),),
    )

    write_frame_record(
        tmp_path / "frame-000001", frame, "recording", 1, [proposal], [result], _GroundTruth(), {}
    )

    case_dir = tmp_path / "frame-000001" / "cases" / proposal.id
    public_case = json.loads((case_dir / "case.json").read_text())
    private_oracle = json.loads((case_dir / "private" / "oracle.json").read_text())
    case = SingleFrameVqaEvaluationCase.model_validate(public_case)
    oracle = SingleFrameVqaOracle.model_validate(private_oracle)
    assert public_case["case_id"] == "frame-1-chair-presence"
    assert public_case["answer_kind"] == "choice"
    assert public_case["allowed_answers"] == ["yes", "no"]
    assert case.answer_kind == "choice"
    assert oracle.expected_answer == "yes"
    assert "expected_answer" not in public_case
    assert "evidence" not in public_case
    assert "grounding:v1:chair-1" not in json.dumps(public_case)


def test_agentic_numeric_frame_record_exports_typed_public_case_and_private_answer(
    tmp_path: Path,
) -> None:
    image = Image.from_numpy(np.zeros((4, 4, 3), dtype=np.uint8))
    frame = CalibratedFrame(
        id="frame-1",
        image=image,
        pointcloud=PointCloud2.from_numpy(np.array([[0.0, 0.0, 1.0]], dtype=np.float32)),
        camera_info=CameraInfo.from_intrinsics(2.0, 2.0, 2.0, 2.0, 4, 4),
        pointcloud_to_camera=Transform.identity(),
        image_is_rectified=True,
    )
    proposal = QuestionProposal(
        "chair-height", "How tall is the chair?", NumericAnswerContract("m", 0.1)
    )
    result = AcceptedOracleResult(proposal, 1.2, ("height:v1:chair-1",), (), ())

    write_frame_record(
        tmp_path / "frame-000001", frame, "recording", 1, [proposal], [result], _GroundTruth(), {}
    )

    case_dir = tmp_path / "frame-000001" / "cases" / proposal.id
    public_case = json.loads((case_dir / "case.json").read_text())
    private_oracle = json.loads((case_dir / "private" / "oracle.json").read_text())
    case = SingleFrameVqaEvaluationCase.model_validate(public_case)
    oracle = SingleFrameVqaOracle.model_validate(private_oracle)
    assert public_case == {
        "schema_version": "1.0",
        "case_id": "frame-1-chair-height",
        "kind": "single_frame_vqa",
        "image_path": "image.jpg",
        "question": "How tall is the chair?",
        "answer_kind": "numeric",
        "unit": "m",
        "tolerance": 0.1,
        "answer_marker": "ANSWER:",
    }
    assert case.allowed_answers is None
    assert private_oracle["expected_answer"] == 1.2
    assert private_oracle["tolerance"] == 0.1
    assert oracle.expected_answer == 1.2
    assert "expected_answer" not in json.dumps(public_case)
