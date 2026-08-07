# Copyright 2026 Dimensional Inc.

from __future__ import annotations

import json
from pathlib import Path

import cv2
import numpy as np

from dimos.benchmark.vqa.generation.dataset import write_frame_record
from dimos.benchmark.vqa.models import (
    CalibratedFrame,
    GroundTruthResult,
    QuestionIntent,
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
