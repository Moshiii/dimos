# Copyright 2026 Dimensional Inc.

from __future__ import annotations

import pytest

from dimos.benchmark.vqa.evaluation.models import (
    EvaluationRunConfig,
    LangChainVisionEvaluationConfig,
)
from dimos.benchmark.vqa.evaluation.persistence import EvaluationRunStore, dataset_identity


def _config() -> EvaluationRunConfig:
    return EvaluationRunConfig(
        model="model", langchain=LangChainVisionEvaluationConfig(model="model"), workers=1
    )


def test_dataset_identity_hashes_only_public_case_records(tmp_path) -> None:
    case = tmp_path / "frame-000001" / "cases" / "case-1"
    (case / "private").mkdir(parents=True)
    (case / "case.json").write_text('{"case_id":"case-1"}')
    (case / "private" / "oracle.json").write_text('{"expected_answer":"left"}')

    before = dataset_identity(tmp_path)
    (case / "private" / "oracle.json").write_text('{"expected_answer":"right"}')

    assert dataset_identity(tmp_path) == before


def test_resume_requires_matching_public_dataset_and_config(tmp_path) -> None:
    (tmp_path / "frame-000001" / "cases" / "case-1").mkdir(parents=True)
    (tmp_path / "frame-000001" / "cases" / "case-1" / "case.json").write_text("{}")
    store = EvaluationRunStore.create(tmp_path / "run", tmp_path, _config())
    store.close()

    resumed = EvaluationRunStore.resume(tmp_path / "run", tmp_path, _config())
    resumed.close()
    with pytest.raises(ValueError, match="incompatible"):
        EvaluationRunStore.resume(
            tmp_path / "run",
            tmp_path,
            EvaluationRunConfig(
                model="other", langchain=LangChainVisionEvaluationConfig(model="other"), workers=1
            ),
        )
    assert "expected_answer" not in (tmp_path / "run" / "run-manifest.json").read_text()
