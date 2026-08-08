# Copyright 2026 Dimensional Inc.

from __future__ import annotations

from typer.testing import CliRunner

from dimos.cli import vqa


def test_evaluate_requires_openai_api_key_for_gpt5(tmp_path, monkeypatch) -> None:
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)

    result = CliRunner().invoke(vqa.app, ["evaluate", "--dataset", str(tmp_path), "--no-viewer"])

    assert result.exit_code == 2
    assert "OPENAI_API_KEY must be set" in result.output


def test_evaluate_rejects_existing_output_without_resume(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    output = tmp_path / "run"
    output.mkdir()

    result = CliRunner().invoke(
        vqa.app, ["evaluate", "--dataset", str(tmp_path), "--output", str(output), "--no-viewer"]
    )

    assert result.exit_code == 2
    assert "run output already exists" in result.output
