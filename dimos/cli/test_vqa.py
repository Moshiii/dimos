# Copyright 2026 Dimensional Inc.

from __future__ import annotations

from typer.testing import CliRunner

from dimos.cli import vqa


def test_evaluate_requires_openai_api_key_for_gpt5(tmp_path, monkeypatch) -> None:
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)

    result = CliRunner().invoke(vqa.app, ["evaluate", "--dataset", str(tmp_path)])

    assert result.exit_code == 2
    assert "OPENAI_API_KEY must be set" in result.output


def test_evaluate_writes_null_accuracy_for_no_cases(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "test-key")
    monkeypatch.setattr(vqa, "evaluate_dataset", lambda *_: [])

    result = CliRunner().invoke(vqa.app, ["evaluate", "--dataset", str(tmp_path)])

    assert result.exit_code == 0
    assert '"accuracy": null' in (tmp_path / "evaluations" / "gpt-5.6-luna" / "summary.json").read_text()
