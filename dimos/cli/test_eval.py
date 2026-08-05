# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import builtins
import json
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest
from typer.testing import CliRunner

from dimos.benchmark.agent_eval.case import AgentCondition
from dimos.benchmark.agent_eval.progress import (
    AssistantTextProgress,
    CaseHeaderProgress,
    StatusProgress,
    ToolEndProgress,
    ToolStartProgress,
)
from dimos.benchmark.agent_eval.single_case import CompactEvalResult
from dimos.cli.dimos import main
import dimos.cli.eval as eval_cli


def _result(
    tmp_path: Path, *, status: str = "completed", task: str = "passed"
) -> CompactEvalResult:
    return CompactEvalResult(
        attempt_id="attempt_" + "a" * 32,
        case_id="hongkong-room-count",
        source="go2_hongkong_office",
        progress=1.0,
        question="How many rooms in total?",
        attempt_status=status,
        task_result=task,
        reason="validator passed" if status == "completed" else "infrastructure failed",
        prediction_status="parsed" if status == "completed" else None,
        integer_answer=4 if status == "completed" else None,
        agent=AgentCondition(
            agent_id="pi-code-policy",
            adapter="pi-node",
            model="gpt-5.6-luna",
            thinking_level="medium",
        ),
        tool_call_count=7,
        duration_seconds=42.75,
        artifact_path=tmp_path / "attempt",
    )


def _case(tmp_path: Path) -> Path:
    path = tmp_path / "case.json"
    path.write_text("{}")
    return path


def test_eval_run_uses_typed_defaults_and_separates_progress(tmp_path, monkeypatch) -> None:
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    captured = {}

    def execute(path, *, config, progress, **kwargs):
        captured.update(path=path, config=config, progress=progress, kwargs=kwargs)
        progress(StatusProgress(channel="eval", message="loading case"))
        progress(
            CaseHeaderProgress(
                case_id="hongkong-room-count",
                source="go2_hongkong_office",
                progress=1.0,
                question="How many rooms in total?",
            )
        )
        progress(AssistantTextProgress(delta="Inspecting memory"))
        progress(ToolStartProgress(code="memory.streams()"))
        progress(ToolEndProgress(ok=True, result="['lidar']", duration_seconds=0.25))
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)
    result = CliRunner().invoke(main, ["eval", "run", str(_case(tmp_path))])

    assert result.exit_code == 0, result.output
    assert captured["config"].agent.auth.mode == "codex-oauth"
    assert "✓ Evaluation passed" in result.stdout
    assert "go2_hongkong_office @ 100%" in result.stdout
    assert "[eval] loading case" in result.stderr
    assert "[pi] Inspecting memory" in result.stderr
    assert "[python_exec] ok (0.2s)" in result.stderr


def test_eval_run_auth_inference_and_explicit_precedence(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "credential-sentinel")
    captured = []

    def execute(*args, **kwargs):
        captured.append(kwargs["config"])
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)
    runner = CliRunner()
    automatic = runner.invoke(main, ["eval", "run", str(_case(tmp_path))])
    explicit = runner.invoke(
        main,
        ["eval", "run", str(_case(tmp_path)), "--agent.auth.mode=codex-oauth"],
    )

    assert automatic.exit_code == explicit.exit_code == 0
    assert captured[0].agent.auth.mode == "openai-api-key"
    assert captured[1].agent.auth.mode == "codex-oauth"
    assert "credential-sentinel" not in automatic.output + explicit.output


def test_eval_run_accepts_dotted_options_and_json(tmp_path, monkeypatch) -> None:
    captured = {}

    def execute(*args, **kwargs):
        captured.update(kwargs)
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)
    output = tmp_path / "results"
    result = CliRunner().invoke(
        main,
        [
            "eval",
            "run",
            str(_case(tmp_path)),
            "--agent.backend=pi",
            "--agent.model=gpt-5.6-luna",
            "--agent.auth.mode=openai-api-key",
            "--agent.auth.env=MY_OPENAI_KEY",
            f"--output={output}",
            "--json",
        ],
    )

    assert result.exit_code == 0, result.output
    assert captured["config"].agent.auth.env == "MY_OPENAI_KEY"
    assert captured["output_root"] == output
    assert json.loads(result.stdout)["task_result"] == "passed"
    assert "private" not in result.stdout


def test_eval_run_quiet_and_exit_codes(tmp_path, monkeypatch) -> None:
    observed = []

    def failed_attempt(*args, **kwargs):
        observed.append(kwargs["progress"])
        return _result(tmp_path, status="failed", task="not_evaluated")

    monkeypatch.setattr(eval_cli, "execute_single_case", failed_attempt)
    failed = CliRunner().invoke(main, ["eval", "run", str(_case(tmp_path)), "--quiet"])

    def preflight(*args, **kwargs):
        raise FileNotFoundError("adapter build missing")

    monkeypatch.setattr(eval_cli, "execute_single_case", preflight)
    preflight_result = CliRunner().invoke(main, ["eval", "run", str(_case(tmp_path))])

    assert failed.exit_code == 1
    assert observed == [None]
    assert failed.stderr == ""
    assert preflight_result.exit_code == 2
    assert "Evaluation preflight failed: FileNotFoundError" in preflight_result.stderr


def test_eval_rejects_invalid_auth_combinations_and_semantic_override(tmp_path) -> None:
    runner = CliRunner()
    case = _case(tmp_path)
    conflicting = runner.invoke(
        main,
        ["eval", "run", str(case), "--agent.auth.path=x", "--agent.auth.env=Y"],
    )
    semantic = runner.invoke(main, ["eval", "run", str(case), "--source.recording=other"])

    assert conflicting.exit_code == 2
    assert "select different auth" in conflicting.stderr
    assert "modes" in conflicting.stderr
    assert semantic.exit_code == 2
    assert "No such option" in semantic.stderr


def test_eval_help_is_typed_and_rejects_unsupported_model(tmp_path) -> None:
    runner = CliRunner()
    help_result = runner.invoke(main, ["eval", "run", "--help"])
    unsupported = runner.invoke(
        main,
        ["eval", "run", str(_case(tmp_path)), "--agent.model=unreviewed-model"],
    )

    assert help_result.exit_code == 0
    assert "--agent.model" in help_result.stdout
    assert "gpt-5.6-luna" in help_result.stdout
    assert "--agent.thinking-level" in help_result.stdout
    assert unsupported.exit_code == 2
    assert "unreviewed-model" in unsupported.stderr


def test_eval_semantic_failure_is_a_successful_attempt(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(
        eval_cli,
        "execute_single_case",
        lambda *args, **kwargs: _result(tmp_path, status="completed", task="failed"),
    )

    result = CliRunner().invoke(main, ["eval", "run", str(_case(tmp_path))])

    assert result.exit_code == 0
    assert "Evaluation failed" in result.stdout


def test_lazy_runtime_import_has_actionable_missing_agents_error(monkeypatch) -> None:
    original_import = builtins.__import__

    def fail_single_case(name, *args, **kwargs):
        if name == "dimos.benchmark.agent_eval.single_case":
            raise ModuleNotFoundError("No module named 'fastapi'")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fail_single_case)

    with pytest.raises(RuntimeError, match="uv sync --extra agents"):
        eval_cli.execute_single_case(Path("case.json"), config=None)


def test_base_cli_help_imports_without_agents_only_modules() -> None:
    script = textwrap.dedent(
        """
        import sys

        class BlockAgentsImports:
            def find_spec(self, fullname, path=None, target=None):
                if fullname.split('.')[0] in {
                    'fastapi', 'ipykernel', 'jupyter_client', 'uvicorn'
                }:
                    raise RuntimeError(f'agents-only import attempted: {fullname}')
                return None

        sys.meta_path.insert(0, BlockAgentsImports())
        from typer.testing import CliRunner
        from dimos.cli.dimos import main

        result = CliRunner().invoke(main, ['--help'])
        assert result.exit_code == 0, result.output
        assert 'eval' in result.stdout
        """
    )

    completed = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stdout + completed.stderr
