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

import json
from pathlib import Path

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
from dimos.core.global_config import global_config


def _result(tmp_path: Path) -> CompactEvalResult:
    return CompactEvalResult(
        attempt_id="attempt_" + "a" * 32,
        case_id="hongkong-room-count",
        source="go2_hongkong_office",
        progress=1.0,
        question="How many rooms in total?",
        attempt_status="completed",
        task_result="passed",
        reason="validator passed",
        prediction_status="parsed",
        integer_answer=4,
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


def test_eval_run_uses_typed_defaults_and_pretty_prints(tmp_path, monkeypatch) -> None:
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    case = tmp_path / "case.json"
    case.write_text("{}")
    captured = {}

    def execute(path, *, config, output_root=None, progress=None):
        captured.update(path=path, config=config, output_root=output_root, progress=progress)
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

    result = CliRunner().invoke(main, ["eval", "run", str(case)])

    assert result.exit_code == 0, result.output
    assert captured["config"].agent.backend == "pi"
    assert captured["config"].agent.auth.mode == "codex-oauth"
    assert "✓ Evaluation passed" in result.output
    assert "go2_hongkong_office @ 100%" in result.output
    assert "Answer     4" in result.output
    assert "[eval] loading case" in result.stderr
    assert "[eval] Session" in result.stderr
    assert "Question   How many rooms in total?" in result.stderr
    assert "Answer     pending" in result.stderr
    assert result.stderr.index("Question") < result.stderr.index("[pi]")
    assert "[pi] Inspecting memory" in result.stderr
    assert "[python_exec] ok (0.2s)" in result.stderr


def test_eval_run_auto_selects_openai_key_from_environment(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "test-secret")
    case = tmp_path / "case.json"
    case.write_text("{}")
    captured = {}

    def execute(path, *, config, output_root=None, progress=None):
        captured["config"] = config
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)

    result = CliRunner().invoke(main, ["eval", "run", str(case)])

    assert result.exit_code == 0, result.output
    assert captured["config"].agent.auth.mode == "openai-api-key"
    assert captured["config"].agent.auth.env == "OPENAI_API_KEY"
    assert "test-secret" not in result.output


def test_eval_run_explicit_codex_auth_overrides_openai_environment(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "test-secret")
    case = tmp_path / "case.json"
    case.write_text("{}")
    captured = {}

    def execute(path, *, config, output_root=None, progress=None):
        captured["config"] = config
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)

    result = CliRunner().invoke(
        main,
        ["eval", "run", str(case), "--agent.auth.mode=codex-oauth"],
    )

    assert result.exit_code == 0, result.output
    assert captured["config"].agent.auth.mode == "codex-oauth"


def test_eval_run_auth_env_implies_openai_key_mode(tmp_path, monkeypatch) -> None:
    case = tmp_path / "case.json"
    case.write_text("{}")
    captured = {}

    def execute(path, *, config, output_root=None, progress=None):
        captured["config"] = config
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)

    result = CliRunner().invoke(
        main,
        ["eval", "run", str(case), "--agent.auth.env=MY_OPENAI_KEY"],
    )

    assert result.exit_code == 0, result.output
    assert captured["config"].agent.auth.mode == "openai-api-key"
    assert captured["config"].agent.auth.env == "MY_OPENAI_KEY"


def test_eval_run_accepts_dotted_auth_and_agent_options(tmp_path, monkeypatch) -> None:
    case = tmp_path / "case.json"
    case.write_text("{}")
    captured = {}

    def execute(path, *, config, output_root=None, progress=None):
        captured.update(config=config, output_root=output_root)
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)
    output = tmp_path / "results"

    result = CliRunner().invoke(
        main,
        [
            "eval",
            "run",
            str(case),
            "--agent.backend=pi",
            "--agent.model=gpt-5.6-luna",
            "--agent.auth.mode=openai-api-key",
            "--agent.auth.env=MY_OPENAI_KEY",
            f"--output={output}",
        ],
    )

    assert result.exit_code == 0, result.output
    assert captured["config"].agent.auth.env == "MY_OPENAI_KEY"
    assert captured["output_root"] == output


def test_eval_run_passes_native_render_as_presentation_config(tmp_path, monkeypatch) -> None:
    case = tmp_path / "case.json"
    case.write_text("{}")
    captured = {}

    def execute(path, *, config, output_root=None, progress=None):
        captured["config"] = config
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)

    result = CliRunner().invoke(main, ["eval", "run", str(case), "--render", "native"])

    assert result.exit_code == 0, result.output
    assert captured["config"].render == "native"


def test_eval_run_honors_global_rerun_web_options(tmp_path, monkeypatch) -> None:
    case = tmp_path / "case.json"
    case.write_text("{}")
    observed = {}
    original = global_config.model_dump()

    def execute(path, *, config, output_root=None, progress=None):
        observed.update(
            viewer=global_config.viewer,
            rerun_web=global_config.rerun_web,
            rerun_open=global_config.rerun_open,
        )
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)
    try:
        result = CliRunner().invoke(
            main,
            [
                "--viewer",
                "rerun",
                "--rerun-web",
                "--rerun-open",
                "web",
                "eval",
                "run",
                str(case),
            ],
        )

        assert result.exit_code == 0, result.output
        assert observed == {
            "viewer": "rerun",
            "rerun_web": True,
            "rerun_open": "web",
        }
    finally:
        global_config.update(**original)


def test_eval_run_is_headless_when_viewer_is_not_explicit(tmp_path, monkeypatch) -> None:
    case = tmp_path / "case.json"
    case.write_text("{}")
    observed = {}
    original = global_config.model_dump()

    def execute(path, *, config, output_root=None, progress=None):
        observed["viewer"] = global_config.viewer
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)
    try:
        global_config.update(viewer="rerun")
        result = CliRunner().invoke(main, ["eval", "run", str(case)])

        assert result.exit_code == 0, result.output
        assert observed["viewer"] == "none"
    finally:
        global_config.update(**original)


def test_eval_run_json_is_one_compact_pydantic_result(tmp_path, monkeypatch) -> None:
    case = tmp_path / "case.json"
    case.write_text("{}")

    def execute(*args, **kwargs):
        kwargs["progress"](StatusProgress(channel="pi", message="session started"))
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)

    result = CliRunner().invoke(main, ["eval", "run", str(case), "--json"])

    assert result.exit_code == 0, result.output
    payload = json.loads(result.stdout)
    assert payload["attempt_status"] == "completed"
    assert payload["task_result"] == "passed"
    assert payload["integer_answer"] == 4
    assert "private" not in result.stdout
    assert "[pi] session started" in result.stderr


def test_eval_run_quiet_suppresses_live_progress(tmp_path, monkeypatch) -> None:
    case = tmp_path / "case.json"
    case.write_text("{}")
    captured = {}

    def execute(*args, **kwargs):
        captured["progress"] = kwargs["progress"]
        return _result(tmp_path)

    monkeypatch.setattr(eval_cli, "execute_single_case", execute)

    result = CliRunner().invoke(main, ["eval", "run", str(case), "--quiet"])

    assert result.exit_code == 0, result.output
    assert captured["progress"] is None
    assert result.stderr == ""


def test_eval_run_rejects_semantic_source_override(tmp_path) -> None:
    case = tmp_path / "case.json"
    case.write_text("{}")

    result = CliRunner().invoke(main, ["eval", "run", str(case), "--source.recording=other"])

    assert result.exit_code != 0
    assert "No such option" in result.output
