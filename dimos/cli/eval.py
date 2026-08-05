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

"""Canonical single-case evaluation CLI."""

import os
from pathlib import Path
import threading
from typing import Literal

import typer

from dimos.benchmark.agent_eval.progress import (
    AssistantTextProgress,
    CaseHeaderProgress,
    EvalProgress,
    FinalResponseProgress,
    StatusProgress,
    ToolEndProgress,
    ToolStartProgress,
)
from dimos.benchmark.agent_eval.single_case import (
    DEFAULT_OPENAI_API_KEY_ENV,
    CodexOAuthConfig,
    CompactEvalResult,
    EvalRunConfig,
    OpenAIApiKeyConfig,
    PiAgentConfig,
    execute_single_case,
)

app = typer.Typer(help="Run immutable agent evaluation cases", no_args_is_help=True)


@app.command("run")
def run(
    case: Path = typer.Argument(..., exists=True, dir_okay=False, readable=True),
    agent_backend: Literal["pi"] = typer.Option("pi", "--agent.backend"),
    agent_model: Literal["gpt-5.6-luna"] = typer.Option("gpt-5.6-luna", "--agent.model"),
    thinking_level: Literal["medium"] = typer.Option("medium", "--agent.thinking-level"),
    auth_mode: Literal["codex-oauth", "openai-api-key"] | None = typer.Option(
        None,
        "--agent.auth.mode",
        help="Auth mode; inferred from auth options or OPENAI_API_KEY when omitted",
    ),
    auth_path: Path | None = typer.Option(None, "--agent.auth.path"),
    auth_env: str | None = typer.Option(None, "--agent.auth.env"),
    output: Path | None = typer.Option(None, "--output"),
    json_output: bool = typer.Option(False, "--json", help="Print compact JSON"),
    quiet: bool = typer.Option(False, "--quiet", help="Suppress live evaluation progress"),
) -> None:
    """Run one static evaluation case synchronously."""
    if auth_mode is None:
        if auth_path is not None and auth_env is not None:
            raise typer.BadParameter(
                "--agent.auth.path and --agent.auth.env select different auth modes"
            )
        if auth_path is not None:
            auth_mode = "codex-oauth"
        elif auth_env is not None or os.environ.get(DEFAULT_OPENAI_API_KEY_ENV):
            auth_mode = "openai-api-key"
        else:
            auth_mode = "codex-oauth"
    if auth_mode == "codex-oauth":
        if auth_env is not None:
            raise typer.BadParameter("--agent.auth.env requires --agent.auth.mode=openai-api-key")
        auth = CodexOAuthConfig(path=auth_path)
    else:
        if auth_path is not None:
            raise typer.BadParameter("--agent.auth.path requires --agent.auth.mode=codex-oauth")
        auth = OpenAIApiKeyConfig(env=auth_env or DEFAULT_OPENAI_API_KEY_ENV)
    config = EvalRunConfig(
        agent=PiAgentConfig(
            backend=agent_backend,
            model=agent_model,
            thinking_level=thinking_level,
            auth=auth,
        )
    )
    renderer = None if quiet else ProgressRenderer()
    try:
        result = (
            execute_single_case(case, config=config, progress=renderer)
            if output is None
            else execute_single_case(
                case,
                config=config,
                output_root=output,
                progress=renderer,
            )
        )
    except Exception as exc:
        if renderer is not None:
            renderer.finish()
        typer.echo(f"Evaluation preflight failed: {type(exc).__name__}: {exc}", err=True)
        raise typer.Exit(2) from exc
    if renderer is not None:
        renderer.finish()
    typer.echo(result.model_dump_json() if json_output else format_result(result))
    if result.attempt_status == "failed":
        raise typer.Exit(1)


def format_result(result: CompactEvalResult) -> str:
    """Render the compact typed result without exposing private oracle material."""
    if result.attempt_status == "failed":
        heading = "! Evaluation not evaluated"
    elif result.task_result == "passed":
        heading = "✓ Evaluation passed"
    else:
        heading = "✗ Evaluation failed"
    source = result.source
    if result.progress is not None:
        source += f" @ {result.progress * 100:g}%"
    answer = str(result.integer_answer) if result.integer_answer is not None else "—"
    rows = (
        ("Case", result.case_id),
        ("Source", source),
        ("Question", result.question),
        ("Answer", answer),
        ("Result", result.task_result),
        (
            "Agent",
            f"{result.agent.agent_id} · {result.agent.model} · {result.agent.thinking_level}",
        ),
        ("Tool calls", str(result.tool_call_count)),
        ("Duration", f"{result.duration_seconds:.1f}s"),
        ("Attempt", result.attempt_id),
        ("Artifacts", str(result.artifact_path)),
    )
    body = "\n".join(f"  {label:<10} {value}" for label, value in rows)
    return f"{heading}\n\n{body}"


class ProgressRenderer:
    """Thread-safe concise terminal renderer for best-effort evaluation progress."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._assistant_open = False
        self._saw_assistant_text = False

    def __call__(self, event: EvalProgress) -> None:
        with self._lock:
            if isinstance(event, AssistantTextProgress):
                if not self._assistant_open:
                    typer.echo("[pi] ", err=True, nl=False)
                    self._assistant_open = True
                typer.echo(event.delta, err=True, nl=False)
                self._saw_assistant_text = True
                return
            self._end_assistant_line()
            if isinstance(event, CaseHeaderProgress):
                source = event.source
                if event.progress is not None:
                    source += f" @ {event.progress * 100:g}%"
                typer.echo("[eval] Session", err=True)
                typer.echo(f"  {'Case':<10} {event.case_id}", err=True)
                typer.echo(f"  {'Source':<10} {source}", err=True)
                typer.echo(f"  {'Question':<10} {event.question}", err=True)
                typer.echo(f"  {'Answer':<10} pending", err=True)
            elif isinstance(event, StatusProgress):
                typer.echo(f"[{event.channel}] {event.message}", err=True)
            elif isinstance(event, ToolStartProgress):
                typer.echo("[python_exec] call", err=True)
                typer.echo(_indent(event.code), err=True)
            elif isinstance(event, ToolEndProgress):
                status = "ok" if event.ok else "error"
                typer.echo(
                    f"[python_exec] {status} ({event.duration_seconds:.1f}s)",
                    err=True,
                )
                if event.result:
                    typer.echo(_indent(event.result), err=True)
            elif isinstance(event, FinalResponseProgress) and not self._saw_assistant_text:
                typer.echo(f"[pi] {event.text}", err=True)

    def finish(self) -> None:
        with self._lock:
            self._end_assistant_line()

    def _end_assistant_line(self) -> None:
        if self._assistant_open:
            typer.echo("", err=True)
            self._assistant_open = False


def _indent(value: str) -> str:
    return "\n".join(f"  {line}" for line in value.splitlines())
