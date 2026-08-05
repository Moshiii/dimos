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

"""Launch the pinned stock Pi CLI and parse its native JSON event stream."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import subprocess
import time

PI_VERSION = "0.80.10"
MAX_STDERR_BYTES = 64 * 1024


@dataclass(frozen=True)
class PiRunResult:
    final_text: str
    tool_call_count: int
    duration_seconds: float
    transcript_path: Path | None
    stderr: str


class PiRunError(RuntimeError):
    def __init__(self, message: str, *, stderr: str = "") -> None:
        super().__init__(message)
        self.stderr = stderr


class PiCliRunner:
    """Thin stock-CLI binding; the extension owns only the `python_exec` tool."""

    def __init__(
        self,
        *,
        cli: Path,
        extension: Path,
        model: str,
        thinking_level: str,
        timeout_s: float,
    ) -> None:
        if not cli.is_file():
            raise FileNotFoundError(f"Pi {PI_VERSION} CLI is not installed: {cli}")
        if not extension.is_file():
            raise FileNotFoundError(
                f"Pi CodePolicy extension is not built: {extension}; "
                "run `npm run build --prefix packages/pi-code-policy-extension`"
            )
        self.cli = cli
        self.extension = extension
        self.model = model
        self.thinking_level = thinking_level
        self.timeout_s = timeout_s

    def run(
        self,
        *,
        prompt: str,
        system_prompt: str,
        mcp_url: str,
        api_key: str,
        run_dir: Path,
    ) -> PiRunResult:
        session_dir = run_dir / "pi-session"
        agent_dir = run_dir / ".pi-agent"
        system_prompt_path = run_dir / "system-prompt.txt"
        system_prompt_path.write_text(system_prompt, encoding="utf-8")
        command = (
            "node",
            str(self.cli),
            "--mode",
            "json",
            "--model",
            f"openai/{self.model}",
            "--thinking",
            self.thinking_level,
            "--session-dir",
            str(session_dir),
            "--name",
            "dimos-frozen-eval",
            "--no-builtin-tools",
            "--tools",
            "python_exec",
            "--no-extensions",
            "--extension",
            str(self.extension),
            "--no-skills",
            "--no-prompt-templates",
            "--no-themes",
            "--no-context-files",
            "--no-approve",
            "--system-prompt",
            str(system_prompt_path),
            prompt,
        )
        env = {
            "PATH": os.environ.get("PATH", ""),
            "OPENAI_API_KEY": api_key,
            "DIMOS_CODE_POLICY_MCP_URL": mcp_url,
            "PI_CODING_AGENT_DIR": str(agent_dir),
            "PI_SKIP_VERSION_CHECK": "1",
            "PI_TELEMETRY": "0",
        }
        started = time.monotonic()
        process = subprocess.Popen(
            command,
            cwd=run_dir,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            stdout, stderr = process.communicate(timeout=self.timeout_s)
        except subprocess.TimeoutExpired as exc:
            process.terminate()
            try:
                stdout, stderr = process.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                stdout, stderr = process.communicate()
            raise PiRunError(
                f"Pi timed out after {self.timeout_s:g}s",
                stderr=_bounded_stderr(stderr),
            ) from exc
        duration = time.monotonic() - started
        stderr = _bounded_stderr(stderr)
        final_text, tool_count, stop_error = parse_pi_events(stdout)
        if process.returncode != 0:
            raise PiRunError(f"Pi exited with status {process.returncode}", stderr=stderr)
        if stop_error is not None:
            raise PiRunError(stop_error, stderr=stderr)
        if final_text is None:
            raise PiRunError("Pi produced no final assistant response", stderr=stderr)
        transcripts = sorted(session_dir.rglob("*.jsonl")) if session_dir.exists() else []
        return PiRunResult(
            final_text=final_text,
            tool_call_count=tool_count,
            duration_seconds=duration,
            transcript_path=transcripts[-1] if transcripts else None,
            stderr=stderr,
        )


def parse_pi_events(stream: str) -> tuple[str | None, int, str | None]:
    """Return the final assistant text, tool count, and terminal error."""
    final_text: str | None = None
    tool_count = 0
    stop_error: str | None = None
    for line in stream.splitlines():
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(event, dict):
            continue
        if event.get("type") == "tool_execution_start":
            tool_count += 1
        if event.get("type") != "message_end":
            continue
        message = event.get("message")
        if not isinstance(message, dict) or message.get("role") != "assistant":
            continue
        text = "".join(
            str(item.get("text", ""))
            for item in message.get("content", [])
            if isinstance(item, dict) and item.get("type") == "text"
        )
        final_text = text
        stop_reason = message.get("stopReason")
        if stop_reason in {"error", "aborted"}:
            stop_error = str(message.get("errorMessage") or f"Pi request {stop_reason}")
    return final_text, tool_count, stop_error


def _bounded_stderr(value: str) -> str:
    encoded = value.encode()
    if len(encoded) <= MAX_STDERR_BYTES:
        return value
    return encoded[:MAX_STDERR_BYTES].decode(errors="ignore")
