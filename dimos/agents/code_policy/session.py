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

"""Persistent IPython execution semantics for code policies."""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
from dataclasses import asdict, dataclass
import io
import time
from typing import Any, cast

from IPython.core.interactiveshell import InteractiveShell
from traitlets.config import Config

DEFAULT_OUTPUT_LIMIT = 32_000
_TRUNCATION_MARKER = "\n... [output truncated]"


@dataclass(frozen=True)
class CellResult:
    """Serializable result of one policy cell."""

    execution_count: int
    success: bool
    duration_s: float
    stdout: str
    stderr: str
    error_type: str | None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class PolicySession:
    """One persistent IPython namespace containing native DimOS handles."""

    def __init__(
        self,
        *,
        app: Any,
        memory: Any,
        output_limit: int = DEFAULT_OUTPUT_LIMIT,
    ) -> None:
        self._output_limit = output_limit
        config = Config()
        config.HistoryManager.enabled = False
        self._shell = InteractiveShell(  # type: ignore[no-untyped-call]
            user_ns={"app": app, "memory": memory},
            config=config,
        )
        self._shell.colors = "nocolor"

    @property
    def user_ns(self) -> dict[str, Any]:
        """Return the IPython user namespace for inspection and tests."""
        return cast("dict[str, Any]", self._shell.user_ns)

    def execute(self, code: str) -> CellResult:
        """Execute one cell while preserving the namespace for later calls."""
        stdout = io.StringIO()
        stderr = io.StringIO()
        started = time.monotonic()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            result = self._shell.run_cell(  # type: ignore[no-untyped-call]
                code,
                store_history=True,
            )
        duration_s = time.monotonic() - started

        error = result.error_before_exec or result.error_in_exec
        execution_count = result.execution_count
        if execution_count is None:
            execution_count = max(1, self._shell.execution_count - 1)

        return CellResult(
            execution_count=execution_count,
            success=result.success,
            duration_s=duration_s,
            stdout=_truncate(stdout.getvalue(), self._output_limit),
            stderr=_truncate(stderr.getvalue(), self._output_limit),
            error_type=type(error).__name__ if error is not None else None,
        )


def format_cell_result(result: CellResult) -> str:
    """Format a cell result as a compact, bounded REPL transcript."""
    state = "completed" if result.success else "failed"
    sections = [f"In [{result.execution_count}] {state} in {result.duration_s:.2f}s"]
    if result.stdout:
        sections.extend(["", "stdout:", result.stdout.rstrip()])
    if result.stderr:
        sections.extend(["", "stderr:", result.stderr.rstrip()])
    if not result.stdout and not result.stderr:
        sections.extend(["", "(completed)" if result.success else f"({result.error_type})"])
    return "\n".join(sections)


def _truncate(value: str, limit: int) -> str:
    if len(value) <= limit:
        return value
    keep = max(0, limit - len(_TRUNCATION_MARKER))
    return value[:keep] + _TRUNCATION_MARKER
