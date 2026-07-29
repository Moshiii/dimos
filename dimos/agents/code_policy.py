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

"""Agent-authored Python execution backed by a persistent Jupyter kernel."""

from __future__ import annotations

import os
import re
import threading
import time
from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MAX_EXECUTION_TIMEOUT_S = 110.0
DEFAULT_STARTUP_TIMEOUT_S = 10.0
DEFAULT_INTERRUPT_GRACE_S = 2.0
DEFAULT_OUTPUT_LIMIT = 32_000
_RECORDING_PATH_ENV = "DIMOS_CODE_POLICY_RECORDING_PATH"
_TRUNCATION_MARKER = "\n... [output truncated]"
_ANSI_ESCAPE_RE = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


class CodePolicyConfig(ModuleConfig):
    recording_path: str
    output_limit: int = DEFAULT_OUTPUT_LIMIT
    startup_timeout_s: float = DEFAULT_STARTUP_TIMEOUT_S
    interrupt_grace_s: float = DEFAULT_INTERRUPT_GRACE_S


class _BoundedTextOutput:
    """Collect the plain-text portions of Jupyter output messages."""

    def __init__(self, limit: int) -> None:
        self._limit = max(0, limit)
        self._parts: list[str] = []
        self._length = 0
        self._truncated = False

    def __call__(self, message: dict[str, Any]) -> None:
        message_type = message.get("header", {}).get("msg_type")
        content = message.get("content", {})
        if message_type == "stream":
            self._append(str(content.get("text", "")))
        elif message_type in {"execute_result", "display_data"}:
            text = content.get("data", {}).get("text/plain")
            if text is not None:
                self._append(str(text))
        elif message_type == "error":
            traceback = content.get("traceback")
            if isinstance(traceback, list):
                self._append("\n".join(str(line) for line in traceback))
            else:
                self._append(f"{content.get('ename', 'Error')}: {content.get('evalue', '')}")

    def text(self) -> str:
        return "".join(self._parts)

    def _append(self, value: str) -> None:
        if not value or self._truncated:
            return
        value = _ANSI_ESCAPE_RE.sub("", value)
        remaining = self._limit - self._length
        if len(value) <= remaining:
            self._parts.append(value)
            self._length += len(value)
            return

        marker = _TRUNCATION_MARKER[:remaining]
        content_limit = max(0, remaining - len(marker))
        self._parts.append(value[:content_limit] + marker)
        self._length = self._limit
        self._truncated = True


def _load_kernel_manager() -> type[Any]:
    try:
        from jupyter_client.manager import KernelManager
    except ImportError as exc:
        raise RuntimeError(
            "Code-policy execution requires the agents extra: uv sync --extra agents"
        ) from exc
    return KernelManager


def _bootstrap_source() -> str:
    return f"""
import os as _os
from dimos.memory2.store.sqlite import SqliteStore as _SqliteStore
from dimos.porcelain.dimos import Dimos as _Dimos

app = _Dimos.connect()
memory = _SqliteStore(path=_os.environ[{_RECORDING_PATH_ENV!r}], must_exist=True)
memory.start()

del _os, _SqliteStore, _Dimos
"""


class CodePolicyModule(Module):
    """Execute trusted agent-authored Python against a running DimOS system."""

    config: CodePolicyConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Runtime objects remain None until this instance has been deployed into
        # its worker, so Module serialization needs no custom hooks.
        self._execution_lock: threading.Lock | None = None
        self._kernel_manager: Any = None
        self._kernel_client: Any = None

    @rpc
    def start(self) -> None:
        self._execution_lock = threading.Lock()
        super().start()

    @skill
    def python_exec(self, code: str, timeout_s: float = MAX_EXECUTION_TIMEOUT_S) -> str:
        """Execute one synchronous Python program in the persistent policy session.

        The session preloads `app` for deployed DimOS RPCs and `memory` for
        current and historical memory2 observations. Imports, functions, and
        variables persist across calls. Use this for observation processing,
        control flow, retries, and coordinated multi-RPC behavior.

        Args:
            code: Complete Python source for one task attempt.
            timeout_s: Execution deadline in seconds, greater than 0 and at most 110.
        """
        if not 0 < timeout_s <= MAX_EXECUTION_TIMEOUT_S:
            return (
                f"Invalid timeout_s={timeout_s!r}; expected a value in "
                f"(0, {MAX_EXECUTION_TIMEOUT_S:g}]"
            )

        lock = self._execution_lock
        if lock is None:
            lock = self._execution_lock = threading.Lock()
        if not lock.acquire(blocking=False):
            return "Code Policy Module busy: another python_exec call is active"

        started_at = time.monotonic()
        logger.info(
            "Code policy execution started",
            source=code,
            timeout_s=timeout_s,
        )
        try:
            try:
                client = self._ensure_kernel()
            except Exception as exc:
                logger.exception("Code policy kernel failed to start")
                return f"Code policy kernel failed to start: {type(exc).__name__}: {exc}"

            output = _BoundedTextOutput(self.config.output_limit)
            try:
                reply = client.execute_interactive(
                    code,
                    allow_stdin=False,
                    output_hook=output,
                    store_history=True,
                    timeout=timeout_s,
                )
            except TimeoutError:
                preserved = self._recover_from_timeout()
                duration_s = time.monotonic() - started_at
                logger.warning(
                    "Code policy execution timed out",
                    duration_s=duration_s,
                    namespace_preserved=preserved,
                    timeout_s=timeout_s,
                )
                if preserved:
                    return (
                        f"Execution timed out after {timeout_s:.1f}s and was interrupted. "
                        "The Python namespace was preserved. Remote RPC work may still be "
                        "running; it was not cancelled."
                    )
                return (
                    f"Execution timed out after {timeout_s:.1f}s. The kernel did not "
                    "recover from interruption and was restarted; the Python namespace "
                    "was reset. Remote RPC work may still be running; it was not cancelled."
                )
            except Exception as exc:
                self._shutdown_kernel(reason=type(exc).__name__)
                logger.exception("Code policy execution failed")
                return (
                    f"Code policy execution failed: {type(exc).__name__}: {exc}. "
                    "The Python namespace was reset."
                )

            duration_s = time.monotonic() - started_at
            transcript = _format_reply(reply, output.text(), duration_s)
            content = reply.get("content", {})
            logger.info(
                "Code policy execution finished",
                duration_s=duration_s,
                execution_count=content.get("execution_count"),
                message_id=reply.get("parent_header", {}).get("msg_id"),
                output=output.text(),
                status=content.get("status"),
            )
            return transcript
        finally:
            lock.release()

    @rpc
    def stop(self) -> None:
        self._shutdown_kernel(reason="module stop")
        super().stop()

    def _ensure_kernel(self) -> Any:
        manager = self._kernel_manager
        client = self._kernel_client
        if manager is not None and client is not None and manager.is_alive():
            return client
        self._shutdown_kernel(reason="kernel unavailable")

        manager_type = _load_kernel_manager()
        manager = manager_type(kernel_name="python3")
        client = None
        try:
            env = os.environ.copy()
            env["DIMOS_TRANSPORT"] = self.config.g.transport
            env[_RECORDING_PATH_ENV] = self.config.recording_path
            manager.start_kernel(env=env)
            client = manager.client()
            client.start_channels()
            client.wait_for_ready(timeout=self.config.startup_timeout_s)
            self._bootstrap(client)
        except Exception:
            if client is not None:
                client.stop_channels()
            try:
                manager.shutdown_kernel(now=True)
            except Exception:
                logger.exception("Failed to stop an uninitialized code policy kernel")
            raise

        self._kernel_manager = manager
        self._kernel_client = client
        logger.info(
            "Code policy kernel started",
            recording_path=self.config.recording_path,
        )
        return client

    def _bootstrap(self, client: Any) -> None:
        reply = client.execute_interactive(
            _bootstrap_source(),
            allow_stdin=False,
            output_hook=lambda _message: None,
            silent=True,
            store_history=False,
            timeout=self.config.startup_timeout_s,
        )
        content = reply.get("content", {})
        if content.get("status") != "ok":
            name = content.get("ename", "KernelBootstrapError")
            value = content.get("evalue", "unknown bootstrap failure")
            raise RuntimeError(f"{name}: {value}")

    def _recover_from_timeout(self) -> bool:
        manager = self._kernel_manager
        client = self._kernel_client
        if manager is None or client is None:
            return False
        try:
            manager.interrupt_kernel()
            client.wait_for_ready(timeout=self.config.interrupt_grace_s)
            logger.info("Code policy kernel recovered after interrupt")
            return True
        except Exception:
            logger.warning("Code policy kernel did not recover after interrupt")

        try:
            manager.restart_kernel(now=True)
            client.wait_for_ready(timeout=self.config.startup_timeout_s)
            self._bootstrap(client)
            logger.info("Code policy kernel restarted after failed interrupt")
        except Exception:
            logger.exception("Code policy kernel failed to restart")
            self._shutdown_kernel(reason="restart failure")
        return False

    def _shutdown_kernel(self, *, reason: str) -> None:
        manager = self._kernel_manager
        client = self._kernel_client
        self._kernel_manager = None
        self._kernel_client = None
        if manager is None and client is None:
            return
        try:
            if manager is not None:
                manager.shutdown_kernel(now=True)
        except Exception:
            logger.exception("Failed to stop code policy kernel")
        finally:
            if client is not None:
                client.stop_channels()
        logger.info("Code policy kernel stopped", reason=reason)


def _format_reply(reply: dict[str, Any], output: str, duration_s: float) -> str:
    content = reply.get("content", {})
    status = content.get("status", "unknown")
    execution_count = content.get("execution_count", "?")
    state = "completed" if status == "ok" else "failed"
    body = output.rstrip()
    if not body and status != "ok":
        body = f"{content.get('ename', 'Error')}: {content.get('evalue', '')}".rstrip()
    if not body:
        body = "(completed)"
    return f"In [{execution_count}] {state} in {duration_s:.2f}s\n\n{body}"


code_policy_module = CodePolicyModule.blueprint
