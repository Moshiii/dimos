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

"""DimOS module that supervises a persistent agent-operated Python session."""

from __future__ import annotations

from dataclasses import fields
import importlib
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import Any, cast

from dimos.agents.annotation import skill
from dimos.agents.code_policy.session import (
    DEFAULT_OUTPUT_LIMIT,
    CellResult,
    format_cell_result,
)
from dimos.core.core import rpc
from dimos.core.introspection.module.info import ModuleInfo
from dimos.core.module import Module, ModuleConfig
from dimos.memory2.module import Recorder
from dimos.porcelain.dimos import Dimos
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

MAX_EXECUTION_TIMEOUT_S = 110.0
DEFAULT_STARTUP_TIMEOUT_S = 10.0
_CHANNEL_FD_ENV = "DIMOS_POLICY_CHANNEL_FD"


class PolicyKernelConfig(ModuleConfig):
    output_limit: int = DEFAULT_OUTPUT_LIMIT
    startup_timeout_s: float = DEFAULT_STARTUP_TIMEOUT_S


class PolicyWorkerError(RuntimeError):
    """The policy subprocess could not complete a protocol operation."""


class PolicyWorkerTimeoutError(TimeoutError):
    """The policy subprocess did not finish the submitted cell in time."""


def resolve_recorder(app: Dimos) -> tuple[str, str]:
    """Return the unique deployed Recorder instance name and active path."""
    recorders = [info for info in app.list_modules() if _is_recorder(info)]
    if not recorders:
        raise RuntimeError(
            "PolicyKernel requires exactly one deployed memory2 Recorder; found none"
        )
    if len(recorders) > 1:
        names = ", ".join(sorted(_module_instance_name(info) for info in recorders))
        raise RuntimeError(
            f"PolicyKernel requires exactly one deployed memory2 Recorder; found multiple: {names}"
        )
    name = _module_instance_name(recorders[0])
    recorder = cast("Any", app.get_module(name))
    path = recorder.recording_path()
    if not isinstance(path, str) or not path:
        raise RuntimeError(f"Recorder {name!r} returned an invalid recording path")
    return name, path


def _module_instance_name(info: ModuleInfo) -> str:
    return info.instance_name or info.name


def _is_recorder(info: ModuleInfo) -> bool:
    if not info.qualified_path:
        return False
    try:
        module_path, class_name = info.qualified_path.rsplit(".", 1)
        candidate = getattr(importlib.import_module(module_path), class_name)
    except (ImportError, AttributeError, ValueError):
        return False
    return isinstance(candidate, type) and issubclass(candidate, Recorder)


class KernelProcess:
    """Supervise one persistent worker over a private socketpair."""

    def __init__(
        self,
        *,
        recording_path: str,
        transport: str,
        output_limit: int,
        startup_timeout_s: float,
        command: list[str] | None = None,
    ) -> None:
        parent_socket, child_socket = socket.socketpair()
        env = os.environ.copy()
        env[_CHANNEL_FD_ENV] = str(child_socket.fileno())
        worker_command = command or [
            sys.executable,
            "-m",
            "dimos.agents.code_policy.worker",
        ]
        try:
            self._process = subprocess.Popen(
                worker_command,
                close_fds=True,
                env=env,
                pass_fds=(child_socket.fileno(),),
                start_new_session=True,
                stdin=subprocess.DEVNULL,
            )
        except BaseException:
            parent_socket.close()
            child_socket.close()
            raise
        child_socket.close()
        self._socket: socket.socket | None = parent_socket
        self._recv_buffer = bytearray()
        try:
            self._request(
                {
                    "op": "init",
                    "recording_path": recording_path,
                    "transport": transport,
                    "output_limit": output_limit,
                },
                timeout_s=startup_timeout_s,
            )
        except BaseException:
            self.close()
            raise

    @property
    def pid(self) -> int:
        return self._process.pid

    def execute(self, code: str, timeout_s: float) -> CellResult:
        try:
            response = self._request({"op": "execute", "code": code}, timeout_s=timeout_s)
        except TimeoutError as exc:
            raise PolicyWorkerTimeoutError(
                f"Policy execution timed out after {timeout_s:.1f}s"
            ) from exc
        result = response.get("result")
        if not isinstance(result, dict):
            raise PolicyWorkerError("Policy worker returned no execution result")
        values = {field.name: result[field.name] for field in fields(CellResult)}
        return CellResult(**values)

    def is_alive(self) -> bool:
        return self._process.poll() is None

    def close(self) -> None:
        process = getattr(self, "_process", None)
        sock = getattr(self, "_socket", None)
        if process is None:
            return
        if process.poll() is None and sock is not None:
            try:
                self._request({"op": "close"}, timeout_s=0.5)
            except BaseException:
                pass
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=1.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=1.0)
        if sock is not None:
            sock.close()
            self._socket = None

    def _request(self, request: dict[str, Any], *, timeout_s: float) -> dict[str, Any]:
        if self._process.poll() is not None:
            raise PolicyWorkerError(f"Policy worker exited with code {self._process.returncode}")
        sock = self._socket
        if sock is None:
            raise PolicyWorkerError("Policy worker protocol channel is closed")
        payload = json.dumps(request, separators=(",", ":")).encode() + b"\n"
        try:
            sock.sendall(payload)
            response = self._receive_line(timeout_s)
        except TimeoutError:
            raise
        except (BrokenPipeError, ConnectionError, OSError) as exc:
            raise PolicyWorkerError(f"Policy worker protocol failed: {exc}") from exc
        decoded = json.loads(response)
        if not isinstance(decoded, dict):
            raise PolicyWorkerError("Policy worker returned a non-object response")
        if not decoded.get("ok"):
            error_type = decoded.get("error_type", "PolicyWorkerError")
            message = decoded.get("error", "unknown worker error")
            raise PolicyWorkerError(f"{error_type}: {message}")
        return cast("dict[str, Any]", decoded)

    def _receive_line(self, timeout_s: float) -> bytes:
        sock = self._socket
        if sock is None:
            raise PolicyWorkerError("Policy worker protocol channel is closed")
        deadline = time.monotonic() + timeout_s
        while True:
            newline = self._recv_buffer.find(b"\n")
            if newline >= 0:
                line = bytes(self._recv_buffer[:newline])
                del self._recv_buffer[: newline + 1]
                return line
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError
            sock.settimeout(remaining)
            chunk = sock.recv(65_536)
            if not chunk:
                raise PolicyWorkerError("Policy worker closed its protocol channel")
            self._recv_buffer.extend(chunk)


class PolicyKernel(Module):
    """Execute trusted agent-authored Python against a running DimOS system."""

    config: PolicyKernelConfig

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._execution_lock = threading.Lock()
        self._kernel: KernelProcess | None = None
        self._execution_count = 0
        self._active_execution: int | None = None
        self._active_started_at: float | None = None
        self._kernel_generation = 0

    def __getstate__(self) -> dict[str, Any]:
        state = cast(
            "dict[str, Any]",
            super().__getstate__(),  # type: ignore[no-untyped-call]
        )
        state.pop("_execution_lock", None)
        state.pop("_kernel", None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._execution_lock = threading.Lock()
        self._kernel = None

    @skill
    def python_exec(self, code: str, timeout_s: float = MAX_EXECUTION_TIMEOUT_S) -> str:
        """Execute one synchronous Python program in the persistent policy session.

        The session preloads `app` for deployed DimOS RPCs and `memory` for
        current and historical memory2 observations. Imports, functions, and
        variables persist across successful calls. Use this for observation
        processing, control flow, retries, and coordinated multi-RPC behavior.

        Args:
            code: Complete Python source for one task attempt.
            timeout_s: Execution deadline in seconds, greater than 0 and at most 110.
        """
        if not 0 < timeout_s <= MAX_EXECUTION_TIMEOUT_S:
            return (
                f"Invalid timeout_s={timeout_s!r}; expected a value in "
                f"(0, {MAX_EXECUTION_TIMEOUT_S:g}]"
            )
        if not self._execution_lock.acquire(blocking=False):
            elapsed = (
                time.monotonic() - self._active_started_at
                if self._active_started_at is not None
                else 0.0
            )
            return (
                f"PolicyKernel busy: execution {self._active_execution} "
                f"has been running for {elapsed:.1f}s"
            )

        self._execution_count += 1
        execution_id = self._execution_count
        self._active_execution = execution_id
        self._active_started_at = time.monotonic()
        logger.info(
            "Policy execution started",
            execution=execution_id,
            generation=self._kernel_generation,
            source=code,
            timeout_s=timeout_s,
        )
        try:
            kernel = self._ensure_kernel()
            result = kernel.execute(code, timeout_s)
            transcript = format_cell_result(result)
            logger.info(
                "Policy execution finished",
                execution=execution_id,
                generation=self._kernel_generation,
                success=result.success,
                duration_s=result.duration_s,
                stdout=result.stdout,
                stderr=result.stderr,
                error_type=result.error_type,
            )
            return transcript
        except PolicyWorkerTimeoutError:
            self._discard_kernel(reason="timeout")
            logger.warning(
                "Policy execution timed out",
                execution=execution_id,
                timeout_s=timeout_s,
            )
            return (
                f"Execution {execution_id} timed out after {timeout_s:.1f}s. "
                "The Python namespace was reset. Remote RPC work may still be running; "
                "it was not cancelled."
            )
        except BaseException as exc:
            self._discard_kernel(reason=type(exc).__name__)
            logger.error(
                "Policy execution failed",
                execution=execution_id,
                error_type=type(exc).__name__,
                error=str(exc),
            )
            return (
                f"Execution {execution_id} failed: {type(exc).__name__}: {exc}. "
                "The Python namespace was reset."
            )
        finally:
            self._active_execution = None
            self._active_started_at = None
            self._execution_lock.release()

    @rpc
    def stop(self) -> None:
        self._discard_kernel(reason="module stop")
        super().stop()

    def _ensure_kernel(self) -> KernelProcess:
        if self._kernel is not None and self._kernel.is_alive():
            return self._kernel
        self._discard_kernel(reason="worker unavailable")
        app = Dimos.connect()
        try:
            recorder_name, recording_path = resolve_recorder(app)
        finally:
            app.stop()
        self._kernel = KernelProcess(
            recording_path=recording_path,
            transport=self.config.g.transport,
            output_limit=self.config.output_limit,
            startup_timeout_s=self.config.startup_timeout_s,
        )
        self._kernel_generation += 1
        logger.info(
            "Policy kernel started",
            generation=self._kernel_generation,
            pid=self._kernel.pid,
            recorder=recorder_name,
            recording_path=recording_path,
        )
        return self._kernel

    def _discard_kernel(self, *, reason: str) -> None:
        kernel = self._kernel
        self._kernel = None
        if kernel is None:
            return
        kernel.close()
        logger.info(
            "Policy kernel stopped",
            generation=self._kernel_generation,
            reason=reason,
        )


policy_kernel = PolicyKernel.blueprint
