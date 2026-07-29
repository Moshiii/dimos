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

from __future__ import annotations

import asyncio
from collections.abc import Iterator
import sys
from typing import cast

import pytest

from dimos.agents.code_policy.policy_kernel import (
    KernelProcess,
    PolicyKernel,
    PolicyWorkerError,
    PolicyWorkerTimeoutError,
    resolve_recorder,
)
from dimos.agents.code_policy.session import CellResult
from dimos.agents.mcp.mcp_server import handle_request
from dimos.core.introspection.module.info import ModuleInfo
from dimos.core.module import Module
from dimos.memory2.module import Recorder
from dimos.porcelain.dimos import Dimos

_FAKE_WORKER = r"""
import json
import os
import socket
import threading

sock = socket.socket(fileno=int(os.environ["DIMOS_POLICY_CHANNEL_FD"]))
with sock, sock.makefile("rw", encoding="utf-8", newline="\n") as channel:
    for line in channel:
        request = json.loads(line)
        op = request["op"]
        if op == "init":
            response = {"ok": True}
        elif op == "execute" and request["code"] == "timeout":
            threading.Event().wait()
            continue
        elif op == "execute" and request["code"] == "crash":
            os._exit(7)
        elif op == "execute" and request["code"] == "protocol":
            response = {"ok": False, "error_type": "RuntimeError", "error": "bad frame"}
        elif op == "execute":
            response = {
                "ok": True,
                "result": {
                    "execution_count": 1,
                    "success": True,
                    "duration_s": 0.01,
                    "stdout": "Out[1]: 42\n",
                    "stderr": "",
                    "error_type": None,
                },
            }
        elif op == "close":
            channel.write(json.dumps({"ok": True}) + "\n")
            channel.flush()
            break
        else:
            response = {"ok": False, "error_type": "ValueError", "error": "bad op"}
        channel.write(json.dumps(response) + "\n")
        channel.flush()
"""


class OtherModule(Module):
    pass


class AlternateRecorder(Recorder):
    pass


class _RecorderHandle:
    def __init__(self, path: str) -> None:
        self._path = path

    def recording_path(self) -> str:
        return self._path


class _FakeApp:
    def __init__(self, modules: list[ModuleInfo], paths: dict[str, str]) -> None:
        self._modules = modules
        self._paths = paths

    def list_modules(self) -> list[ModuleInfo]:
        return self._modules

    def get_module(self, name: str) -> _RecorderHandle:
        return _RecorderHandle(self._paths[name])


class _FakeKernel:
    def __init__(self, result: CellResult | BaseException) -> None:
        self.result = result
        self.closed = False

    def execute(self, code: str, timeout_s: float) -> CellResult:
        if isinstance(self.result, BaseException):
            raise self.result
        return self.result

    def close(self) -> None:
        self.closed = True

    def is_alive(self) -> bool:
        return True


def _module_info(module_cls: type[Module], instance_name: str) -> ModuleInfo:
    return ModuleInfo(
        name=instance_name,
        instance_name=instance_name,
        class_name=module_cls.__name__,
        qualified_path=f"{__name__}.{module_cls.__name__}",
    )


@pytest.fixture
def policy_kernel() -> Iterator[PolicyKernel]:
    module = PolicyKernel()
    try:
        yield module
    finally:
        module.stop()


@pytest.fixture
def kernel_processes() -> Iterator[list[KernelProcess]]:
    processes: list[KernelProcess] = []
    try:
        yield processes
    finally:
        for process in processes:
            process.close()


def _start_fake_process(kernel_processes: list[KernelProcess]) -> KernelProcess:
    process = KernelProcess(
        recording_path="/unused/test.db",
        transport="lcm",
        output_limit=1000,
        startup_timeout_s=2.0,
        command=[sys.executable, "-c", _FAKE_WORKER],
    )
    kernel_processes.append(process)
    return process


def test_resolve_recorder_returns_unique_recorder_path() -> None:
    modules = [
        _module_info(OtherModule, "other"),
        _module_info(AlternateRecorder, "recorder"),
    ]
    app = _FakeApp(modules, {"recorder": "/tmp/policy.db"})

    assert resolve_recorder(cast("Dimos", app)) == ("recorder", "/tmp/policy.db")


def test_resolve_recorder_rejects_missing_recorder() -> None:
    app = _FakeApp([_module_info(OtherModule, "other")], {})

    with pytest.raises(RuntimeError, match="found none"):
        resolve_recorder(cast("Dimos", app))


def test_resolve_recorder_lists_ambiguous_recorders() -> None:
    modules = [
        _module_info(AlternateRecorder, "recorder-b"),
        _module_info(AlternateRecorder, "recorder-a"),
    ]
    app = _FakeApp(modules, {})

    with pytest.raises(RuntimeError, match="recorder-a, recorder-b"):
        resolve_recorder(cast("Dimos", app))


def test_kernel_process_returns_framed_execution_result(
    kernel_processes: list[KernelProcess],
) -> None:
    process = _start_fake_process(kernel_processes)

    result = process.execute("answer", timeout_s=1.0)

    assert result.success
    assert result.stdout == "Out[1]: 42\n"


def test_kernel_process_times_out_without_fixed_sleep(
    kernel_processes: list[KernelProcess],
) -> None:
    process = _start_fake_process(kernel_processes)

    with pytest.raises(PolicyWorkerTimeoutError, match="timed out"):
        process.execute("timeout", timeout_s=0.01)


def test_kernel_process_reports_worker_crash(kernel_processes: list[KernelProcess]) -> None:
    process = _start_fake_process(kernel_processes)

    with pytest.raises(PolicyWorkerError, match="closed|exited"):
        process.execute("crash", timeout_s=1.0)


def test_kernel_process_reports_startup_failure() -> None:
    with pytest.raises(PolicyWorkerError, match="closed|exited|protocol failed"):
        KernelProcess(
            recording_path="/unused/test.db",
            transport="lcm",
            output_limit=1000,
            startup_timeout_s=1.0,
            command=[sys.executable, "-c", "raise SystemExit(9)"],
        )


def test_kernel_process_reports_protocol_failure(
    kernel_processes: list[KernelProcess],
) -> None:
    process = _start_fake_process(kernel_processes)

    with pytest.raises(PolicyWorkerError, match="RuntimeError: bad frame"):
        process.execute("protocol", timeout_s=1.0)


def test_kernel_process_close_reaps_worker(kernel_processes: list[KernelProcess]) -> None:
    process = _start_fake_process(kernel_processes)

    process.close()

    assert not process.is_alive()


def test_policy_kernel_exposes_only_python_exec_skill(policy_kernel: PolicyKernel) -> None:
    assert [skill.func_name for skill in policy_kernel.get_skills()] == ["python_exec"]


def test_mcp_lists_python_exec_from_policy_kernel(policy_kernel: PolicyKernel) -> None:
    skills = policy_kernel.get_skills()

    response = asyncio.run(
        handle_request(
            {"method": "tools/list", "id": 1},
            skills,
            {"python_exec": policy_kernel.python_exec},
        )
    )

    assert response is not None
    assert [tool["name"] for tool in response["result"]["tools"]] == ["python_exec"]


def test_policy_kernel_returns_repl_transcript(mocker, policy_kernel: PolicyKernel) -> None:
    worker = _FakeKernel(
        CellResult(
            execution_count=3,
            success=True,
            duration_s=0.2,
            stdout="Out[3]: 42\n",
            stderr="",
            error_type=None,
        )
    )
    mocker.patch.object(policy_kernel, "_ensure_kernel", return_value=worker)

    result = policy_kernel.python_exec("6 * 7")

    assert "In [3] completed" in result
    assert "Out[3]: 42" in result


def test_policy_kernel_rejects_invalid_timeout_without_starting_worker(
    mocker, policy_kernel: PolicyKernel
) -> None:
    ensure_kernel = mocker.patch.object(policy_kernel, "_ensure_kernel")

    result = policy_kernel.python_exec("pass", timeout_s=111.0)

    assert "Invalid timeout" in result
    ensure_kernel.assert_not_called()


def test_policy_kernel_rejects_concurrent_call(policy_kernel: PolicyKernel) -> None:
    policy_kernel._execution_lock.acquire()
    policy_kernel._active_execution = 9
    try:
        result = policy_kernel.python_exec("pass")
    finally:
        policy_kernel._execution_lock.release()

    assert "PolicyKernel busy: execution 9" in result


def test_policy_timeout_discards_namespace_and_warns_about_remote_work(
    mocker, policy_kernel: PolicyKernel
) -> None:
    worker = _FakeKernel(PolicyWorkerTimeoutError("timeout"))
    mocker.patch.object(policy_kernel, "_ensure_kernel", return_value=worker)
    policy_kernel._kernel = cast("KernelProcess", worker)

    result = policy_kernel.python_exec("while True: pass", timeout_s=0.01)

    assert worker.closed
    assert "namespace was reset" in result
    assert "Remote RPC work may still be running" in result


def test_policy_worker_failure_discards_namespace(mocker, policy_kernel: PolicyKernel) -> None:
    worker = _FakeKernel(PolicyWorkerError("protocol failed"))
    mocker.patch.object(policy_kernel, "_ensure_kernel", return_value=worker)
    policy_kernel._kernel = cast("KernelProcess", worker)

    result = policy_kernel.python_exec("pass")

    assert worker.closed
    assert "PolicyWorkerError: protocol failed" in result
    assert "namespace was reset" in result


def test_policy_call_recovers_with_fresh_worker_after_failure(
    mocker, policy_kernel: PolicyKernel
) -> None:
    failed_worker = _FakeKernel(PolicyWorkerError("crashed"))
    recovered_worker = _FakeKernel(
        CellResult(
            execution_count=1,
            success=True,
            duration_s=0.01,
            stdout="Out[1]: 'fresh'\n",
            stderr="",
            error_type=None,
        )
    )
    policy_kernel._kernel = cast("KernelProcess", failed_worker)
    mocker.patch.object(
        policy_kernel,
        "_ensure_kernel",
        side_effect=[failed_worker, recovered_worker],
    )

    failed = policy_kernel.python_exec("old_state")
    recovered = policy_kernel.python_exec("'fresh'")

    assert "namespace was reset" in failed
    assert failed_worker.closed
    assert "Out[1]: 'fresh'" in recovered
