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

"""Standalone one-tool MCP host for :class:`CodePolicySession`."""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import Mapping
import os
import socket
import subprocess
import sys
import threading
from typing import Any

from fastapi import FastAPI
from fastapi.responses import JSONResponse
import requests
from starlette.requests import Request
import uvicorn

from dimos.agents.code_policy_core import (
    MAX_EXECUTION_TIMEOUT_S,
    CodePolicySession,
    CodePolicySessionConfig,
    FrozenMemoryEnvironment,
    LiveDimosEnvironment,
)
from dimos.agents.mcp.mcp_adapter import McpAdapter

PYTHON_EXEC_DESCRIPTION = """Execute one synchronous Python program in the persistent policy session.

The trusted, unsandboxed session preloads `memory` for observations and, in a live
environment, `app` for deployed DimOS RPCs. Imports, functions, variables, and
mutations persist until the host resets the session.
"""

PYTHON_EXEC_TOOL = {
    "name": "python_exec",
    "description": PYTHON_EXEC_DESCRIPTION,
    "inputSchema": {
        "type": "object",
        "properties": {
            "code": {"type": "string"},
            "timeout_s": {"type": "number", "default": MAX_EXECUTION_TIMEOUT_S},
        },
        "required": ["code"],
        "additionalProperties": False,
    },
}


class StandaloneCodePolicyServer:
    """Own a CodePolicy session and serve it directly over MCP."""

    def __init__(
        self,
        config: CodePolicySessionConfig,
        *,
        host: str = "127.0.0.1",
        port: int = 0,
    ) -> None:
        self.config = config
        self.host = host
        self.port = port
        self.session = CodePolicySession(config)
        self.app = FastAPI()
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self.shutdown_requested = threading.Event()
        self._install_routes()

    @property
    def mcp_url(self) -> str:
        if self.port <= 0:
            raise RuntimeError("standalone CodePolicy server has not started")
        return f"http://{self.host}:{self.port}/mcp"

    @property
    def control_url(self) -> str:
        if self.port <= 0:
            raise RuntimeError("standalone CodePolicy server has not started")
        return f"http://{self.host}:{self.port}/control"

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("standalone CodePolicy server already started")
        self.session.start()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(2048)
        self.port = int(sock.getsockname()[1])
        self._socket = sock
        server = uvicorn.Server(uvicorn.Config(self.app, log_level="warning", access_log=False))
        self._server = server

        def serve() -> None:
            asyncio.run(server.serve(sockets=[sock]))

        self._thread = threading.Thread(
            target=serve,
            name=f"code-policy-mcp-{self.port}",
            daemon=True,
        )
        self._thread.start()
        if not McpAdapter(self.mcp_url, timeout=2).wait_for_ready(timeout=10, interval=0.05):
            self.stop()
            raise TimeoutError("standalone CodePolicy MCP server did not become ready")

    def stop(self) -> None:
        server = self._server
        thread = self._thread
        self._server = None
        self._thread = None
        if server is not None:
            server.should_exit = True
        if thread is not None:
            thread.join(timeout=5)
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self.session.stop()

    def run_forever(self) -> None:
        self.start()
        try:
            while not self.shutdown_requested.wait(0.2):
                thread = self._thread
                if thread is None or not thread.is_alive():
                    raise RuntimeError("standalone CodePolicy MCP server stopped unexpectedly")
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def _install_routes(self) -> None:
        @self.app.post("/mcp")
        async def mcp_endpoint(request: Request) -> JSONResponse:
            try:
                body = await request.json()
            except Exception:
                return JSONResponse(_error(None, -32700, "Parse error"), status_code=400)
            return JSONResponse(await self._handle_mcp(body))

        @self.app.post("/control/{operation}")
        async def control_endpoint(operation: str, request: Request) -> JSONResponse:
            body: dict[str, Any] = {}
            if request.headers.get("content-length") not in {None, "0"}:
                body = await request.json()
            if operation == "receipt":
                value: Any = self.session.get_session_receipt().model_dump(mode="json")
            elif operation == "reset":
                value = self.session.reset_session().model_dump(mode="json")
            elif operation == "interrupt":
                value = {"interrupted": self.session.interrupt_active()}
            elif operation == "records":
                records = self.session.get_execution_records(body.get("session_id"))
                value = [record.model_dump(mode="json") for record in records]
            elif operation == "shutdown":
                self.shutdown_requested.set()
                value = {"accepted": True}
            else:
                return JSONResponse({"error": f"unknown control operation: {operation}"}, 404)
            return JSONResponse(value)

    async def _handle_mcp(self, body: Any) -> dict[str, Any]:
        if not isinstance(body, dict):
            return _error(None, -32600, "Invalid request")
        request_id = body.get("id")
        method = body.get("method")
        if method == "initialize":
            return _result(
                request_id,
                {
                    "protocolVersion": "2025-11-25",
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": "dimos-code-policy", "version": "1.0.0"},
                },
            )
        if method == "tools/list":
            return _result(request_id, {"tools": [PYTHON_EXEC_TOOL]})
        if method != "tools/call":
            return _error(request_id, -32601, f"Unknown: {method}")
        params = body.get("params") or {}
        if params.get("name") != "python_exec":
            return _result(request_id, _text(f"Tool not found: {params.get('name', '')}"))
        arguments = params.get("arguments") or {}
        if not isinstance(arguments, dict) or set(arguments) - {"code", "timeout_s"}:
            return _result(request_id, _text("Invalid python_exec arguments"))
        code = arguments.get("code")
        timeout_s = arguments.get("timeout_s", MAX_EXECUTION_TIMEOUT_S)
        if not isinstance(code, str) or not code:
            return _result(request_id, _text("python_exec code must be a non-empty string"))
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
            return _result(request_id, _text("python_exec timeout_s must be numeric"))
        transcript = await asyncio.to_thread(self.session.python_exec, code, float(timeout_s))
        return _result(request_id, _text(transcript))


class StandaloneCodePolicyProcess:
    """Runner-owned standalone process plus private control client."""

    def __init__(
        self,
        config: CodePolicySessionConfig,
        *,
        environment: Mapping[str, str] | None = None,
    ) -> None:
        self.config = config
        self.environment = dict(environment or {})
        self.port = _available_port()
        self.mcp_url = f"http://127.0.0.1:{self.port}/mcp"
        self.control_url = f"http://127.0.0.1:{self.port}/control"
        self.process: subprocess.Popen[str] | None = None

    def start(self, timeout_s: float = 10.0) -> None:
        if self.process is not None:
            raise RuntimeError("standalone CodePolicy process already started")
        self.process = subprocess.Popen(
            (
                sys.executable,
                "-m",
                "dimos.agents.code_policy_server",
                "--config-json",
                self.config.model_dump_json(),
                "--port",
                str(self.port),
            ),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env={**os.environ, **self.environment},
        )
        if not McpAdapter(self.mcp_url, timeout=2).wait_for_ready(timeout=timeout_s, interval=0.05):
            self.close()
            raise TimeoutError("standalone CodePolicy process did not become ready")

    def receipt(self) -> dict[str, Any]:
        value = self._control("receipt")
        if not isinstance(value, dict):
            raise TypeError("CodePolicy receipt response is not an object")
        return value

    def reset(self) -> dict[str, Any]:
        value = self._control("reset")
        if not isinstance(value, dict):
            raise TypeError("CodePolicy reset response is not an object")
        return value

    def records(self, session_id: str | None = None) -> list[dict[str, Any]]:
        value = self._control("records", {"session_id": session_id})
        if not isinstance(value, list):
            raise TypeError("CodePolicy records response is not a list")
        return value

    def interrupt(self) -> bool:
        return bool(self._control("interrupt").get("interrupted"))

    def close(self) -> None:
        process = self.process
        self.process = None
        if process is None:
            return
        if process.poll() is None:
            try:
                self._control("shutdown")
                process.wait(timeout=5)
            except Exception:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)

    def _control(self, operation: str, body: dict[str, Any] | None = None) -> Any:
        response = requests.post(f"{self.control_url}/{operation}", json=body or {}, timeout=5)
        response.raise_for_status()
        return response.json()

    def __enter__(self) -> StandaloneCodePolicyProcess:
        self.start()
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()


def _result(request_id: Any, result: Any) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "result": result}


def _error(request_id: Any, code: int, message: str) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "error": {"code": code, "message": message}}


def _text(value: str) -> dict[str, Any]:
    return {"content": [{"type": "text", "text": value}]}


def _available_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m dimos.agents.code_policy_server")
    parser.add_argument("--config-json")
    parser.add_argument("--port", type=int, default=0)
    environment = parser.add_mutually_exclusive_group()
    environment.add_argument("--live-memory")
    environment.add_argument("--frozen-source")
    parser.add_argument("--derived-memory")
    parser.add_argument("--cutoff-timestamp", type=float)
    args = parser.parse_args(argv)
    if args.config_json:
        config = CodePolicySessionConfig.model_validate_json(args.config_json)
    elif args.live_memory:
        config = CodePolicySessionConfig(
            environment=LiveDimosEnvironment(recording_path=args.live_memory)
        )
    elif args.frozen_source and args.derived_memory and args.cutoff_timestamp is not None:
        config = CodePolicySessionConfig(
            environment=FrozenMemoryEnvironment(
                recording_path=args.frozen_source,
                derived_recording_path=args.derived_memory,
                memory_cutoff_timestamp=args.cutoff_timestamp,
            )
        )
    else:
        parser.error("provide --config-json, --live-memory, or all frozen source arguments")
    StandaloneCodePolicyServer(config, port=args.port).run_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
