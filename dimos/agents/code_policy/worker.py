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

"""Private subprocess entry point for persistent policy execution."""

from __future__ import annotations

import json
import os
import socket
from typing import Any

from dimos.agents.code_policy.session import PolicySession
from dimos.core.global_config import global_config
from dimos.memory2.store.sqlite import SqliteStore
from dimos.porcelain.dimos import Dimos

_CHANNEL_FD_ENV = "DIMOS_POLICY_CHANNEL_FD"


def _reply(channel: Any, payload: dict[str, Any]) -> None:
    channel.write(json.dumps(payload, separators=(",", ":")) + "\n")
    channel.flush()


def _serve(channel: Any) -> None:
    app: Dimos | None = None
    memory: SqliteStore | None = None
    session: PolicySession | None = None
    try:
        for line in channel:
            request = json.loads(line)
            operation = request.get("op")
            if operation == "init":
                global_config.update(transport=request["transport"])
                app = Dimos.connect()
                memory = SqliteStore(path=request["recording_path"], must_exist=True)
                memory.start()
                session = PolicySession(
                    app=app,
                    memory=memory,
                    output_limit=request["output_limit"],
                )
                _reply(channel, {"ok": True})
            elif operation == "execute":
                if session is None:
                    raise RuntimeError("Policy worker is not initialized")
                _reply(channel, {"ok": True, "result": session.execute(request["code"]).to_dict()})
            elif operation == "close":
                _reply(channel, {"ok": True})
                return
            else:
                raise ValueError(f"Unknown policy worker operation: {operation!r}")
    except BaseException as exc:
        _reply(
            channel,
            {
                "ok": False,
                "error_type": type(exc).__name__,
                "error": str(exc),
            },
        )
    finally:
        if memory is not None:
            memory.stop()
        if app is not None:
            app.stop()


def main() -> None:
    """Serve requests on the inherited private socket."""
    raw_fd = os.environ.get(_CHANNEL_FD_ENV)
    if raw_fd is None:
        raise RuntimeError(f"{_CHANNEL_FD_ENV} is required")
    sock = socket.socket(fileno=int(raw_fd))
    with sock, sock.makefile("rw", encoding="utf-8", newline="\n") as channel:
        _serve(channel)


if __name__ == "__main__":
    main()
