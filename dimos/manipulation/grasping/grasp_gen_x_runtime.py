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

"""Isolated GraspGenX subprocess runtime."""

from __future__ import annotations

import base64
from io import BytesIO
import json
from pathlib import Path
import select
import subprocess
import threading

import numpy as np

from dimos.manipulation.grasping.grasp_gen_x import (
    GRASPGENX_MODEL_REPO,
    GRASPGENX_MODEL_REVISION,
    GRASPGENX_MODEL_VERSION,
    GraspGenXConfig,
)


class GraspGenXRuntime:
    """Run GraspGenX in its pinned environment without modifying DimOS dependencies."""

    def __init__(self, config: GraspGenXConfig) -> None:
        worker_python = Path(config.worker_python)
        if not worker_python.is_file():
            raise FileNotFoundError(
                f"GraspGenX worker Python not found: {worker_python}. "
                "Run bin/setup-graspgenx-env first."
            )
        worker_script = Path(__file__).with_name("grasp_gen_x_worker.py")
        self._timeout = config.worker_timeout
        self._lock = threading.Lock()
        self._process = subprocess.Popen(
            [str(worker_python), str(worker_script)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        try:
            self._request(
                {
                    "op": "initialize",
                    "gripper": config.gripper.model_dump(),
                    "model_repo": GRASPGENX_MODEL_REPO,
                    "model_revision": GRASPGENX_MODEL_REVISION,
                    "model_version": GRASPGENX_MODEL_VERSION,
                }
            )
        except Exception:
            self.stop()
            raise

    def infer(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Run inference on finite XYZ points in metres."""
        result = self._request({"op": "infer", "points": _encode_array(points)})
        return _decode_array(result["poses"]), _decode_array(result["scores"])

    def stop(self) -> None:
        """Stop the isolated model worker."""
        process = self._process
        if process.poll() is not None:
            return
        try:
            self._request({"op": "shutdown"})
        except Exception:
            process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    def _request(self, request: dict[str, object]) -> dict[str, object]:
        with self._lock:
            process = self._process
            if process.poll() is not None or process.stdin is None or process.stdout is None:
                raise RuntimeError("GraspGenX worker is not running")
            process.stdin.write(json.dumps(request, separators=(",", ":")) + "\n")
            process.stdin.flush()
            ready, _, _ = select.select([process.stdout], [], [], self._timeout)
            if not ready:
                raise TimeoutError(f"GraspGenX worker did not respond within {self._timeout:.0f}s")
            line = process.stdout.readline()
            if not line:
                stderr = process.stderr.read() if process.stderr is not None else ""
                raise RuntimeError(f"GraspGenX worker exited unexpectedly: {stderr.strip()}")
            response = json.loads(line)
            if not response.get("ok"):
                raise RuntimeError(str(response.get("error", "GraspGenX worker failed")))
            result = response.get("result", {})
            if not isinstance(result, dict):
                raise RuntimeError("GraspGenX worker returned an invalid response")
            return result


def _encode_array(value: np.ndarray) -> str:
    buffer = BytesIO()
    np.save(buffer, value, allow_pickle=False)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def _decode_array(value: object) -> np.ndarray:
    if not isinstance(value, str):
        raise RuntimeError("GraspGenX worker returned a non-array payload")
    return np.load(BytesIO(base64.b64decode(value)), allow_pickle=False)
