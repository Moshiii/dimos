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

"""Standalone GraspGenX worker for the isolated inference environment."""

from __future__ import annotations

import base64
from contextlib import redirect_stdout
from io import BytesIO
import json
import os
import sys
from typing import Any

import numpy as np


def _encode_array(value: np.ndarray) -> str:
    buffer = BytesIO()
    np.save(buffer, value, allow_pickle=False)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def _decode_array(value: str) -> np.ndarray:
    return np.load(BytesIO(base64.b64decode(value)), allow_pickle=False)


def _initialize(request: dict[str, Any]) -> Any:
    from huggingface_hub import snapshot_download

    snapshot_root = snapshot_download(
        repo_id=request["model_repo"],
        revision=request["model_revision"],
        allow_patterns=[f"{request['model_version']}/gen/*", f"{request['model_version']}/dis/*"],
    )
    os.environ["GRASPGENX_CHECKPOINT_DIR"] = snapshot_root
    os.environ["GRASPGENX_GRIPPER_CFG_DIR"] = snapshot_root
    with redirect_stdout(sys.stderr):
        from graspgenx.grasp_server import SWEEP_VOLUME_ONLY_BACKBONES, GraspGenXSampler
        from graspgenx.utils.checkpoint_io import load_model_cfg
        from graspgenx.x_grippers import make_sweep_volume_gripper_info

    model_version = request["model_version"]
    model_config = load_model_cfg(
        os.path.join(snapshot_root, model_version, "gen"),
        os.path.join(snapshot_root, model_version, "dis"),
        gen_pth=None,
        dis_pth=None,
    )
    for component in ("diffusion", "discriminator"):
        if getattr(model_config, component).gripper_backbone not in SWEEP_VOLUME_ONLY_BACKBONES:
            raise RuntimeError("GraspGenX checkpoint requires an asset-backed gripper")
    gripper = request["gripper"]
    gripper_info = make_sweep_volume_gripper_info(
        extents_open=gripper["extents_open"],
        offset_open=gripper["offset_open"],
        extents_mid=gripper["extents_half_open"],
        offset_mid=gripper["offset_half_open"],
        fingertip_depth=gripper["fingertip_depth"],
        gripper_type={"parallel_2f": 0, "revolute_2f": 1, "revolute_3f": 2}[gripper["family"]],
    )
    return GraspGenXSampler, GraspGenXSampler(model_config, gripper_info=gripper_info)


def main() -> None:
    sampler_type: Any = None
    sampler: Any = None
    for line in sys.stdin:
        try:
            request = json.loads(line)
            operation = request["op"]
            if operation == "initialize":
                sampler_type, sampler = _initialize(request)
                result: dict[str, object] = {}
            elif operation == "infer":
                if sampler is None:
                    raise RuntimeError("GraspGenX worker has not been initialized")
                poses, scores = sampler_type.run_inference(
                    _decode_array(request["points"]), sampler
                )
                result = {
                    "poses": _encode_array(poses.detach().cpu().numpy()),
                    "scores": _encode_array(scores.detach().cpu().numpy()),
                }
            elif operation == "shutdown":
                print(json.dumps({"ok": True, "result": {}}), flush=True)
                return
            else:
                raise ValueError(f"unknown operation: {operation}")
            print(json.dumps({"ok": True, "result": result}), flush=True)
        except Exception as exc:
            print(json.dumps({"ok": False, "error": str(exc)}), flush=True)


if __name__ == "__main__":
    main()
