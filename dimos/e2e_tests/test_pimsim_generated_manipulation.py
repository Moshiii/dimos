# Copyright 2025-2026 Dimensional Inc.
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

import os
from pathlib import Path
import time
from typing import Any

import pytest

from dimos import Dimos
from dimos.e2e_tests.dimos_cli_call import DimosCliCall
from dimos.e2e_tests.scene_control import load_episode_scene_control

pimsim_authoring: Any = pytest.importorskip("pimsim.authoring")

pytestmark = [pytest.mark.self_hosted_large, pytest.mark.mujoco]


def _libero_root() -> Path:
    configured = os.environ.get("PIMSIM_LIBERO_ROOT")
    if configured is None:
        pytest.skip("set PIMSIM_LIBERO_ROOT to run generated manipulation E2E tests")
    root = Path(configured).expanduser().resolve()
    if not root.exists():
        pytest.fail(f"PIMSIM_LIBERO_ROOT does not exist: {root}")
    return root


def _connect_when_ready(call: DimosCliCall, timeout: float = 120.0) -> Dimos:
    deadline = time.monotonic() + timeout
    last_error: BaseException | None = None
    while time.monotonic() < deadline:
        process = call.process
        if process is not None and process.poll() is not None:
            pytest.fail(f"DimOS exited during startup with code {process.returncode}")
        app: Dimos | None = None
        try:
            app = Dimos.connect(timeout=2.0)
            app.get_module("PickAndPlaceModule")
            app.get_module("PimSimEpisodeControl")
            return app
        except Exception as error:
            last_error = error
            if app is not None:
                app.stop()
            time.sleep(0.5)
    pytest.fail(f"DimOS modules were not ready after {timeout:.0f}s: {last_error}")


def _wait_for_object(pick_and_place: Any, name: str, timeout: float = 20.0) -> Any:
    deadline = time.monotonic() + timeout
    last_observation: Any = None
    while time.monotonic() < deadline:
        last_observation = pick_and_place.look()
        if name in last_observation.message:
            return last_observation
        time.sleep(0.25)
    pytest.fail(f"object {name!r} was not observed after {timeout:.0f}s: {last_observation}")


def test_generated_lift_uses_public_skill_and_private_goal(tmp_path: Path) -> None:
    episode = pimsim_authoring.generate_xarm_tabletop_episode(
        tmp_path / "episode",
        libero_root=_libero_root(),
        scene_seed=7,
        scenario_seed=3,
    )
    scenario = episode.scenarios["lift-alphabet-soup"]

    call = DimosCliCall()
    call.global_args = [
        "--simulation-provider",
        "pimsim",
        "--scene-package",
        str(episode.scene_package),
        "--transport",
        "zenoh",
        "--viewer",
        "none",
    ]
    call.demo_args = ["run", "xarm-perception-sim"]
    call.extra_env["PIMSIM_MUJOCO_VIEWER"] = "0"
    call.start()

    app: Dimos | None = None
    scene_control = load_episode_scene_control("pimsim")
    try:
        app = _connect_when_ready(call)
        scene_control.start()
        reset = scene_control.reset_scenario(str(scenario))
        assert reset["scenario_id"] == "lift-alphabet-soup-seed-3"
        assert all(condition["passed"] for condition in reset["initial_conditions"])

        pick_and_place = app.get_module("PickAndPlaceModule")
        observation = _wait_for_object(pick_and_place, "alphabet-soup")
        assert observation.success, observation
        result = pick_and_place.pick("alphabet-soup")
        evaluation = scene_control.evaluate_goal()
        assert result.success, f"{result}; private evaluation: {evaluation}"
        assert evaluation["scenario_id"] == "lift-alphabet-soup-seed-3"
        assert evaluation["passed"] is True
    finally:
        scene_control.stop()
        if app is not None:
            app.stop()
        call.stop()
