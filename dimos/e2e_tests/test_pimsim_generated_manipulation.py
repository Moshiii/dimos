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

import json
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


def _libero_bundle() -> Path:
    configured = os.environ.get("PIMSIM_LIBERO_BUNDLE")
    if configured is None:
        pytest.skip("set PIMSIM_LIBERO_BUNDLE to run native tabletop E2E tests")
    root = Path(configured).expanduser().resolve()
    if not root.exists():
        pytest.fail(f"PIMSIM_LIBERO_BUNDLE does not exist: {root}")
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
    show_viewer = os.environ.get("PIMSIM_TEST_VIEWER") == "1"
    episode = pimsim_authoring.generate_xarm_tabletop_episode(
        tmp_path / "episode",
        libero_root=_libero_root(),
        scene_seed=6,
        scenario_seed=3,
    )
    _run_lift_episode(
        episode.scene_package,
        episode.scenarios["lift-alphabet-soup"],
        scenario_id="lift-alphabet-soup-seed-3",
        object_name="alphabet-soup",
        show_viewer=show_viewer,
    )


def test_native_bundle_lift_uses_exact_suite_scene(tmp_path: Path) -> None:
    show_viewer = os.environ.get("PIMSIM_TEST_VIEWER") == "1"
    episode = pimsim_authoring.materialize_xarm_tabletop_episode(
        tmp_path / "episode",
        asset_bundle=_libero_bundle(),
        scene_seed=296,
        scenario_seed=3,
        object_count=2,
    )
    scenario = episode.scenarios["lift-object"]
    _run_lift_episode(
        episode.scene_package,
        scenario,
        scenario_id="lift-object-seed-3",
        object_name=_role_semantic_name(episode.scene_package, scenario, "object"),
        show_viewer=show_viewer,
    )


@pytest.mark.parametrize(
    ("family_id", "scene_seed", "scenario_seed", "z_offset"),
    (
        pytest.param("object-in-receptacle", 296, 3, 0.10, id="place-in"),
        pytest.param("object-on-support", 48, 15, 0.08, id="place-on"),
    ),
)
def test_native_bundle_place_uses_public_skills_and_private_goal(
    tmp_path: Path,
    family_id: str,
    scene_seed: int,
    scenario_seed: int,
    z_offset: float,
) -> None:
    show_viewer = os.environ.get("PIMSIM_TEST_VIEWER") == "1"
    episode = pimsim_authoring.materialize_xarm_tabletop_episode(
        tmp_path / "episode",
        asset_bundle=_libero_bundle(),
        scene_seed=scene_seed,
        scenario_seed=scenario_seed,
        object_count=2,
    )
    _run_drop_episode(
        episode.scene_package,
        episode.scenarios[family_id],
        z_offset,
        show_viewer=show_viewer,
    )


def _run_lift_episode(
    scene_package: Path,
    scenario: Path,
    *,
    scenario_id: str,
    object_name: str,
    show_viewer: bool,
) -> None:
    call = DimosCliCall()
    call.global_args = [
        "--simulation-provider",
        "pimsim",
        "--scene-package",
        str(scene_package),
        "--transport",
        "zenoh",
        "--viewer",
        "rerun" if show_viewer else "none",
    ]
    call.demo_args = ["run", "xarm-perception-sim"]
    call.extra_env["PIMSIM_MUJOCO_VIEWER"] = "1" if show_viewer else "0"
    call.start()

    app: Dimos | None = None
    scene_control = load_episode_scene_control("pimsim")
    try:
        app = _connect_when_ready(call)
        scene_control.start()
        reset = scene_control.reset_scenario(str(scenario))
        assert reset["scenario_id"] == scenario_id
        assert all(condition["passed"] for condition in reset["initial_conditions"])

        pick_and_place = app.get_module("PickAndPlaceModule")
        observation = _wait_for_object(pick_and_place, object_name)
        assert observation.success, observation
        result = pick_and_place.pick(object_name)
        evaluation = scene_control.evaluate_goal()
        assert result.success, f"{result}; private evaluation: {evaluation}"
        assert evaluation["scenario_id"] == scenario_id
        assert evaluation["passed"] is True
    finally:
        scene_control.stop()
        if app is not None:
            app.stop()
        call.stop()


def _run_drop_episode(
    scene_package: Path,
    scenario: Path,
    z_offset: float,
    *,
    show_viewer: bool,
) -> None:
    call = DimosCliCall()
    call.global_args = [
        "--simulation-provider",
        "pimsim",
        "--scene-package",
        str(scene_package),
        "--transport",
        "zenoh",
        "--viewer",
        "rerun" if show_viewer else "none",
    ]
    call.demo_args = ["run", "xarm-perception-sim"]
    call.extra_env["PIMSIM_MUJOCO_VIEWER"] = "1" if show_viewer else "0"
    call.start()

    app: Dimos | None = None
    scene_control = load_episode_scene_control("pimsim")
    try:
        app = _connect_when_ready(call)
        scene_control.start()
        pick_and_place = app.get_module("PickAndPlaceModule")
        reset = scene_control.reset_scenario(str(scenario))
        assert all(condition["passed"] for condition in reset["initial_conditions"])
        object_name = _role_semantic_name(scene_package, scenario, "object")
        target_name = _role_semantic_name(scene_package, scenario, "target")
        observation = _wait_for_object(pick_and_place, object_name)
        assert target_name in observation.message, observation
        pick_result = pick_and_place.pick(object_name)
        assert pick_result.success, pick_result
        place_result = pick_and_place.drop_on(target_name, z_offset=z_offset)
        evaluation = _wait_for_goal(scene_control)
        assert place_result.success, f"{place_result}; private evaluation: {evaluation}"
        assert evaluation["scenario_id"] == reset["scenario_id"]
        assert evaluation["passed"] is True
    finally:
        scene_control.stop()
        if app is not None:
            app.stop()
        call.stop()


def _wait_for_goal(scene_control: Any, timeout: float = 5.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    evaluation: dict[str, Any] = {}
    while time.monotonic() < deadline:
        evaluation = scene_control.evaluate_goal()
        if evaluation["passed"]:
            return evaluation
        time.sleep(0.1)
    return evaluation


def _role_semantic_name(scene_package: Path, scenario: Path, role: str) -> str:
    scenario_raw = json.loads(scenario.read_text(encoding="utf-8"))
    package_raw = json.loads(scene_package.read_text(encoding="utf-8"))
    entity_id = scenario_raw["role_bindings"][role]
    for entity in package_raw["entities"]:
        if entity["entity_id"] == entity_id:
            return str(entity["semantic_class"])
    raise AssertionError(f"scenario role {role!r} references unknown entity {entity_id!r}")
