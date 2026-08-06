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

from pathlib import Path

import pytest

from dimos.benchmark.agent_eval.case import SimulatorSceneSource, SourcePreparationRef
from dimos.benchmark.agent_eval.realtime_runtime import DimosSimulatorRuntime
from dimos.core.global_config import global_config


@pytest.fixture
def restore_global_config():
    original = global_config.model_dump()
    yield
    global_config.update(**original)


@pytest.mark.parametrize(
    ("viewer", "rerun_web", "rerun_open"),
    [
        ("rerun", True, "web"),
        ("none", False, "none"),
    ],
)
def test_runtime_start_preserves_resolved_visualization_settings(
    tmp_path: Path,
    mocker,
    restore_global_config,
    viewer,
    rerun_web,
    rerun_open,
) -> None:
    global_config.update(viewer=viewer, rerun_web=rerun_web, rerun_open=rerun_open)
    runtime = DimosSimulatorRuntime(
        source=SimulatorSceneSource(
            scene="dimsim-apartment",
            simulation_provider="pimsim",
            robot="unitree-go2",
            dimos_blueprint="unitree-go2",
        ),
        attempt_path=tmp_path,
    )
    resolve_blueprint = mocker.patch(
        "dimos.benchmark.agent_eval.realtime_runtime.get_by_name",
        side_effect=RuntimeError("stop after configuration"),
    )

    with pytest.raises(RuntimeError, match="stop after configuration"):
        runtime.start()

    resolve_blueprint.assert_called_once_with("unitree-go2")
    assert (
        global_config.viewer,
        global_config.rerun_web,
        global_config.rerun_open,
    ) == (viewer, rerun_web, rerun_open)


def test_prepare_physically_follows_original_route_then_teleports_to_task_start(
    tmp_path: Path,
    mocker,
) -> None:
    runtime = DimosSimulatorRuntime(
        source=SimulatorSceneSource(
            scene="dimsim-apartment",
            simulation_provider="pimsim",
            robot="unitree-go2",
            dimos_blueprint="unitree-go2",
        ),
        attempt_path=tmp_path,
    )
    explorer = mocker.Mock()
    scene_control = mocker.Mock()
    mocker.patch.object(runtime, "_explorer", explorer)
    mocker.patch.object(runtime, "_control", scene_control)
    recipe = SourcePreparationRef(
        revision="original-route-v1",
        exploration_route=((1.0, 2.0), (3.0, 4.0)),
        final_start_pose=(3.0, 0.0, 0.52),
        step_timeout_seconds=60.0,
        odometry_timeout_seconds=30.0,
        start_tolerance_metres=0.25,
    )

    receipt = runtime.prepare(recipe)

    explorer.follow_points.assert_called_once_with(
        [(1.0, 2.0), (3.0, 4.0)],
        per_waypoint_timeout=60.0,
    )
    scene_control.set_agent_position.assert_called_once_with(3.0, 0.0, 0.52)
    explorer.wait_until_position.assert_called_once_with(
        3.0,
        0.0,
        tolerance=0.25,
        timeout=30.0,
    )
    assert receipt["waypoints_completed"] == 2
