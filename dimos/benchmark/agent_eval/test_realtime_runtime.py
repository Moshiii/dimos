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

from dimos.benchmark.agent_eval.case import SimulatorSceneSource
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
