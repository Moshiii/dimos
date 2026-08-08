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

"""Case-bound public DimOS stack for one VLN-CE R2R episode."""

from pathlib import Path

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.benchmark.agent_eval.observation_recorder import (
    DEFAULT_AGENT_EVAL_RECORDING_PATH,
    AgentEvalObservationRecorder,
)
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.perception.experimental.spatial_perception import SpatialMemory
from dimos.visualization.vis_module import vis_module

from .connection import VlnceConnection


def vlnce_r2r_eval_blueprint(
    *,
    socket_path: str | Path,
    attempt_id: str,
    case_id: str,
    episode_id: str,
    protocol_revision: str = "vlnce-public.v1",
    recording_path: str | Path = DEFAULT_AGENT_EVAL_RECORDING_PATH,
) -> Blueprint:
    """Compose a public-only navigation stack bound to one benchmark attempt."""

    memory_root = Path(recording_path).parent / "spatial-memory"

    return (
        autoconnect(
            vis_module(viewer_backend=global_config.viewer),
            VlnceConnection.blueprint(
                socket_path=str(socket_path),
                attempt_id=attempt_id,
                case_id=case_id,
                episode_id=episode_id,
                protocol_revision=protocol_revision,
            ),
            ReplanningAStarPlanner.blueprint(
                robot_width=0.2,
                robot_rotation_diameter=0.2,
            ),
            SpatialMemory.blueprint(
                db_path=str(memory_root / "chromadb"),
                visual_memory_path=str(memory_root / "visual-memory.pkl"),
                output_dir=str(memory_root),
                new_memory=True,
            ),
            NavigationSkillContainer.blueprint(),
            AgentEvalObservationRecorder.blueprint(
                db_path=recording_path,
                on_existing="overwrite",
            ),
        )
        .remappings(
            [
                (ReplanningAStarPlanner, "nav_cmd_vel", "cmd_vel"),
                (ReplanningAStarPlanner, "odometry", "unused_benchmark_odometry"),
                (AgentEvalObservationRecorder, "global_map", "pointcloud"),
            ]
        )
        .global_config(
            configure_system=False,
            n_workers=7,
            robot_model="vlnce_habitat_cylinder",
            transport="zenoh",
        )
    )
