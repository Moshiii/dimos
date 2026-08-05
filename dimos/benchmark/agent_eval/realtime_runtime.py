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

"""Concrete DimOS/PiMSim lifecycle for canonical real-time evaluations."""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any, cast

from pydantic import JsonValue

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.benchmark.agent_eval.case import SimulatorSceneSource, SourcePreparationRef
from dimos.benchmark.agent_eval.observation_recorder import AgentEvalObservationRecorder
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.global_config import global_config
from dimos.e2e_tests.scene_contract import PlanarBounds
from dimos.e2e_tests.scene_control import SceneControl, load_scene_control
from dimos.perception.experimental.spatial_perception import SpatialMemory
from dimos.porcelain.dimos import Dimos
from dimos.robot.get_all_blueprints import get_by_name
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer
from dimos.simulation.mujoco.direct_cmd_vel_explorer import DirectCmdVelExplorer
from dimos.simulation.providers import load_simulation_provider


class DimosSimulatorRuntimeFactory:
    """Validate plugin entry points and create one evaluator-owned runtime."""

    def preflight(self, source: SimulatorSceneSource) -> None:
        load_simulation_provider(source.simulation_provider)
        control = load_scene_control(source.simulation_provider)
        control.stop()

    def create(self, *, source: SimulatorSceneSource, attempt_path: Path) -> DimosSimulatorRuntime:
        return DimosSimulatorRuntime(source=source, attempt_path=attempt_path)


class DimosSimulatorRuntime:
    def __init__(self, *, source: SimulatorSceneSource, attempt_path: Path) -> None:
        self.source = source
        self.memory_path = attempt_path / "live-memory" / "recording.db"
        self.memory_path.parent.mkdir(parents=True, exist_ok=True)
        self._coordinator: ModuleCoordinator | None = None
        self._control: SceneControl | None = None
        self._explorer: DirectCmdVelExplorer | None = None
        self._app: Dimos | None = None
        self._previous_config = {
            "simulation": global_config.simulation,
            "simulation_provider": global_config.simulation_provider,
            "scene_package": global_config.scene_package,
            "robot_model": global_config.robot_model,
        }

    def start(self) -> dict[str, JsonValue]:
        robot_model = _robot_model(self.source.robot)
        global_config.update(
            simulation="mujoco",
            simulation_provider=self.source.simulation_provider,
            scene_package=self.source.scene,
            robot_model=robot_model,
        )
        # Resolving the source blueprint materializes its case-bound provider binding.
        base = get_by_name(self.source.dimos_blueprint)
        external_pi_navigation_skills = autoconnect(
            NavigationSkillContainer.blueprint(),
            PersonFollowSkillContainer.blueprint(camera_info=GO2Connection.camera_info_static),
            UnitreeSkillContainer.blueprint(),
        )
        evaluation_blueprint = autoconnect(
            base,
            SpatialMemory.blueprint(),
            AgentEvalObservationRecorder.blueprint(
                db_path=self.memory_path,
                on_existing="overwrite",
            ),
            external_pi_navigation_skills,
        )
        self._coordinator = ModuleCoordinator.build(evaluation_blueprint)
        self._coordinator.start_rpc_service()
        self._control = load_scene_control(self.source.simulation_provider)
        self._control.start()
        self._explorer = DirectCmdVelExplorer(linear_speed=0.5)
        self._explorer.start()
        self._app = Dimos.connect(timeout=120.0)
        deadline = time.monotonic() + 120.0
        while not self.memory_path.exists():
            if time.monotonic() >= deadline:
                raise TimeoutError("live Memory2 recording did not become ready")
            time.sleep(0.05)
        modules = self._coordinator.list_modules()
        module_types = {module.class_name for module in modules}
        required_modules = {
            "PimSimGo2",
            "ReplanningAStarPlanner",
            "SpatialMemory",
            "AgentEvalObservationRecorder",
            "NavigationSkillContainer",
        }
        missing_modules = sorted(required_modules - module_types)
        if missing_modules:
            raise RuntimeError(
                "live runtime is missing required capabilities: " + ", ".join(missing_modules)
            )
        return cast(
            "dict[str, JsonValue]",
            {
                "schema_version": "1.0",
                "provider": self.source.simulation_provider,
                "scene": self.source.scene,
                "robot": self.source.robot,
                "blueprint": self.source.dimos_blueprint,
                "module_count": len(modules),
                "required_modules": sorted(required_modules),
                "sensor_ready": True,
                "odometry_ready": self._position_ready(),
                "memory_ready": True,
                "porcelain_ready": True,
                "motion_ready": hasattr(self._app, "ReplanningAStarPlanner"),
            },
        )

    def prepare(self, recipe: SourcePreparationRef | None) -> dict[str, JsonValue]:
        if recipe is None:
            return {"schema_version": "1.0", "recipe": None, "completed": True}
        if self._explorer is None or self._control is None:
            raise RuntimeError("runtime is not started")
        for x, y in recipe.exploration_route:
            self._control.set_agent_position(x, y, recipe.final_start_pose[2])
            self._explorer.wait_until_position(
                x,
                y,
                tolerance=recipe.start_tolerance_metres,
                timeout=recipe.step_timeout_seconds,
            )
            time.sleep(recipe.observation_dwell_seconds)
        self._control.set_agent_position(*recipe.final_start_pose)
        self._explorer.wait_until_position(
            recipe.final_start_pose[0],
            recipe.final_start_pose[1],
            tolerance=recipe.start_tolerance_metres,
            timeout=recipe.odometry_timeout_seconds,
        )
        return {
            "schema_version": "1.0",
            "recipe": recipe.kind,
            "revision": recipe.revision,
            "waypoints_completed": len(recipe.exploration_route),
            "start_pose_verified": True,
            "completed": True,
        }

    def robot_position(self) -> tuple[float, float]:
        if self._explorer is None:
            raise RuntimeError("odometry observer is not started")
        return self._explorer.position()

    def semantic_object_bounds(self, query: str) -> PlanarBounds:
        if self._control is None:
            raise RuntimeError("scene control is not started")
        return self._control.semantic_object_bounds(query)

    def healthy(self) -> bool:
        return self._coordinator is not None and self._coordinator.ping() == "pong"

    def cancel_motion(self) -> None:
        if self._app is not None and hasattr(self._app, "ReplanningAStarPlanner"):
            cast("Any", self._app.ReplanningAStarPlanner).cancel_goal()

    def close(self) -> None:
        errors: list[str] = []
        for name, resource in (
            ("explorer", self._explorer),
            ("scene-control", self._control),
            ("porcelain", self._app),
            ("dimos", self._coordinator),
        ):
            if resource is None:
                continue
            try:
                resource.stop()
            except Exception as exc:
                errors.append(f"{name}: {type(exc).__name__}: {exc}")
        global_config.update(**self._previous_config)
        if errors:
            raise RuntimeError("; ".join(errors))

    def _position_ready(self) -> bool:
        self.robot_position()
        return True


def _robot_model(value: str) -> str:
    normalized = value.lower().replace("-", "_")
    aliases = {"go2": "unitree_go2", "unitree_go2": "unitree_go2"}
    try:
        return aliases[normalized]
    except KeyError as exc:
        raise ValueError(f"unsupported simulator evaluation robot {value!r}") from exc
