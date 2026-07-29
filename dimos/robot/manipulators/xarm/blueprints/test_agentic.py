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

from dimos.agents.code_policy import CodePolicyModule
from dimos.agents.mcp.mcp_client import McpClient
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import ModuleBase
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.memory2.module import Recorder
from dimos.perception.manipulation_policy_recorder import ManipulationPolicyRecorder
from dimos.robot.manipulators.xarm.blueprints.agentic import (
    xarm_perception_agent,
    xarm_perception_sim_agent,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule


def _module_types(blueprint: Blueprint) -> list[type[ModuleBase]]:
    return [atom.module for atom in blueprint.blueprints]


def _skill_names(module_type: type[ModuleBase]) -> set[str]:
    return {
        name for name in dir(module_type) if getattr(getattr(module_type, name), "__skill__", False)
    }


def test_sim_agent_composes_one_recorder_and_code_policy_only_in_simulation() -> None:
    sim_modules = _module_types(xarm_perception_sim_agent)
    real_modules = _module_types(xarm_perception_agent)

    assert sim_modules.count(CodePolicyModule) == 1
    assert sim_modules.count(ManipulationPolicyRecorder) == 1
    assert sum(issubclass(module, Recorder) for module in sim_modules) == 1
    assert CodePolicyModule not in real_modules
    assert all(not issubclass(module, Recorder) for module in real_modules)


def test_sim_agent_exposes_code_policy_and_retains_atomic_skills() -> None:
    sim_modules = _module_types(xarm_perception_sim_agent)

    assert "python_exec" in _skill_names(CodePolicyModule)
    assert PickAndPlaceModule in sim_modules
    assert {"pick", "open_gripper"} <= _skill_names(PickAndPlaceModule)


def test_sim_agent_prompt_soft_routes_complex_work_to_python_exec() -> None:
    client_atom = next(
        atom for atom in xarm_perception_sim_agent.blueprints if atom.module is McpClient
    )

    assert "Prefer **python_exec**" in client_atom.kwargs["system_prompt"]
    assert (
        "Direct skills remain appropriate for a single atomic action"
        in client_atom.kwargs["system_prompt"]
    )


def test_sim_agent_selects_registered_scene_pointcloud() -> None:
    assert (
        xarm_perception_sim_agent.remapping_map[(MujocoSimModule.name, "pointcloud")]
        == "raw_pointcloud"
    )
