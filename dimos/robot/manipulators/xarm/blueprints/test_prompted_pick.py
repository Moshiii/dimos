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

"""Composition tests for the real xArm6 prompted-pick blueprint."""

from typing import Any

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom
from dimos.core.coordination.module_coordinator import _resolve_single_ref
from dimos.experimental.world_belief.worldbelief_module import WorldBeliefModule
from dimos.experimental.world_belief.worldbelief_recorder import WorldBeliefRecorder
from dimos.experimental.world_belief.xarm6_blueprint import xarm6_worldbelief
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.pick_and_place_module import (
    PickAndPlaceModuleConfig,
    PromptedPickAndPlaceModule,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.perception.experimental.object_scene_registration import (
    ObjectSceneRegistrationModule,
)
from dimos.perception.memory.prompted_object_localizer import PromptedObjectLocalizerModule
from dimos.robot.manipulators.xarm.blueprints.prompted_pick import xarm6_prompted_pick
from dimos.robot.manipulators.xarm.grasp_config import (
    XARM_GRASP_FRAME_TO_TCP,
    XARM_GRIPPER_SWEEP,
)


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.active_blueprints if atom.module is module_type)


def _module_count(blueprint: Blueprint, module_type: type) -> int:
    return sum(atom.module is module_type for atom in blueprint.active_blueprints)


def test_prompted_pick_composes_only_the_selected_target_provider() -> None:
    assert _module_count(xarm6_prompted_pick, PromptedPickAndPlaceModule) == 1
    assert _module_count(xarm6_prompted_pick, PromptedObjectLocalizerModule) == 1
    assert _module_count(xarm6_prompted_pick, GraspGenXModule) == 1
    assert _module_count(xarm6_prompted_pick, WorldBeliefRecorder) == 1
    assert _module_count(xarm6_prompted_pick, McpServer) == 1
    assert _module_count(xarm6_prompted_pick, McpClient) == 0
    assert _module_count(xarm6_prompted_pick, ObjectSceneRegistrationModule) == 0
    assert _module_count(xarm6_prompted_pick, WorldBeliefModule) == 0
    assert PromptedObjectLocalizerModule.dedicated_worker is True
    assert getattr(PromptedPickAndPlaceModule.pick, "__skill__", False) is True


def test_prompted_pick_rpc_dependencies_resolve_to_one_module_each() -> None:
    pick_atom = BlueprintAtom.create(PromptedPickAndPlaceModule, kwargs={})
    pick_refs = {ref.name: ref for ref in pick_atom.module_refs}
    localizer_ref = pick_refs["_prompted_localizer"]
    generator_ref = pick_refs["_grasp_generator"]

    assert (
        _resolve_single_ref(
            pick_atom,
            localizer_ref,
            localizer_ref.spec,
            xarm6_prompted_pick,
            set(),
        )
        == PromptedObjectLocalizerModule.name
    )
    assert (
        _resolve_single_ref(
            pick_atom,
            generator_ref,
            generator_ref.spec,
            xarm6_prompted_pick,
            set(),
        )
        == GraspGenXModule.name
    )

    localizer_atom = BlueprintAtom.create(PromptedObjectLocalizerModule, kwargs={})
    recorder_ref = next(ref for ref in localizer_atom.module_refs if ref.name == "recorder")
    assert (
        _resolve_single_ref(
            localizer_atom,
            recorder_ref,
            recorder_ref.spec,
            xarm6_prompted_pick,
            set(),
        )
        == WorldBeliefRecorder.name
    )


def test_prompted_pick_uses_xarm6_gripper_and_visualization_geometry() -> None:
    config = PickAndPlaceModuleConfig(
        **_module_kwargs(xarm6_prompted_pick, PromptedPickAndPlaceModule)
    )
    robot = config.robots[0]
    coordinator_atom = next(
        atom for atom in xarm6_prompted_pick.active_blueprints if atom.name == "ControlCoordinator"
    )
    hardware = coordinator_atom.kwargs["hardware"][0]

    assert robot.xacro_args["dof"] == "6"
    assert robot.xacro_args["add_gripper"] == "true"
    assert robot.gripper_hardware_id == "arm"
    assert hardware.gripper_joints == ["arm/gripper"]
    assert isinstance(config.visualization, ViserVisualizationConfig)
    assert config.grasp_visualization is not None
    assert config.grasp_visualization.gripper == XARM_GRIPPER_SWEEP
    assert config.grasp_visualization.grasp_frame_to_tcp == XARM_GRASP_FRAME_TO_TCP


def test_xarm6_worldbelief_composition_remains_perception_only() -> None:
    assert _module_count(xarm6_worldbelief, WorldBeliefModule) == 1
    assert _module_count(xarm6_worldbelief, WorldBeliefRecorder) == 1
    assert _module_count(xarm6_worldbelief, PromptedObjectLocalizerModule) == 0
    assert _module_count(xarm6_worldbelief, PromptedPickAndPlaceModule) == 0
