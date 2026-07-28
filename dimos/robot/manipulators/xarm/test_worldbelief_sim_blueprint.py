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

from typing import Any

import pytest

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.topics import trajectory_task_name
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_HOME,
    make_xarm6_sim_hardware,
    make_xarm6_sim_module_kwargs,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule, MujocoSimModuleConfig


def _module_kwargs(blueprint: Blueprint, module_type: type) -> dict[str, Any]:
    return next(atom.kwargs for atom in blueprint.blueprints if atom.module is module_type)


def _coordinator_tasks(blueprint: Blueprint) -> list[TaskConfig]:
    return _module_kwargs(blueprint, ControlCoordinator)["tasks"]


def test_xarm6_sim_factories_agree_on_dof_camera_frame_and_home() -> None:
    """The sim module and the coordinator adapter must start from one home pose.

    ``MujocoSimModule`` resets the MJCF to ``reset_joint_positions`` while the
    coordinator's mujoco adapter commands ``initial_positions``; if they drift
    apart the arm lurches on the first tick.
    """
    hardware = make_xarm6_sim_hardware("test-xarm6-scene.xml")
    sim_config = MujocoSimModuleConfig(**make_xarm6_sim_module_kwargs("test-xarm6-scene.xml"))

    assert len(XARM6_SIM_HOME) == 6
    assert sim_config.dof == 6
    assert len(hardware.joints) == 6
    assert sim_config.camera_name == "wrist_camera"
    assert sim_config.base_frame_id == "link6"
    assert hardware.adapter_type == "sim_mujoco"
    assert hardware.gripper_joints == ["arm/gripper"]
    assert sim_config.reset_joint_positions == XARM6_SIM_HOME
    assert hardware.adapter_kwargs["initial_positions"] == XARM6_SIM_HOME


@pytest.mark.self_hosted
def test_xarm6_worldbelief_sim_wires_sim_camera_into_the_worldbelief_stack() -> None:
    """Sim blueprint swaps the RealSense for MuJoCo and keeps its own state dir."""
    # Imported lazily: these pull in the heavy perception stack (torch, yoloe)
    # and the pyrealsense2 SDK, neither of which the factory test above needs.
    from dimos.perception.worldbelief_module import WorldBeliefModule
    from dimos.perception.worldbelief_recorder import WorldBeliefRecorder
    from dimos.robot.manipulators.xarm.blueprints.worldbelief import xarm6_worldbelief
    from dimos.robot.manipulators.xarm.blueprints.worldbelief_sim import xarm6_worldbelief_sim

    modules = {atom.module for atom in xarm6_worldbelief_sim.blueprints}
    assert MujocoSimModule in modules
    assert "RealSenseCamera" not in {module.__name__ for module in modules}

    robot = _module_kwargs(xarm6_worldbelief_sim, ManipulationModule)["robots"][0]
    # MujocoSimModule parents the camera TF to link6; with the gripper attached
    # the tip link is link_tcp, so link6 only gets published if listed here.
    assert "link6" in robot.tf_extra_links
    assert robot.gripper_hardware_id == "arm"
    assert robot.home_joints == XARM6_SIM_HOME
    sim_config = MujocoSimModuleConfig(**_module_kwargs(xarm6_worldbelief_sim, MujocoSimModule))
    assert sim_config.base_frame_id in robot.tf_extra_links

    tasks = _coordinator_tasks(xarm6_worldbelief_sim)
    assert [(task.name, task.type) for task in tasks] == [
        (trajectory_task_name(robot.name), "trajectory")
    ]

    # Shared worldbelief_stack: same perception knobs, distinct recordings.
    sim_belief = _module_kwargs(xarm6_worldbelief_sim, WorldBeliefModule)
    hw_belief = _module_kwargs(xarm6_worldbelief, WorldBeliefModule)
    paths = ("db_path", "history_path")
    assert {k: v for k, v in sim_belief.items() if k not in paths} == {
        k: v for k, v in hw_belief.items() if k not in paths
    }
    for key in paths:
        assert "xarm6_sim" in str(sim_belief[key])
        assert sim_belief[key] != hw_belief[key]
    assert (
        _module_kwargs(xarm6_worldbelief_sim, WorldBeliefRecorder)["db_path"]
        == sim_belief["db_path"]
    )
