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

from dimos.agents.code_policy import CodePolicyModule
from dimos.agents.mcp.mcp_client import McpClient
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.module import ModuleBase
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.memory2.module import OnExisting, Recorder
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.xarm.blueprints.agentic import (
    _XARM_SIM_POLICY_RECORDING_PATH,
    _CodePolicyDemoRecorder,
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
    assert sim_modules.count(_CodePolicyDemoRecorder) == 1
    assert sum(issubclass(module, Recorder) for module in sim_modules) == 1
    assert CodePolicyModule not in real_modules
    assert all(not issubclass(module, Recorder) for module in real_modules)


def test_sim_agent_configures_ephemeral_demo_observation_database() -> None:
    recorder = next(
        atom
        for atom in xarm_perception_sim_agent.blueprints
        if atom.module is _CodePolicyDemoRecorder
    )
    code_policy = next(
        atom for atom in xarm_perception_sim_agent.blueprints if atom.module is CodePolicyModule
    )

    assert {stream.name for stream in recorder.streams} == {
        "color_image",
        "coordinator_joint_state",
    }
    assert recorder.kwargs == {
        "db_path": _XARM_SIM_POLICY_RECORDING_PATH,
        "on_existing": OnExisting.OVERWRITE,
        "record_tf": False,
    }
    assert code_policy.kwargs["recording_path"] == str(_XARM_SIM_POLICY_RECORDING_PATH)


def test_demo_recording_supports_attached_live_reader(tmp_path: Path) -> None:
    path = tmp_path / "observations.db"
    recorder = _CodePolicyDemoRecorder(
        db_path=path,
        on_existing=OnExisting.OVERWRITE,
        record_tf=False,
    )
    writer = recorder.store.stream("coordinator_joint_state", JointState)
    writer.append(JointState(position=[0.1]), ts=1.0)

    reader = SqliteStore(path=path, must_exist=True)
    reader.start()
    try:
        assert reader.streams.coordinator_joint_state.last().data.position == [0.1]

        writer.append(JointState(position=[0.2]), ts=2.0)

        assert reader.streams.coordinator_joint_state.last().data.position == [0.2]
        assert len(reader.streams.coordinator_joint_state.after(0).to_list()) == 2
    finally:
        reader.stop()
        recorder.stop()


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
