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

"""Agentic xArm manipulation blueprints."""

from __future__ import annotations

from dimos.agents.code_policy import CodePolicyModule
from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.memory2.module import OnExisting, Recorder
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.agent_prompts import (
    BASE_MANIPULATION_AGENT_SYSTEM_PROMPT,
    CODE_POLICY_MANIPULATION_AGENT_SYSTEM_PROMPT,
    MANIPULATION_AGENT_SYSTEM_PROMPT,
)
from dimos.robot.manipulators.xarm.blueprints.basic import xarm7_planner_coordinator
from dimos.robot.manipulators.xarm.blueprints.perception import xarm_perception
from dimos.robot.manipulators.xarm.blueprints.simulation import xarm_perception_sim
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

_XARM_SIM_POLICY_RECORDING_PATH = (
    STATE_DIR / "code_policy" / "xarm_perception_sim" / "recordings" / "observations.db"
)


class _CodePolicyDemoRecorder(Recorder):
    """Record representative simulator observations for the code-policy demo."""

    coordinator_joint_state: In[JointState]
    color_image: In[Image]


xarm7_planner_coordinator_agent = autoconnect(
    xarm7_planner_coordinator,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=BASE_MANIPULATION_AGENT_SYSTEM_PROMPT),
)

xarm_perception_agent = autoconnect(
    xarm_perception,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=MANIPULATION_AGENT_SYSTEM_PROMPT),
)

xarm_perception_sim_agent = autoconnect(
    xarm_perception_sim,
    _CodePolicyDemoRecorder.blueprint(
        db_path=_XARM_SIM_POLICY_RECORDING_PATH,
        on_existing=OnExisting.OVERWRITE,
        record_tf=False,
    ),
    CodePolicyModule.blueprint(
        recording_path=str(_XARM_SIM_POLICY_RECORDING_PATH),
    ),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=CODE_POLICY_MANIPULATION_AGENT_SYSTEM_PROMPT),
).remappings(
    [
        # Mujoco's raw cloud and scene registration's derived cloud otherwise
        # share one transport. Policies should observe the registered scene.
        (MujocoSimModule, "pointcloud", "raw_pointcloud"),
        (ObjectSceneRegistrationModule, "pointcloud", "pointcloud"),
    ]
)
