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

"""Real xArm6 prompted-localization grasp pipeline."""

from __future__ import annotations

from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import STATE_DIR
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import Out
from dimos.experimental.world_belief.worldbelief_recorder import WorldBeliefRecorder
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.pick_and_place_module import PromptedPickAndPlaceModule
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.perception.memory.prompted_object_localizer import PromptedObjectLocalizerModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.camera_config import XARM6_WRIST_CAMERA_TRANSFORM
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config, xarm6_hardware
from dimos.robot.manipulators.xarm.grasp_config import (
    XARM_GRASP_FRAME_TO_TCP,
    XARM_GRIPPER_SWEEP,
    make_xarm_graspgenx_config,
)

_RECORDING_PATH = STATE_DIR / "worldbelief" / "xarm6" / "recordings" / "xarm6_prompted_pick.db"
_GRASPGENX_CONFIG = make_xarm_graspgenx_config()

_hardware = xarm6_hardware("arm", gripper=True)
_hardware.auto_enable = True


class _XArm6PromptedPickCoordinator(ControlCoordinator):
    arm_joints: Out[JointState]


xarm6_prompted_pick = (
    autoconnect(
        PromptedPickAndPlaceModule.blueprint(
            robots=[
                make_xarm6_model_config(
                    name="arm",
                    add_gripper=True,
                    tf_extra_links=["link_base"],
                )
            ],
            planning_timeout=10.0,
            visualization={"backend": "viser"},
            floor_z=-0.02,
            planning_frame="world",
            grasp_approach_vector=(0.0, 0.0, -1.0),
            grasp_visualization={
                "gripper": XARM_GRIPPER_SWEEP,
                "grasp_frame_to_tcp": XARM_GRASP_FRAME_TO_TCP,
            },
            grasp_verification={
                "enabled": False,
                "open_position": 0.85,
                "closed_position": 0.0,
                "held_threshold": 0.02,
                "timeout": 2.0,
                "poll_interval": 0.05,
            },
        ),
        RealSenseCamera.blueprint(
            width=640,
            height=480,
            fps=15,
            base_frame_id="link6",
            base_transform=XARM6_WRIST_CAMERA_TRANSFORM,
        ),
        WorldBeliefRecorder.blueprint(db_path=_RECORDING_PATH),
        PromptedObjectLocalizerModule.blueprint(db_path=_RECORDING_PATH),
        GraspGenXModule.blueprint(
            **_GRASPGENX_CONFIG.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
        ),
        McpServer.blueprint(),
        coordinator(
            cls=_XArm6PromptedPickCoordinator,
            instance_name="ControlCoordinator",
            publish_robot_joint_states=True,
            hardware=[_hardware],
            tasks=[trajectory_task(_hardware)],
        ),
    )
    .remappings([(WorldBeliefRecorder, "coordinator_joint_state", "arm_joints")])
    .global_config(n_workers=7)
)
