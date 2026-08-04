# Copyright 2025-2026 Dimensional Inc.
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

"""Compatibility exports for manipulation blueprints.

Robot-owned manipulation blueprints now live under ``dimos.robot.manipulators``.
"""

import math

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.picknplace import PickNPlaceModule
from dimos.manipulation.visualization.rerun import picknplace_rerun_config
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.detection.detectors.yoloe import YoloePromptMode
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.blueprints.agentic import (
    xarm7_planner_coordinator_agent as xarm7_planner_coordinator_agent,
    xarm_perception_agent as xarm_perception_agent,
    xarm_perception_sim_agent as xarm_perception_sim_agent,
)
from dimos.robot.manipulators.xarm.blueprints.basic import (
    xarm7_planner_coordinator as xarm7_planner_coordinator,
)
from dimos.robot.manipulators.xarm.blueprints.perception import xarm_perception as xarm_perception
from dimos.robot.manipulators.xarm.blueprints.simulation import (
    XARM_GRASP_TABLE,
    xarm_perception_sim as xarm_perception_sim,
)
from dimos.robot.manipulators.xarm.config import (
    XARM_GRASP_SIM_PATH,
    make_xarm6_model_config,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
    xarm6_hardware,
)
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.vis_module import vis_module

PICKNPLACE_CAMERA_TRANSFORM = Transform(
    translation=Vector3(0.06693724, -0.0309563, 0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),
)

_picknplace_xarm6_hardware = xarm6_hardware("arm", gripper=True)
_picknplace_xarm6_model = make_xarm6_model_config(
    name="arm",
    add_gripper=True,
    tf_extra_links=["link_base", "link6"],
    home_joints=[0.0, math.radians(-40.0), math.radians(-50.0), 0.0, math.radians(90.0), 0.0],
)
_picknplace_xarm6_model.max_velocity = 0.25
_picknplace_xarm6_model.max_acceleration = 0.5
_xarm_graspgenx = make_xarm_graspgenx_config()
_picknplace_sim_hardware = make_xarm7_sim_hardware(XARM_GRASP_SIM_PATH)


picknplace = autoconnect(
    coordinator(
        hardware=[_picknplace_xarm6_hardware],
        tasks=[trajectory_task(_picknplace_xarm6_hardware)],
    ),
    ManipulationModule.blueprint(
        robots=[_picknplace_xarm6_model],
        visualization=ViserVisualizationConfig(port=8095),
        floor_z=0.0,
        planning_timeout=10.0,
    ),
    RealSenseCamera.blueprint(
        width=848,
        height=480,
        fps=15,
        camera_name="camera",
        base_frame_id="link6",
        base_transform=PICKNPLACE_CAMERA_TRANSFORM,
        enable_depth=True,
        align_depth_to_color=True,
        enable_pointcloud=False,
    ),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="link_base",
        prompt_mode=YoloePromptMode.LRPC,
        register_objects=False,
        detect_on_request=True,
        detector_confidence=0.4,
        object_voxel_downsample=0.001,
    ),
    PickNPlaceModule.blueprint(align_grasp_yaw=True),
    vis_module(
        global_config.viewer,
        rerun_config=picknplace_rerun_config(),
    ),
).global_config(rerun_open="web")


picknplace_graspgenx = autoconnect(
    coordinator(
        hardware=[_picknplace_xarm6_hardware],
        tasks=[trajectory_task(_picknplace_xarm6_hardware)],
    ),
    ManipulationModule.blueprint(
        robots=[_picknplace_xarm6_model],
        visualization=ViserVisualizationConfig(port=8095),
        floor_z=0.0,
        planning_timeout=10.0,
    ),
    RealSenseCamera.blueprint(
        width=848,
        height=480,
        fps=15,
        camera_name="camera",
        base_frame_id="link6",
        base_transform=PICKNPLACE_CAMERA_TRANSFORM,
        enable_depth=True,
        align_depth_to_color=True,
        enable_pointcloud=False,
    ),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="link_base",
        prompt_mode=YoloePromptMode.LRPC,
        register_objects=False,
        detect_on_request=True,
        detector_confidence=0.4,
        object_voxel_downsample=0.001,
    ),
    PickNPlaceModule.blueprint(align_grasp_yaw=True, grasp_strategy="graspgenx"),
    GraspGenXModule.blueprint(
        **_xarm_graspgenx.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    ),
    vis_module(
        global_config.viewer,
        rerun_config=picknplace_rerun_config(),
    ),
).global_config(rerun_open="web")


picknplace_graspgenx_edgetam = autoconnect(
    coordinator(
        hardware=[_picknplace_xarm6_hardware],
        tasks=[trajectory_task(_picknplace_xarm6_hardware)],
    ),
    ManipulationModule.blueprint(
        robots=[_picknplace_xarm6_model],
        visualization=ViserVisualizationConfig(port=8095),
        floor_z=0.0,
        planning_timeout=10.0,
    ),
    RealSenseCamera.blueprint(
        width=848,
        height=480,
        fps=15,
        camera_name="camera",
        base_frame_id="link6",
        base_transform=PICKNPLACE_CAMERA_TRANSFORM,
        enable_depth=True,
        align_depth_to_color=True,
        enable_pointcloud=False,
    ),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="link_base",
        prompt_mode=YoloePromptMode.PROMPT,
        segmentation_backend="edgetam",
        register_objects=False,
        detect_on_request=True,
        detector_confidence=0.4,
        object_voxel_downsample=0.001,
    ),
    PickNPlaceModule.blueprint(align_grasp_yaw=True, grasp_strategy="graspgenx"),
    GraspGenXModule.blueprint(
        **_xarm_graspgenx.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    ),
    vis_module(
        global_config.viewer,
        rerun_config=picknplace_rerun_config(),
    ),
).global_config(rerun_open="web")


# The EdgeTAM workflow above against the MuJoCo grasp scene: the wrist camera
# replaces the RealSense and the xArm7 sim replaces the xArm6 hardware, so the
# prompted-segmentation path can be exercised without a robot.
picknplace_graspgenx_edgetam_sim = autoconnect(
    coordinator(
        hardware=[_picknplace_sim_hardware],
        tasks=[trajectory_task(_picknplace_sim_hardware)],
    ),
    ManipulationModule.blueprint(
        robots=[make_xarm7_sim_robot_config()],
        visualization=ViserVisualizationConfig(port=8095),
        static_box_obstacles=[XARM_GRASP_TABLE],
        planning_timeout=10.0,
    ),
    MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(XARM_GRASP_SIM_PATH)),
    ObjectSceneRegistrationModule.blueprint(
        target_frame="world",
        prompt_mode=YoloePromptMode.PROMPT,
        segmentation_backend="edgetam",
        register_objects=False,
        detect_on_request=True,
        detector_confidence=0.4,
        object_voxel_downsample=0.001,
    ),
    PickNPlaceModule.blueprint(align_grasp_yaw=True, grasp_strategy="graspgenx"),
    GraspGenXModule.blueprint(
        **_xarm_graspgenx.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    ),
    vis_module(
        global_config.viewer,
        rerun_config=picknplace_rerun_config(),
    ),
).global_config(rerun_open="web")
