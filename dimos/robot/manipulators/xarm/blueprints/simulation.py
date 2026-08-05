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

"""Simulation xArm perception manipulation blueprints."""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.grasping.grasp_gen_x import GraspGenXModule
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.manipulation.visualization.rerun import picknplace_rerun_config
from dimos.perception.object_scene_registration import ObjectSceneRegistrationModule
from dimos.perception.sim_object_scene import SimObjectScene
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    XARM_GRASP_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.utils.data import LfsPath
from dimos.visualization.vis_module import vis_module


def _xarm7_perception_sim(
    scene_path: object,
    static_box_obstacles: tuple = (),
    object_scene: object | None = None,
    pick_and_place_kwargs: dict[str, object] | None = None,
) -> object:
    hw = make_xarm7_sim_hardware(scene_path)
    return autoconnect(
        PickAndPlaceModule.blueprint(
            robots=[make_xarm7_sim_robot_config()],
            planning_timeout=10.0,
            visualization={"backend": "viser"},
            heuristic_grasp_fallback=True,
            static_box_obstacles=list(static_box_obstacles),
            **(pick_and_place_kwargs or {}),
        ),
        MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(scene_path)),
        object_scene or ObjectSceneRegistrationModule.blueprint(target_frame="world"),
        coordinator(hardware=[hw], tasks=[trajectory_task(hw)]),
        vis_module(global_config.viewer, rerun_config=picknplace_rerun_config()),
    )


xarm_perception_sim = autoconnect(_xarm7_perception_sim(XARM7_SIM_PATH))

# The room-and-objects scene with learned grasps: GraspGenX proposals feed
# pick's provider path, and the table matches data/xarm_grasp_sim/scene.xml so
# the planner always respects it.
XARM_GRASP_TABLE = {"name": "table", "center": (0.47, 0.0, 0.065), "size": (0.38, 0.60, 0.13)}

# Ground-truth detections from sim state instead of the camera: perception is
# the weak link in this scene, and grasping is what we are testing.
_XARM_GRASPGENX = make_xarm_graspgenx_config()
_XARM_GRASP_MESH_DIR = LfsPath("xarm_grasp_sim") / "assets" / "manip"
_XARM_GRASP_OBJECTS = {
    name: str(_XARM_GRASP_MESH_DIR / f"{name}.obj")
    for name in ("bottle", "box", "can", "cup", "marker", "tape")
}

_XARM_GRASP_PICK_KWARGS: dict[str, object] = {
    "max_grasp_candidates_to_check": 30,
    "grasp_pre_grasp_offset": 0.25,
    "grasp_retreat_offset": 0.10,
    "grasp_retreat_lift_offset": 0.01,
    "grasp_viz_gripper": _XARM_GRASPGENX.gripper,
    "grasp_viz_frame_to_tcp": _XARM_GRASPGENX.grasp_frame_to_tcp,
    "use_mesh_obstacles": True,
}


def _xarm_grasp_scene(object_scene: object) -> object:
    return _xarm7_perception_sim(
        XARM_GRASP_SIM_PATH,
        static_box_obstacles=(XARM_GRASP_TABLE,),
        object_scene=object_scene,
        pick_and_place_kwargs=_XARM_GRASP_PICK_KWARGS,
    )


def _xarm_graspgenx() -> object:
    return GraspGenXModule.blueprint(
        **_XARM_GRASPGENX.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    )


xarm_grasp_sim = autoconnect(
    _xarm_grasp_scene(SimObjectScene.blueprint(objects=_XARM_GRASP_OBJECTS)),
    _xarm_graspgenx(),
)

# The same scene through the real camera pipeline: MujocoSimModule's wrist camera
# feeds ObjectSceneRegistrationModule with the hardware blueprint's tuning
# (dimos/robot/manipulators/xarm/blueprints/perception.py), so perception is back
# in the loop and the ground-truth scene above is the reference to compare against.
xarm_grasp_sim_perception = autoconnect(
    _xarm_grasp_scene(
        ObjectSceneRegistrationModule.blueprint(
            target_frame="world",
            distance_threshold=0.08,
            max_distance=1.0,
            use_aabb=True,
            max_obstacle_width=0.06,
            # The wrist camera sees the arm's own links mid-trajectory and YOLO-E
            # registers them as objects; only detect on request, from still frames.
            # Each request is a single view, so promotion cannot require several.
            detect_on_request=True,
            min_detections_for_permanent=1,
        )
    ),
    _xarm_graspgenx(),
)
