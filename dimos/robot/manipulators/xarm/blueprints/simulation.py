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
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.perception.experimental.object_scene_registration import ObjectSceneRegistrationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.config import (
    XARM7_SIM_PATH,
    XARM_GRASP_SIM_PATH,
    make_xarm7_sim_hardware,
    make_xarm7_sim_module_kwargs,
    make_xarm7_sim_robot_config,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.visualization.rerun.bridge import RerunBridgeModule


def _xarm7_perception_sim(scene_path: object) -> object:
    hw = make_xarm7_sim_hardware(scene_path)
    return autoconnect(
        PickAndPlaceModule.blueprint(
            robots=[make_xarm7_sim_robot_config()],
            planning_timeout=10.0,
            visualization={"backend": "viser"},
            heuristic_grasp_fallback=True,
        ),
        MujocoSimModule.blueprint(**make_xarm7_sim_module_kwargs(scene_path)),
        ObjectSceneRegistrationModule.blueprint(target_frame="world"),
        coordinator(hardware=[hw], tasks=[trajectory_task(hw)]),
        RerunBridgeModule.blueprint(),
    )


xarm_perception_sim = _xarm7_perception_sim(XARM7_SIM_PATH)

# Same stack on the room-and-objects scene instead of the bare stock table.
xarm_grasp_sim = _xarm7_perception_sim(XARM_GRASP_SIM_PATH)
