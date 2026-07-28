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

"""xArm6 WorldBelief perception stack against the MuJoCo sim.

Mirrors :mod:`worldbelief` (xarm6_worldbelief) with the RealSense wrist camera
replaced by :class:`MujocoSimModule`, which renders the MJCF ``wrist_camera``
and publishes the same color/depth/camera-info topics plus the
``link6 -> wrist_camera_*_optical_frame`` TF from simulation ground truth. The
perception half of the stack is shared verbatim via ``worldbelief_stack``.

KNOWN LIMITATION -- detected object poses read low in Z. The ``world -> link6``
TF edge comes from ManipulationModule FK, which places ``link_base`` at the
world origin, but ``data/xarm6/scene.xml`` spawns it on a 0.12m pedestal. The
sim renders from the true (pedestal) camera pose while perception unprojects
using the FK pose, so scanned objects land ~9.5cm below ground truth and IK
targets derived from them are unreachable. Scan/detect/recall work; pick does
not. Fixing the base-frame disagreement is tracked separately.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.robot.manipulators.common.blueprints import coordinator, trajectory_task
from dimos.robot.manipulators.xarm.blueprints.worldbelief import worldbelief_stack
from dimos.robot.manipulators.xarm.config import (
    XARM6_SIM_HOME,
    XARM6_SIM_PATH,
    make_xarm6_model_config,
    make_xarm6_sim_hardware,
    make_xarm6_sim_module_kwargs,
)
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

_hw = make_xarm6_sim_hardware(XARM6_SIM_PATH)

xarm6_worldbelief_sim = autoconnect(
    # Provides world->link6 FK/TF (the sim camera's TF parent) and the gripper
    # skill path (set_gripper -> coordinator set_gripper_position RPC).
    ManipulationModule.blueprint(
        robots=[
            make_xarm6_model_config(
                name="arm",
                add_gripper=True,
                # link6 parents the camera; with the gripper attached the tip
                # link is link_tcp, so link6 is no longer published implicitly.
                tf_extra_links=["link6", "link_base"],
                home_joints=XARM6_SIM_HOME,
            ),
        ],
    ),
    MujocoSimModule.blueprint(**make_xarm6_sim_module_kwargs(XARM6_SIM_PATH)),
    worldbelief_stack("xarm6_sim"),
    coordinator(
        hardware=[_hw],
        tasks=[trajectory_task(_hw)],
    ),
).global_config(n_workers=8)
