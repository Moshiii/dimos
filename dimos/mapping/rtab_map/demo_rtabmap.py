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

"""RTAB-Map on a RealSense, with nothing else attached.

    dimos run demo-rtabmap           # RGB-D: colour + aligned depth
    dimos run demo-rtabmap-stereo    # stereo: the infrared pair

A camera, RTAB-Map, and the viewer. No drive stack and no robot, so this runs on a bench
with only the camera plugged in -- pick it up, walk it around a room, and watch
``odom_path`` and ``cloud_map`` in Rerun. ``alfred_rtabmap`` is the same mapping half
wired into the real robot.

Walk a loop and come back to where you started: ``map_path`` snaps onto itself when
RTAB-Map recognises the place, while ``odom_path`` keeps whatever drift it accumulated.
Seeing those two diverge and then re-converge is the whole point of the pose graph.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.rtab_map.rtabmap import RERUN_CONFIG, RtabmapSlam
from dimos.visualization.vis_module import vis_module

WIDTH = 848
HEIGHT = 480
FPS = 15
# Measured off the D435if on this NUC; a D455 is 0.0949.
IR_BASELINE_M = 0.0499

demo_rtabmap = autoconnect(
    RealSenseCamera.blueprint(
        width=WIDTH,
        height=HEIGHT,
        fps=FPS,
        enable_color=True,
        enable_depth=True,
        # One camera_info then describes both images.
        align_depth_to_color=True,
        # The dot pattern is what makes the depth good, and the colour imager cannot
        # see it through its IR-cut filter.
        emitter_enabled=True,
        enable_infrared=False,
        enable_pointcloud=False,
        enable_imu=False,
        # Nothing here owns a robot body, so the camera is the origin.
        base_transform=None,
    ),
    RtabmapSlam.blueprint(input_mode="rgbd"),
    vis_module(global_config.viewer, rerun_config=RERUN_CONFIG),
).global_config(n_workers=4)

demo_rtabmap_stereo = (
    autoconnect(
        RealSenseCamera.blueprint(
            width=WIDTH,
            height=HEIGHT,
            fps=FPS,
            enable_infrared=True,
            # The dots move with the camera rather than the world, so a feature
            # tracker latches onto them and biases motion toward zero.
            emitter_enabled=False,
            enable_color=False,
            enable_depth=False,
            enable_pointcloud=False,
            enable_imu=False,
            base_transform=None,
        ),
        RtabmapSlam.blueprint(input_mode="stereo_ir", baseline_m=IR_BASELINE_M),
        vis_module(global_config.viewer, rerun_config=RERUN_CONFIG),
    )
    .remappings(
        [
            (RealSenseCamera, "infrared_left", "image_left"),
            (RealSenseCamera, "infrared_right", "image_right"),
            # In this mode the rig is the infrared pair, so its intrinsics are the
            # ones RTAB-Map must be given.
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
        ]
    )
    .global_config(n_workers=4)
)
