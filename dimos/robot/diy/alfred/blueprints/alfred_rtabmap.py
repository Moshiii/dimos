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

"""Alfred mapping from a RealSense alone, with RTAB-Map instead of the lidar.

    dimos run alfred-rtabmap           # RGB-D: colour + aligned depth
    dimos run alfred-rtabmap-stereo    # stereo: the infrared pair

``alfred_nav`` gets its pose from FastLIO2 and a Mid-360, ``alfred_cuvslam`` from the
D455's infrared pair on the GPU. This runs the same robot on RTAB-Map, which is CPU-only
and so is the one of the three that runs on this NUC's Intel graphics.

To watch it in Rerun the two things to look at are ``odom_path`` / ``map_path`` (the
trajectories, drawn as lines) and ``cloud_map`` (the map). The ``Odometry`` streams render
only as the current pose, which is why the paths are published separately.

**The two modes.** Both end at the same four outputs; they differ in what RTAB-Map tracks
on and where the map's geometry comes from.

``alfred_rtabmap`` (RGB-D) tracks features on the colour image and takes the map straight
from the camera's own depth. The projector's dot pattern is what makes that depth good and
is invisible to the colour imager behind its IR-cut filter, so ``emitter_enabled`` is left
**on** -- the reason cuVSLAM has to turn it off does not apply. Depth is aligned to colour,
which is what lets one ``camera_info`` describe both.

``alfred_rtabmap_stereo`` tracks on the infrared pair and makes RTAB-Map compute disparity
itself. The IR imagers are global-shutter, so they do not smear when the robot turns, and
they ignore the colour sensor's auto-exposure. It costs a dense stereo match per keyframe.
Here the emitter must be **off**: the dots move with the camera rather than the world, so
a feature tracker latches onto them and biases motion toward zero.

15 fps rather than 30: colour at 848x480 is ~18 MB/s of LCM traffic on its own and depth
another ~12 MB/s. The native C++ SDK speaks LCM only -- there is no shared-memory path for
it -- and RTAB-Map's odometry is comfortable at 15 Hz, so the second half of that
bandwidth would buy nothing.

Alfred's drive stack is included so this is a complete robot blueprint. To bring up the
mapping half alone -- on a bench, with only the camera plugged in -- run it as
``dimos run alfred-rtabmap --disable AlfredHighLevel --disable MovementManager``.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.rtab_map.rtabmap import RERUN_CONFIG, RtabmapSlam
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.visualization.vis_module import vis_module

CAMERA_NAME = "d435if"
WIDTH = 848
HEIGHT = 480
FPS = 15
# Distance between the D435's infrared imagers, read off this unit with
# rs.get_extrinsics_to(). A D455 is 0.0949.
IR_BASELINE_M = 0.0499

alfred_rtabmap = autoconnect(
    RealSenseCamera.blueprint(
        camera_name=CAMERA_NAME,
        width=WIDTH,
        height=HEIGHT,
        fps=FPS,
        enable_color=True,
        enable_depth=True,
        # One camera_info then describes both images.
        align_depth_to_color=True,
        # Good depth matters more than clean infrared here; nothing reads the IR pair.
        emitter_enabled=True,
        enable_infrared=False,
        # RTAB-Map builds its own cloud from depth, in the map frame.
        enable_pointcloud=False,
        enable_imu=False,
        # The mount belongs to the URDF; left at its default the camera also
        # publishes base_link -> camera_link and camera_link ends up with two parents.
        base_transform=None,
    ),
    RtabmapSlam.blueprint(input_mode="rgbd"),
    MovementManager.blueprint(),
    AlfredHighLevel.blueprint(),
    vis_module(global_config.viewer, rerun_config=RERUN_CONFIG),
).global_config(n_workers=6)

alfred_rtabmap_stereo = (
    autoconnect(
        RealSenseCamera.blueprint(
            camera_name=CAMERA_NAME,
            width=WIDTH,
            height=HEIGHT,
            fps=FPS,
            enable_infrared=True,
            # The dots move with the camera, not the world, so a feature tracker
            # latches onto them and biases motion toward zero.
            emitter_enabled=False,
            enable_color=False,
            enable_depth=False,
            enable_pointcloud=False,
            enable_imu=False,
            base_transform=None,
        ),
        RtabmapSlam.blueprint(input_mode="stereo_ir", baseline_m=IR_BASELINE_M),
        MovementManager.blueprint(),
        AlfredHighLevel.blueprint(),
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
    .global_config(n_workers=6)
)
