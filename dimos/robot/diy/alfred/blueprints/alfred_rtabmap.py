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

"""RTAB-Map on Alfred: drive the robot and watch the trail it leaves.

    dimos run alfred-rtabmap

The mapping half of ``demo-rtabmap`` wired to the real robot: the RealSense feeds
RTAB-Map, ``OdometryPath`` accumulates its visual odometry into ``path``, and
``MovementManager`` drives Alfred from the viewer's teleop keys. No lidar and no
planner, so this does not conflict with ``alfred-nav`` -- FastLIO2 owns
``odometry`` there, RTAB-Map owns it here. Run one or the other.

Drive a loop and come back to where you started: ``path`` keeps whatever drift the
visual odometry accumulated, and the ``map`` -> ``odom`` edge snaps when RTAB-Map
recognises the place, so the trail stays continuous while the map pulls back onto
itself.

The camera is assumed to sit at the body origin. Once the mount is measured, pass
its ``base_transform`` here -- RTAB-Map's pose is the camera's, so an unmeasured
mount offsets the whole trail by it.

Every run also records the camera streams and RTAB-Map's odometry into
``recordings/<timestamp>/rtabmap.db``, so a session that felt laggy can be replayed
into RTAB-Map afterwards instead of re-driving it. The depth stream is stored
losslessly and dominates the write -- expect a few MB/s, and delete old runs.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.odometry_path import OdometryPath
from dimos.mapping.rtab_map.recorder import RtabmapRecorder
from dimos.mapping.rtab_map.rtabmap import RERUN_CONFIG, RtabmapSlam
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.visualization.vis_module import vis_module


def _recording_dir() -> Path:
    now = datetime.now().astimezone()
    stamp = (
        now.strftime("%Y-%m-%d") + "_" + now.strftime("%I-%M%p").lower() + "-" + now.strftime("%Z")
    )
    return RECORDINGS_DIR / stamp


# Resolved on the host at import, so every module in the run agrees on one directory.
_RECORDING_DIR = _recording_dir()

alfred_rtabmap = autoconnect(
    RealSenseCamera.blueprint(
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
    ),
    RtabmapSlam.blueprint(input_mode="rgbd"),
    # The db stream is named after the recorder's port, so RTAB-Map's odometry is
    # recorded as "rtabmap_odometry" rather than a bare "odometry".
    RtabmapRecorder.blueprint(db_path=str(_RECORDING_DIR / "rtabmap.db")).remappings(
        [(RtabmapRecorder, "rtabmap_odometry", "odometry")]
    ),
    # RTAB-Map publishes where the robot is now; this keeps the history so the
    # viewer can draw where it has been.
    OdometryPath.blueprint(),
    # tele_cmd_vel from the viewer -> cmd_vel, which AlfredHighLevel drives on.
    MovementManager.blueprint(),
    AlfredHighLevel.blueprint(),
    vis_module(
        global_config.viewer,
        rerun_config={**RERUN_CONFIG, "memory_limit": "1GB"},
    ),
).global_config(n_workers=9)
