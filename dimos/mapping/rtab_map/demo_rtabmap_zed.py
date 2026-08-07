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

"""RTAB-Map mapping around a ZED's own tracking pose.

    dimos run demo-rtabmap-zed

Kept in its own module because ``ZEDCamera`` imports the proprietary ``pyzed`` SDK at
import time. Putting this beside the RealSense demos would make those unimportable on
any machine without the ZED SDK installed.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.zed.camera import ZEDCamera
from dimos.mapping.odometry_path import OdometryPath
from dimos.mapping.rtab_map.rtabmap import RERUN_CONFIG, RtabmapSlam
from dimos.visualization.vis_module import vis_module

# A ZED instead of the RealSense, and RTAB-Map taking the pose rather than computing
# it. The ZED's colour imagers are rolling shutter, which RTAB-Map's own odometry
# cannot compensate for -- it has one timestamp per frame and no per-row model. The
# ZED SDK has both the per-row timing and the IMU, so letting it own odom->base_link
# and giving RTAB-Map only the graph and map->odom is the arrangement that actually
# handles the shutter.
demo_rtabmap_zed = (
    autoconnect(
        ZEDCamera.blueprint(
            enable_tracking=True,
            enable_depth=True,
            enable_pointcloud=False,
            base_transform=None,
        ),
        RtabmapSlam.blueprint(
            input_mode="rgbd",
            use_external_odometry=True,
            # The ZED publishes tracking at frame rate, so a frame's pose is never far away.
            external_odometry_timeout_s=0.1,
        ),
        OdometryPath.blueprint(),
        vis_module(global_config.viewer, rerun_config=RERUN_CONFIG),
    )
    .remappings(
        [
            # ZEDCamera publishes its SDK pose as `odometry`; RtabmapSlam consumes it as
            # `external_odometry` and does not publish odom->base_link itself.
            (ZEDCamera, "odometry", "external_odometry"),
        ]
    )
    .global_config(n_workers=4)
)
