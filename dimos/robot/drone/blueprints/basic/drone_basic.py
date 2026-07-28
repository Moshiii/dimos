#!/usr/bin/env python3

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

"""Basic drone blueprint with connection, camera, and visualization."""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.robot.drone.camera_module import DroneCameraModule
from dimos.robot.drone.connection_module import DroneConnectionModule
from dimos.visualization.vis_module import vis_module


def _static_drone_body(rr: Any) -> list[Any]:
    """Static visualization of drone body."""
    return [
        rr.Boxes3D(
            half_sizes=[0.25, 0.25, 0.1],
            colors=[(255, 100, 0)],
        ),
        rr.Transform3D(parent_frame="tf#/base_link"),
    ]


def _drone_rerun_blueprint() -> Any:
    """Split layout: camera feed + 3D world view side by side."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/video", name="Camera"),
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
            ),
            column_shares=[1, 2],
        ),
    )


_rerun_config = {
    "blueprint": _drone_rerun_blueprint,
    "static": {
        "world/tf/base_link": _static_drone_body,
    },
}

_vis = vis_module(global_config.viewer, rerun_config=_rerun_config)

# Determine connection string based on replay flag
connection_string = "udp:0.0.0.0:14550"
video_port = 5600
if global_config.replay:
    connection_string = "replay"

drone_basic = autoconnect(
    _vis,
    DroneConnectionModule.blueprint(
        connection_string=connection_string,
        video_port=video_port,
        outdoor=False,
    ),
    # fx, fy fitted from two Mini 4 Pro flights against recorded altitude (1465.9 and 1448.3 px
    # at 1920 wide, agreeing to 1.2 %). The 1000.0 that was here is a placeholder and 31 % low,
    # which puts a tag at 3 m more than two metres closer than it is.
    #
    # cx, cy stay at the image centre: the principal point is still unmeasured. Do NOT substitute
    # the nadir pixel (970.7, 615.2) measured on 2026-07-28 — that is the principal point and the
    # gimbal's pointing error summed, and it belongs to a controller, not to an intrinsics matrix.
    #
    # Same numbers in ROS CameraInfo form, loadable by CameraInfo.from_yaml:
    #   mini4pro/calib/mini4pro_gimbal_1080.yaml
    DroneCameraModule.blueprint(camera_intrinsics=[1457.1, 1457.1, 960.0, 540.0]),
)
