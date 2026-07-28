#!/usr/bin/env python3
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

"""City around a recorded drone flight — no drone needed.

The recording's streams stand in for the live platform (the replay module's
aliases map ``gps_location``/``odom``/``color_image`` onto the standard
topics); :class:`CityMeshModule` streams tiles around the flight's fixes and
:class:`EnuSnapTF` places them in the drone's north-aligned world — the
compass-equipped registration path, which the autopilot's EKF satisfies.

    dimos --replay-db flight-25hz.db run drone-city-replay
"""

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.navigation.tracer import Tracer
from dimos.robot.drone.replay import DroneReplay
from dimos.visualization.citymesh.enu_tf import EnuSnapTF
from dimos.visualization.citymesh.module import CityMeshModule
from dimos.visualization.ground_frustum import GroundFrustumModule, ground_frustum_override
from dimos.visualization.rerun import shapes
from dimos.visualization.vis_module import vis_module

# The recorded autopilot reports height above takeoff, not MSL; the snap adds
# the pad's elevation back (DEM ground at the recording site). Goes away once
# the drone connection publishes MSL altitudes.
TAKEOFF_MSL_M = 90.0

# One set of intrinsics for everything camera: the replay's synthesized
# camera_info topic, the frustum the viewer hangs the video on, and the
# AprilTag pose solver. fx/fy as fitted in drone-basic against two Mini 4 Pro
# flights — the 1000.0 that was here is a placeholder and 31 % low, which
# widens the frustum's cone and oversizes the frame it drops on the city.
DRONE_INTRINSICS = (1457.1, 1457.1, 960.0, 540.0)
DRONE_RESOLUTION = (1920, 1080)
DRONE_CAMERA_INFO = CameraInfo.from_intrinsics(
    *DRONE_INTRINSICS, *DRONE_RESOLUTION, frame_id="drone/camera_optical"
)

# The frustum alone draws tighter than the calibration says: eyeballed against
# the city, the fitted cone still overspills the ground the frame covers. A
# longer focal length narrows it — 0.7 here is "30 % less wide", so the divide.
# Kept off DRONE_CAMERA_INFO on purpose: that one is measurement, this is taste.
FRUSTUM_WIDTH_SCALE = 0.7
FRUSTUM_CAMERA_INFO = CameraInfo.from_intrinsics(
    DRONE_INTRINSICS[0] / FRUSTUM_WIDTH_SCALE,
    DRONE_INTRINSICS[1] / FRUSTUM_WIDTH_SCALE,
    DRONE_INTRINSICS[2],
    DRONE_INTRINSICS[3],
    *DRONE_RESOLUTION,
    frame_id="drone/camera_optical",
)

# Printed tag edge length; adjust to the tags actually on the wall.
APRILTAG_EDGE_M = 0.1


class OdometryTracer(Tracer):
    """Breadcrumb of where the robot has walked, on world/odometry_path."""

    odometry: In[Odometry]


def _camera_info_to_pinhole(msg: Any) -> Any:
    """Pinhole onto the video entity, which it must share to project it.

    No ``optical_frame``: the video self-binds to its tf frame, a second
    parent would conflict.
    """
    return msg.to_rerun(image_topic="world/video")


def _fat_path(msg: Any) -> Any:
    """Breadcrumb thick enough to read from city altitude."""
    return msg.to_rerun(radii=0.1, color=[255, 90, 60])


def _rerun_blueprint() -> Any:
    """Camera + 3D world with a City tab, as the go2 city blueprint has."""
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world/video", name="Camera"),
            rrb.Tabs(
                rrb.Spatial3DView(
                    origin="world",
                    name="3D",
                    background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                    line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
                ),
                rrb.Spatial3DView(
                    origin="world/city",
                    name="City",
                    background=rrb.Background(kind="SolidColor", color=[6, 16, 48]),
                ),
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


drone_city_replay = autoconnect(
    DroneReplay.blueprint(
        dataset=global_config.replay_db,
        camera_intrinsics=DRONE_INTRINSICS,
        camera_resolution=DRONE_RESOLUTION,
    ),
    CityMeshModule.blueprint(),
    OdometryTracer.blueprint(),
    EnuSnapTF.blueprint(
        parent="drone/world",
        robot_frame="drone/base_link",
        altitude_offset_m=TAKEOFF_MSL_M,
    ),
    # The gimbal flies nadir, so the frame it sees belongs on the city, not on
    # a metre-long stub at the drone. This tracks how far out the plane sits;
    # the matching override below hangs the video's image plane there. Pinned
    # to the landing point itself — drop ``ground_point`` for the horizontal
    # ground_z plane instead, or lift it to (0, 0, 3) to keep the frame off the
    # mesh it otherwise fights for pixels.
    GroundFrustumModule.blueprint(
        world_frame="drone/world",
        camera_frame="drone/camera_optical",
        ground_point=(0.0, 0.0, 0.0),
    ),
    # AprilTags, straight from the H.264 stream: the H264InputMixin decodes
    # into the recognizer's own image input. DICT_APRILTAG_36h11 markers land
    # on tf under drone/world, posed via the recorded tree (world -> optical).
    # VideoMarkerDetectionModule.blueprint(
    #     marker_length_m=APRILTAG_EDGE_M,
    #     camera_info=DRONE_CAMERA_INFO,
    #     world_frame="drone/world",
    #     decode_hz=5.0,
    # ),
    # MarkerTfModule.blueprint(world_frame="drone/world"),
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config={
            "blueprint": _rerun_blueprint,
            "visual_override": {
                "world/odometry_path": _fat_path,
                **ground_frustum_override(FRUSTUM_CAMERA_INFO, image_topic="world/video"),
            },
            "models": {"drone/base_link": shapes.quadcopter()},
        },
    ),
).global_config(transport="zenoh", n_workers=7)
