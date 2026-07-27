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
from dimos.navigation.tracer import Tracer
from dimos.robot.unitree.go2.zenoh.replay import GO2ZenohReplay
from dimos.visualization.citymesh.enu_tf import EnuSnapTF
from dimos.visualization.vis_module import vis_module

# The recorded autopilot reports height above takeoff, not MSL; the snap adds
# the pad's elevation back (DEM ground at the recording site). Goes away once
# the drone connection publishes MSL altitudes.
TAKEOFF_MSL_M = 70.0


class OdometryTracer(Tracer):
    """Breadcrumb of where the robot has walked, on world/odometry_path."""

    odometry: In[Odometry]


def _camera_info_to_pinhole(msg: Any) -> Any:
    """Pinhole onto the video entity, hung on the optical tf frame.

    Same trick as the go2 city blueprint: the pinhole must land on the
    video's own entity to project it, and parenting it to the tf frame
    poses the frustum in the world view.
    """
    return msg.to_rerun(image_topic="world/video", optical_frame=msg.frame_id)


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
    GO2ZenohReplay.blueprint(dataset=global_config.replay_db),
    CityMeshModule.blueprint(),
    OdometryTracer.blueprint(),
    EnuSnapTF.blueprint(
        parent="drone/world",
        robot_frame="drone/base_link",
        altitude_offset_m=TAKEOFF_MSL_M,
    ),
    vis_module(
        viewer_backend=global_config.viewer,
        rerun_config={
            "blueprint": _rerun_blueprint,
            "visual_override": {"world/camera_info": _camera_info_to_pinhole},
        },
    ),
).global_config(transport="zenoh", n_workers=5)
