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

"""The camera's image plane pushed down to the ground it is looking at.

A pinhole's frustum is drawn at a fixed ``image_plane_distance`` — a metre,
by default, which from 100 m up is a speck at the drone. Stretch that
distance to where the optical axis meets the ground and the frustum spans
the flight instead: the video frame lands on the city, roughly where the
camera is actually seeing it.

Only "roughly", and only while the gimbal points down: rerun draws the image
plane perpendicular to the optical axis, so the plane coincides with the
ground exactly at nadir and tilts off it from there. Past
``nadir_tolerance_deg`` the frustum snaps back to its plain short self rather
than hanging a wall over the map.

The module publishes the meeting point, not the drawing: a
:class:`PointStamped` on the optical axis, in the camera's own frame, which
is a fact about the flight and rides the normal typed transport.
:func:`ground_frustum_override` is what turns it into a pinhole on the video
entity — the entity a pinhole must share to project onto it.
"""

from __future__ import annotations

from collections.abc import Callable
from functools import partial
import math
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.visualization.rerun.bridge import RerunData

logger = setup_logger()


class Config(ModuleConfig):
    world_frame: str = "drone/world"
    camera_frame: str = "drone/camera_optical"
    # Height of the plane the frustum stops at, in ``world_frame`` — the pad a
    # drone took off from, or a little above it to keep the frame off the mesh.
    ground_z: float = 0.0
    # Pin the plane to one world point instead of that horizontal plane — the
    # landing point, say. It then stops where the point is rather than where
    # the ground is under the camera; the two agree while the drone is over it.
    ground_point: tuple[float, float, float] | None = None
    # How far off straight-down the camera may point before the stretched
    # plane is more lie than picture.
    nadir_tolerance_deg: float = 30.0
    # Fallback range when the camera is not looking down (rerun's own default
    # image plane distance).
    near_distance_m: float = 1.0
    max_distance_m: float = 2000.0


class GroundFrustumModule(Module):
    """Publishes where the camera's optical axis meets the ground."""

    config: Config

    # Only a clock: the pose comes from tf, which has no stream to tick on.
    odometry: In[Odometry]
    tf: In[TFMessage]
    ground_frustum: Out[PointStamped]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(self.odometry.observable().subscribe(self._on_odometry))  # type: ignore[no-untyped-call]

    def _on_odometry(self, msg: Odometry) -> None:
        try:
            pose = self.tfbuffer.get(
                self.config.world_frame,
                self.config.camera_frame,
                time_point=msg.ts,
                time_tolerance=1.0,
            )
        except Exception:
            logger.exception("ground frustum tf lookup failed")
            return
        if pose is None:
            return
        self.ground_frustum.publish(
            PointStamped(
                0.0, 0.0, self._distance(pose), ts=msg.ts, frame_id=self.config.camera_frame
            )
        )

    def _distance(self, pose: Transform) -> float:
        return ground_range(pose, self.config)


def ground_range(pose: Transform, config: Config) -> float:
    """Optical-axis range from the camera to the plane it stops at.

    The axis is the optical frame's +Z in world; its downward component is
    the cosine of the tilt off nadir, which is both the nadir test and the
    secant that turns height into slant range. With a ``ground_point`` the
    range is instead that point's distance along the axis — the plane
    through it, since rerun draws the image square to the axis regardless.
    """
    matrix = pose.to_matrix()
    axis, camera = matrix[:3, 2], matrix[:3, 3]
    down = -float(axis[2])
    if down < math.cos(math.radians(config.nadir_tolerance_deg)):
        return config.near_distance_m
    if config.ground_point is not None:
        distance = float(np.dot(np.asarray(config.ground_point) - camera, axis))
    else:
        distance = (float(camera[2]) - config.ground_z) / down
    if distance <= 0.0:
        return config.near_distance_m
    return min(distance, config.max_distance_m)


def render_image_plane(
    msg: PointStamped,
    camera_info: CameraInfo,
    image_topic: str = "world/video",
) -> RerunData:
    """The camera's pinhole, its image plane out at the point's range."""
    return camera_info.to_rerun(image_plane_distance=msg.z, image_topic=image_topic)


def ground_frustum_override(
    camera_info: CameraInfo,
    image_topic: str = "world/video",
    entity: str = "world/ground_frustum",
) -> dict[str, Callable[[Any], RerunData]]:
    return {entity: partial(render_image_plane, camera_info=camera_info, image_topic=image_topic)}
