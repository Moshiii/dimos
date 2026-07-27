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

"""``world -> enu`` for platforms whose world frame is already north-aligned.

A compass-equipped autopilot (DJI) boots its world frame ENU-aligned, so
georegistration reduces to a translation: where the shared ENU origin sits
in the world frame. This module measures it as the median offset between the
robot's tf pose and the GPS fix expressed in ENU, and publishes the
transform on tf — which is all
:class:`~dimos.visualization.citymesh.module.CityMeshModule` needs to land
in the world view.

The robot's position comes from the tf tree (``parent -> robot_frame``), not
from an Odometry topic: every dimos robot has tf, not all have odometry
messages. It is one of a pluggable pair of registration strategies: the
other, still to be built on :mod:`.georegister`, recovers the yaw from the
track shape for compass-free platforms (Go2). Those must NOT use this one —
their world frame has arbitrary yaw, and this module assumes yaw zero.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
import math
from typing import TYPE_CHECKING

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.visualization.citymesh.frame import EnuFrame

logger = setup_logger()


class EnuSnap:
    """The estimator: robot positions + fixes in, a ``world -> enu`` transform out.

    The frame is anchored exactly as CityMeshModule anchors its own —
    ``snap_origin`` of the first fix, sea level — so the two agree on what
    ``enu`` means without talking to each other.
    """

    def __init__(
        self,
        frame_id: str = "enu",
        parent: str = "world",
        window: int = 30,
        min_samples: int = 3,
    ) -> None:
        self.frame_id = frame_id
        self.parent = parent
        self.min_samples = min_samples
        self.frame: EnuFrame | None = None
        self._offsets: deque[tuple[float, float, float]] = deque(maxlen=window)

    def add_fix(self, msg: NavSatFix, position: Sequence[float]) -> Transform | None:
        """Fold in one fix paired with the robot's position in the parent frame.

        Returns a transform once enough samples agree, else None. The median
        over the window is what makes a single wild fix harmless.
        """
        if not msg.has_fix or not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return None
        if self.frame is None:
            from dimos.visualization.citymesh.frame import EnuFrame, snap_origin

            lat0, lon0 = snap_origin(msg.latitude, msg.longitude)
            self.frame = EnuFrame.at(lat0, lon0, 0.0, datum="msl", undulation=0.0)
            logger.info("enu frame anchored", lat=lat0, lon=lon0, frame=self.frame_id)

        alt = self.frame.origin_msl if math.isnan(msg.altitude) else msg.altitude
        e, n, u = self.frame.geodetic_to_enu(msg.latitude, msg.longitude, alt, datum="msl")[0]
        x, y, z = (float(c) for c in position)
        self._offsets.append((x - float(e), y - float(n), z - float(u)))
        if len(self._offsets) < self.min_samples:
            return None

        cols = list(zip(*self._offsets, strict=True))
        t = [sorted(c)[len(c) // 2] for c in cols]
        return Transform(
            translation=Vector3(*t),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id=self.parent,
            child_frame_id=self.frame_id,
        )


class Config(ModuleConfig):
    frame: str = "enu"
    parent: str = "world"
    robot_frame: str = "base_link"
    window: int = 30
    min_samples: int = 3


class EnuSnapTF(Module):
    """Publishes the estimator's ``world -> enu`` on tf, once per fix."""

    config: Config
    gps: In[NavSatFix]

    @rpc
    def start(self) -> None:
        super().start()
        self._snap = EnuSnap(
            frame_id=self.config.frame,
            parent=self.config.parent,
            window=self.config.window,
            min_samples=self.config.min_samples,
        )
        self.register_disposable(self.gps.observable().subscribe(self._on_fix))  # type: ignore[no-untyped-call]

    def _on_fix(self, msg: NavSatFix) -> None:
        try:
            pose = self.tf.get(self.config.parent, self.config.robot_frame, time_tolerance=1.0)
            if pose is None:
                return
            transform = self._snap.add_fix(msg, pose.translation)
        except Exception:
            logger.exception("enu snap failed")
            return
        if transform is not None:
            self.tf.publish(transform.now())
