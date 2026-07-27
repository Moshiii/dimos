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

A compass-equipped autopilot (DJI) fuses GPS into its odometry, so its world
frame is ENU-aligned and GPS-anchored by construction. The fix and the pose
then come from the same estimate — their difference is not noise but the
EKF's origin. So georegistration needs no history: every (fix, pose) pair
pins the transform outright, and publishing it per fix simply tracks the
autopilot's current best estimate. That is all
:class:`~dimos.visualization.citymesh.module.CityMeshModule` needs to land
in the world view.

The robot's position comes from the tf tree (``parent -> robot_frame``), not
from an Odometry topic: every dimos robot has tf, not all have odometry
messages. This is one of a pluggable pair of registration strategies: the
other, still to be built on :mod:`.georegister`, recovers the yaw from the
track shape for compass-free platforms (Go2). Those must NOT use this one —
their world frame has arbitrary yaw, and this module assumes yaw zero.
"""

from __future__ import annotations

from collections.abc import Sequence
import math

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def snap_transform(
    fix: NavSatFix,
    position: Sequence[float],
    frame_id: str = "enu",
    parent: str = "world",
) -> Transform | None:
    """The ``parent -> enu`` transform pinned by one (fix, pose) pair.

    The ENU frame is anchored exactly as CityMeshModule anchors its own —
    ``snap_origin`` of the fix, sea level — so the two agree on what ``enu``
    means without talking to each other. Identity rotation: the parent frame
    is assumed north-aligned.
    """
    if not fix.has_fix or not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
        return None
    from dimos.visualization.citymesh.frame import EnuFrame, snap_origin

    lat0, lon0 = snap_origin(fix.latitude, fix.longitude)
    frame = EnuFrame.at(lat0, lon0, 0.0, datum="msl", undulation=0.0)
    alt = frame.origin_msl if math.isnan(fix.altitude) else fix.altitude
    e, n, u = frame.geodetic_to_enu(fix.latitude, fix.longitude, alt, datum="msl")[0]
    x, y, z = (float(c) for c in position)
    return Transform(
        translation=Vector3(x - float(e), y - float(n), z - float(u)),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id=parent,
        child_frame_id=frame_id,
    )


class Config(ModuleConfig):
    frame: str = "enu"
    parent: str = "world"
    robot_frame: str = "base_link"


class EnuSnapTF(Module):
    """Publishes ``world -> enu`` on every fix, from that fix and the tf pose.

    Recomputed each time rather than measured once: the pair always encodes
    the autopilot's current estimate, so the transform improves whenever the
    EKF's does. The pose is looked up at the fix's own timestamp, so motion
    between the two samples doesn't smear into the transform.
    """

    config: Config
    gps: In[NavSatFix]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(self.gps.observable().subscribe(self._on_fix))  # type: ignore[no-untyped-call]

    def _on_fix(self, msg: NavSatFix) -> None:
        try:
            pose = self.tf.get(
                self.config.parent,
                self.config.robot_frame,
                time_point=msg.ts,
                time_tolerance=1.0,
            )
            if pose is None:
                return
            transform = snap_transform(
                msg, pose.translation, frame_id=self.config.frame, parent=self.config.parent
            )
        except Exception:
            logger.exception("enu snap failed")
            return
        if transform is not None:
            self.tf.publish(transform.now())
