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

"""Record the go2web zenoh bridge's streams into a memory2 SQLite db.

Everything GO2Zenoh puts on the graph. The tf tree is recorded by the
``Recorder`` base (``record_tf``), so the mount frames and the live
``odom -> mid360_link`` edge come along for free — do NOT declare a ``tf``
In port for it: ``Module.tf`` is a property, the annotation silently loses
to it and the coordinator dies with "Output tf is not a valid stream".

Poses anchor in ``odom`` (there is no ``world`` on this rig) and come from
the odometry stream, as in ``PointlioRecorder``: every sensor observation is
stamped with the latest odometry pose so the recording carries the trajectory.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class GO2ZenohRecorderConfig(RecorderConfig):
    db_path: str | Path = "recording_go2_zenoh.db"
    root_frame: str = "odom"


class GO2ZenohRecorder(Recorder):
    config: GO2ZenohRecorderConfig

    odometry: In[Odometry]
    lidar: In[PointCloud2]  # Mid-360 per-scan cloud, frame mid360_link
    pointlio_map: In[PointCloud2]  # Point-LIO global keyframe cloud, frame odom
    video: In[CompressedVideo]  # front camera, H.264 annex-B
    gps: In[NavSatFix]

    _last_odom_pose: Pose | None = None

    @pose_setter_for("odometry")
    async def _odom_pose(self, msg: Odometry) -> Pose | None:
        self._last_odom_pose = msg.pose.pose
        return self._last_odom_pose

    @pose_setter_for("lidar", "pointlio_map", "video", "gps")
    async def _sensor_pose(self, msg: Any) -> Pose | None:
        return self._last_odom_pose
