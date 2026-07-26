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

Everything GO2Zenoh puts on the graph except ``pointlio_map`` — the accumulated
map is Point-LIO's product and rebuildable from ``lidar`` + ``odometry``, so
recording each (growing) snapshot would only bloat the db. The tf tree is
recorded by the ``Recorder`` base, so the mount frames and the live
``odom -> mid360_link`` edge come along for free.

Poses anchor in ``odom`` (there is no ``world`` on this rig): odometry carries
its own pose, everything else resolves through tf via its ``frame_id``.
"""

from __future__ import annotations

from pathlib import Path

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
    video: In[CompressedVideo]  # front camera, H.264 annex-B
    gps: In[NavSatFix]

    @pose_setter_for("odometry")
    async def _odom_pose(self, msg: Odometry) -> Pose | None:
        return msg.pose.pose
