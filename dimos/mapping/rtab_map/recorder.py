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

"""Record what RTAB-Map is fed, and what it produced, into a memory2 db.

The recording is everything the module consumes -- ``color_image``,
``depth_image``, ``camera_info`` -- so a session can be replayed into RTAB-Map
offline at whatever speed the question needs. Each observation carries both the
sensor's own timestamp and the ``reception_ts`` it arrived at, which is what
separates a slow RTAB-Map from a camera that was already behind.

``rtabmap_odometry`` is wired to RtabmapSlam's ``odometry`` output with
``.remappings()``, and doubles as the pose source for the image streams, so the
frames carry the trajectory rather than needing a tf lookup that this rig has no
``world`` frame for.

``cloud_map`` is the assembled map, so each message is the *whole* map rather
than the frame's contribution to it: RTAB-Map re-assembles it from the optimized
graph every ``cloud_publish_period_s`` (1 s by default) up to ``cloud_max_points``
(2M, ~24 MB). Recording it therefore stores the map once per publish, and the
period is the knob for how much of that lands on disk.

Depth and the cloud are stored losslessly (JPEG is the default for an ``Image``,
and its 16-bit millimetres are a measurement, not a picture). They are the bulk
of the write -- an 848x480 depth frame at 15 Hz alone is a few MB/s, so
recordings are large and the disk has to keep up.
"""

from __future__ import annotations

from dimos.core.stream import In
from dimos.memory2.module import OnExisting, Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class RtabmapRecorderConfig(RecorderConfig):
    # Append into a populated db (keep other streams); replace only our own.
    on_existing: OnExisting = OnExisting.APPEND
    # The intrinsics describe the rig, not a place.
    poseless_streams: list[str] = ["camera_info"]


class RtabmapRecorder(Recorder):
    config: RtabmapRecorderConfig

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    rtabmap_odometry: In[Odometry]
    cloud_map: In[PointCloud2]

    _last_odom_pose: Pose | None = None

    def _prepare_streams(self) -> None:
        super()._prepare_streams()
        depth = self.config.stream_remapping.get("depth_image", "depth_image")
        self.store.stream(depth, Image, codec="lz4+lcm")
        cloud = self.config.stream_remapping.get("cloud_map", "cloud_map")
        self.store.stream(cloud, PointCloud2, codec="lz4+lcm")

    @pose_setter_for("rtabmap_odometry")
    async def _odom_pose(self, msg: Odometry) -> Pose | None:
        pose = getattr(msg, "pose", None)
        self._last_odom_pose = getattr(pose, "pose", None) if pose is not None else None
        return self._last_odom_pose

    @pose_setter_for("color_image", "depth_image")
    async def _frame_pose(self, msg: Image) -> Pose | None:
        return self._last_odom_pose

    @pose_setter_for("cloud_map")
    async def _cloud_pose(self, msg: PointCloud2) -> Pose | None:
        """Identity: the assembled map already sits in map-frame coordinates, so
        anchoring it to the robot would apply the trajectory to it a second time."""
        return Transform.identity().to_pose()
