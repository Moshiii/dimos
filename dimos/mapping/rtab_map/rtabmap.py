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

"""Native C++ RTAB-Map RGB-D SLAM module."""

from __future__ import annotations

from pathlib import Path as FilePath
import time
from typing import Literal

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.patterns import Glob

MODULE_DIR = FilePath(__file__).resolve().parent

# Path.to_rerun() lifts the line 0.5 m by default, which suits a ground robot whose path
# would otherwise z-fight with the floor costmap. These paths are the camera's own
# trajectory and have to sit where the camera actually went, or they float above the
# cloud_map they are meant to line up with. Pass as ``vis_module(..., rerun_config=...)``.
#
# Glob rather than a plain string: the bridge matches a str pattern by exact equality
# against the whole entity path, so a remapped or namespaced topic would miss silently
# and the only symptom would be a path floating half a metre off.
RERUN_CONFIG: dict[str, object] = {
    "visual_override": {
        Glob("**odom_path"): lambda path: path.to_rerun(z_offset=0.0, color=(0, 255, 128)),
        Glob("**map_path"): lambda path: path.to_rerun(z_offset=0.0, color=(255, 160, 0)),
    }
}


class RtabmapConfig(NativeModuleConfig):
    cwd: str | None = str(MODULE_DIR)
    executable: str = "result/bin/rtabmap_slam"
    # Runs with cwd=MODULE_DIR (NativeModule resolves a relative cwd against the
    # module's own file), and nix resolves "." inside a git repo to the repo root
    # with ?dir= filled in -- which is what lets the flake reach ../../../native/cpp.
    build_command: str | None = "nix build ."
    stdin_config: bool = True

    # "rgbd"      colour + aligned depth. Metric odometry and a dense map.
    # "stereo_ir" the infrared pair, rtabmap computing disparity itself. The
    #             global-shutter IR imagers do not smear under motion and ignore the
    #             colour sensor's auto-exposure, at the cost of that disparity work.
    # "mono"      colour only. One camera cannot resolve scale, so this is rtabmap's
    #             original appearance-only loop closure detection: no pose, no cloud,
    #             just place recognition. Mainly a throughput baseline.
    input_mode: Literal["rgbd", "stereo_ir", "mono"] = "rgbd"
    # Off runs the visual odometry alone -- no pose graph, no loop closure, no map and
    # no map->odom. The cheapest configuration, and the one to use when something else
    # owns the map. Ignored in mono mode, which *is* the loop closure detector.
    enable_loop_closure: bool = True
    # Report mean/worst processing time this often, so a run states the rate it could
    # sustain rather than the rate it happened to be fed. 0 disables.
    timing_report_period_s: float = 10.0
    # Distance between the infrared imagers. Only read in stereo_ir mode. This is
    # the D435/D435i factory value; a D455 is 0.0949.
    baseline_m: float = 0.0499

    database_path: str = str(FilePath.home() / ".cache/dimos/rtabmap.db")
    # A stale database silently turns a mapping run into a relocalization run
    # against yesterday's map, so a fresh session is the safer default.
    delete_db_on_start: bool = True

    # 0 = frame-to-map. It keeps a local feature map instead of only the previous
    # frame, which is what stops slow drift on a robot that pauses and turns.
    odom_strategy: int = 0
    # Consecutive failed frames before odometry restarts itself from the last good
    # pose. Without this a single bad patch of wall parks odometry forever.
    odom_reset_countdown: int = 10

    # How often a frame is allowed to become a graph node and trigger a loop closure
    # search. Odometry still runs on every frame. RTAB-Map's own default rate.
    #
    # Enforced by this module rather than handed to RTAB-Map: Rtabmap/DetectionRate is
    # read by RtabmapThread, and this drives Rtabmap directly, so setting the parameter
    # would be accepted and then quietly ignored.
    detection_rate_hz: float = 1.0
    # Confidence a hypothesis needs before it counts as a loop closure. Lower finds
    # more closures and accepts more false ones -- a false closure warps the map.
    loop_closure_threshold: float = 0.11
    # RTAB-Map's real-time mechanism: when a map update exceeds this, nodes are moved
    # out of working memory until it fits, trading map size for a rate guarantee.
    # 0 = no limit, which is right when mapping a bounded space.
    time_budget_ms: float = 0.0
    # Hard cap on working-memory nodes. 0 = unlimited.
    memory_threshold_nodes: int = 0

    # How far the robot must move before a frame may become a node. This -- not
    # detection_rate_hz -- is why a stationary camera never grows the map. 0 disables
    # the gate, which is how you make a bench test produce a map without moving.
    linear_update_m: float = 0.1
    angular_update_rad: float = 0.1

    # Feature budgets. Both are first-order speed knobs: max_features feeds the loop
    # closure dictionary, vis_max_features the frame-to-frame registration.
    max_features: int = 500
    vis_max_features: int = 1000
    # Correspondences needed to accept a transform. Raise it if odometry is jumping.
    min_inliers: int = 20
    # Kp/DetectorStrategy. 8=GFTT/ORB (default), 2=ORB, 1=SIFT.
    feature_type: int = 8
    # 0=TORO, 1=g2o, 2=GTSAM, 3=Ceres. This build links g2o and TORO; GTSAM and Ceres
    # are not compiled in, so asking for them falls back with a warning from rtabmap.
    optimizer_strategy: int = 1

    # A D4xx's stereo depth degrades with range; past ~4 m it contributes noise to
    # the map faster than it contributes structure.
    max_depth_m: float = 4.0
    min_depth_m: float = 0.3

    publish_cloud_map: bool = True
    cloud_decimation: int = 4
    cloud_voxel_size_m: float = 0.05
    cloud_publish_period_s: float = 1.0
    cloud_max_points: int = 2_000_000

    map_frame: str = "map"
    odom_frame: str = "odom"
    base_frame: str = "base_link"
    # Only meaningful before the graph has moved: until then map->odom is identity.
    # Turn it off whenever something else in the graph owns that edge -- two
    # publishers of one tf edge fight.
    publish_map_to_odom: bool = True

    # Colour and depth arrive as separate messages; this is how far apart their
    # stamps may be and still be treated as one instant.
    max_pair_skew_s: float = 0.02

    # Trajectory rendering. Poses are appended by distance rather than per frame,
    # so a stationary robot does not spend the whole budget standing still.
    path_max_poses: int = 5000
    path_publish_period_s: float = 0.5
    path_min_step_m: float = 0.02

    # Raw RTAB-Map parameters, applied last. The escape hatch for the ~600
    # parameters that are not worth a config field each.
    extra_parameters: dict[str, str] = Field(default_factory=dict)


class RtabmapSlam(NativeModule):
    """RGB-D SLAM: visual odometry plus appearance-based loop closure.

    Two pose streams, because two different things produce them. ``odometry`` is
    RTAB-Map's visual odometry, ``odom`` -> ``base_link``: it drifts but never jumps,
    which is what a controller needs. ``corrected_odometry`` is the pose graph's
    answer, ``map`` -> ``base_link``: it jumps at a loop closure, which is the whole
    point -- that is where a revisit gets pulled back onto itself.

    ``tf`` carries ``odom`` -> ``base_link`` from the odometry and ``map`` -> ``odom``
    from the correction, so the jump lands on the edge that is allowed to jump.

    ``cloud_map`` is re-assembled from the *optimized* graph each time it is
    published rather than accumulated frame by frame, so a loop closure visibly
    pulls the map back together instead of leaving two copies of the same wall.

    ``odom_path`` and ``map_path`` are the two trajectories as ``nav_msgs/Path``,
    which is what renders as a line in Rerun; the ``Odometry`` streams render only
    as the current pose.

    The pose is the *colour camera's*, since RTAB-Map is given the camera's optical
    frame as the body origin. On a robot whose camera is not at the body origin,
    either set ``base_frame`` to the camera's own frame and let the static tf tree
    carry it to the body, or fold the mount extrinsic in.
    """

    config: RtabmapConfig

    camera_info: In[CameraInfo]
    # rgbd mode reads this pair...
    color_image: In[Image]
    depth_image: In[Image]
    # ...and stereo_ir mode reads this one. Only the active pair is subscribed by
    # the native process, so the other may be left unwired.
    image_left: In[Image]
    image_right: In[Image]

    odometry: Out[Odometry]
    corrected_odometry: Out[Odometry]
    map_tf: Out[Odometry]
    cloud_map: Out[PointCloud2]
    tf: Out[TFMessage]
    odom_path: Out[Path]
    map_path: Out[Path]

    @rpc
    def start(self) -> None:
        super().start()
        self._odom_poses: list[PoseStamped] = []
        self._map_poses: list[PoseStamped] = []
        self._odom_last_publish = 0.0
        self._map_last_publish = 0.0
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odometry, self.odometry))
        )
        self.register_disposable(
            Disposable(self.map_tf.transport.subscribe(self._on_map_tf, self.map_tf))
        )
        self.register_disposable(
            Disposable(
                self.corrected_odometry.transport.subscribe(
                    self._on_corrected_odometry, self.corrected_odometry
                )
            )
        )

    @staticmethod
    def _transform(message: Odometry, frame_id: str, child_frame_id: str) -> Transform:
        return Transform(
            frame_id=frame_id,
            child_frame_id=child_frame_id,
            translation=Vector3(
                message.pose.position.x, message.pose.position.y, message.pose.position.z
            ),
            rotation=Quaternion(
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ),
            ts=message.ts or time.time(),
        )

    def _on_odometry(self, message: Odometry) -> None:
        self.tf.publish(
            TFMessage(self._transform(message, self.config.odom_frame, self.config.base_frame))
        )
        if self._append(self._odom_poses, message, self.config.odom_frame):
            self._odom_last_publish = self._maybe_publish(
                self.odom_path, self._odom_poses, self.config.odom_frame, self._odom_last_publish
            )

    def _on_map_tf(self, message: Odometry) -> None:
        """The pose graph's correction, map->odom. Identity until the graph moves."""
        if not self.config.publish_map_to_odom:
            return
        self.tf.publish(
            TFMessage(self._transform(message, self.config.map_frame, self.config.odom_frame))
        )

    def _on_corrected_odometry(self, message: Odometry) -> None:
        if self._append(self._map_poses, message, self.config.map_frame):
            self._map_last_publish = self._maybe_publish(
                self.map_path, self._map_poses, self.config.map_frame, self._map_last_publish
            )

    def _append(self, poses: list[PoseStamped], message: Odometry, frame_id: str) -> bool:
        """Append only once the pose has actually moved, and cap the history."""
        position = message.pose.position
        if poses:
            previous = poses[-1].position
            step = (
                (position.x - previous.x) ** 2
                + (position.y - previous.y) ** 2
                + (position.z - previous.z) ** 2
            ) ** 0.5
            if step < self.config.path_min_step_m:
                return False
        poses.append(
            PoseStamped(
                ts=message.ts or time.time(),
                frame_id=frame_id,
                position=position,
                orientation=message.pose.orientation,
            )
        )
        if len(poses) > self.config.path_max_poses:
            del poses[0 : len(poses) - self.config.path_max_poses]
        return True

    def _maybe_publish(
        self, stream: Out[Path], poses: list[PoseStamped], frame_id: str, last_publish: float
    ) -> float:
        now = time.time()
        if now - last_publish < self.config.path_publish_period_s:
            return last_publish
        stream.publish(Path(ts=now, frame_id=frame_id, poses=list(poses)))
        return now
