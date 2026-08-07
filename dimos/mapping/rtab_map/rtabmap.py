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
from typing import TYPE_CHECKING, Any, Literal

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
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

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype

MODULE_DIR = FilePath(__file__).resolve().parent


# The trail comes from OdometryPath, whose output stream is ``path``.
#
# Path.to_rerun() lifts the line 0.5 m by default, which suits a ground robot whose path
# would otherwise z-fight with a floor costmap. This trail is the camera's own
# trajectory and has to sit where the camera actually went, or it floats above the
# cloud_map it is meant to line up with. Pass as ``vis_module(..., rerun_config=...)``.
#
# Glob rather than a plain string: the bridge matches a str pattern by exact equality
# against the whole entity path, so a remapped or namespaced topic would miss silently
# and the only symptom would be a trail floating half a metre off.
#
# A named function rather than a lambda because the config is pickled to the worker
# that hosts the bridge, and a lambda has no importable name to pickle by.
def _path_trail(path: Path) -> Archetype:
    return path.to_rerun(z_offset=0.0, color=(0, 255, 128))


RERUN_CONFIG: dict[str, object] = {"visual_override": {Glob("**path"): _path_trail}}


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
    # Take the pose from the ``external_odometry`` stream instead of running RTAB-Map's
    # own visual odometry. The split rtabmap_ros uses: somebody else owns
    # odom->base_link, and RTAB-Map contributes only the graph and the map->odom
    # correction. Any Odometry publisher works -- a ZED's SDK pose, FastLIO2, cuVSLAM,
    # wheel odometry.
    #
    # It is also the only way to get a rolling-shutter-compensated pose, because
    # RTAB-Map's own odometry cannot compensate: it has one timestamp per frame and no
    # per-row model. A ZED's SDK has both, so it can hand over a corrected pose that
    # this module then maps around.
    use_external_odometry: bool = False
    # How far an external pose may sit from a frame's stamp and still be used for it.
    # Both must share a clock. Frames with nothing close enough are dropped rather than
    # mapped against a stale pose.
    external_odometry_timeout_s: float = 0.1
    # Report mean/worst processing time this often, so a run states the rate it could
    # sustain rather than the rate it happened to be fed. 0 disables.
    timing_report_period_s: float = 10.0
    # Distance between the two imagers of the stereo pair, metres. Only read in
    # stereo_ir mode -- in rgbd mode the camera has already triangulated. It is
    # per-model and per-unit (a D435 is ~50 mm, a D455 ~95 mm), so there is no default
    # that is right on the next camera: ask the rig for it, with
    # ``RealSenseCamera.between_cam_distance()`` for a RealSense. Unset, stereo_ir
    # refuses to start rather than map at the wrong scale.
    between_cam_distance: float | None = None

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
    # One tf frame per camera: the optical frame that camera's images and camera_info
    # are stamped in, ordered to match the ports (camera 1 is the unsuffixed
    # color_image/depth_image/camera_info, camera 2 is ``*_2``, up to 4).
    #
    # Only the names live here. ``base_frame`` -> each of them is read off the ``tf``
    # stream, and that transform is exactly what RTAB-Map wants as the camera's
    # localTransform -- so three cameras facing three directions are described by
    # whatever already publishes their frames (a URDF, a StaticTfPublisher), and the
    # geometry is not written down twice. Mapping waits, saying so, until tf can place
    # every camera.
    #
    # Empty is the single camera at the body origin. Naming several is rgbd only:
    # stereo_ir and mono are one rig by construction.
    camera_frames: list[str] = Field(default_factory=list)
    # Only meaningful before the graph has moved: until then map->odom is identity.
    # Turn it off whenever something else in the graph owns that edge -- two
    # publishers of one tf edge fight.
    publish_map_to_odom: bool = True

    # Colour and depth arrive as separate messages; this is how far apart their
    # stamps may be and still be treated as one instant.
    max_pair_skew_s: float = 0.02

    # Raw RTAB-Map parameters, applied last. The escape hatch for the ~600
    # parameters that are not worth a config field each.
    extra_parameters: dict[str, str] = Field(default_factory=dict)

    def to_config_dict(self) -> dict[str, Any]:
        # The base drops None and the native half requires every field to be present,
        # so unset travels as the 0 it rejects, with its own error message.
        return {
            **super().to_config_dict(),
            "between_cam_distance": self.between_cam_distance or 0.0,
        }


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

    Neither pose stream draws a trail on its own -- a viewer renders ``Odometry`` as
    the current pose and nothing more. Pair this with ``OdometryPath`` to accumulate
    ``odometry`` into a ``nav_msgs/Path``, which is what renders as a line. Feed it the
    continuous ``odometry`` rather than ``corrected_odometry`` and let the ``map`` ->
    ``odom`` edge carry the correction, or the trail jumps at every loop closure.

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
    # Cameras 2..4, subscribed only as far as ``camera_frames`` names them. Ports have
    # to be declared statically, which is what fixes the ceiling at four.
    camera_info_2: In[CameraInfo]
    color_image_2: In[Image]
    depth_image_2: In[Image]
    camera_info_3: In[CameraInfo]
    color_image_3: In[Image]
    depth_image_3: In[Image]
    camera_info_4: In[CameraInfo]
    color_image_4: In[Image]
    depth_image_4: In[Image]
    # ...and stereo_ir mode reads this one. Only the active pair is subscribed by
    # the native process, so the other may be left unwired.
    image_left: In[Image]
    image_right: In[Image]
    # Only subscribed when use_external_odometry is set; otherwise RTAB-Map computes
    # its own and this may be left unwired.
    external_odometry: In[Odometry]

    odometry: Out[Odometry]
    corrected_odometry: Out[Odometry]
    map_tf: Out[Odometry]
    cloud_map: Out[PointCloud2]
    # Written from here (odom->base_link, map->odom) and read by the native half,
    # which places each camera of a multi-camera rig off the tree.
    tf: IO[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odometry, self.odometry))
        )
        self.register_disposable(
            Disposable(self.map_tf.transport.subscribe(self._on_map_tf, self.map_tf))
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
        # With external odometry the native half publishes nothing here, and its
        # provider owns odom->base_link. Two publishers of one tf edge fight.
        if self.config.use_external_odometry:
            return
        self.tf.publish(
            TFMessage(self._transform(message, self.config.odom_frame, self.config.base_frame))
        )

    def _on_map_tf(self, message: Odometry) -> None:
        """The pose graph's correction, map->odom. Identity until the graph moves."""
        if not self.config.publish_map_to_odom:
            return
        self.tf.publish(
            TFMessage(self._transform(message, self.config.map_frame, self.config.odom_frame))
        )
