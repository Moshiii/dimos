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

"""Python NativeModule wrapper for the FAST-LIVO2 binary.

Unlike the pointlio/fastlio2 modules (which bind the Livox SDK directly),
FastLivo consumes sensor *streams*: the livox module's lidar (PointCloud2 with
per-point ``offset_time``) and imu topics, plus any camera's color_image +
camera_info topics. Camera intrinsics are read from the camera_info stream at
runtime — nothing is hardcoded to a specific camera. Output is odometry (also
broadcast on TF).

Usage::

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
    from dimos.hardware.sensors.lidar.fastlivo.module import FastLivo
    from dimos.hardware.sensors.lidar.livox.module import Mid360

    ModuleCoordinator.build(autoconnect(
        Mid360.blueprint(),
        RealSenseCamera.blueprint(),
        FastLivo.blueprint(),
    )).loop()

FAST-LIVO2 tuning lives directly on ``FastLivoConfig`` and is passed to the
C++ binary as plain CLI args (no YAML). Extrinsic defaults are for the
Mid-360 + RealSense D435i rig (``dimos/robot/assembly/mid360_realsense_30.py``);
derive values for other rigs with ``scripts/compute_extrinsics.py``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.cmu_nav.frames import FRAME_ODOM
from dimos.spec import perception


class FastLivoConfig(NativeModuleConfig):
    cwd: str | None = "cpp"
    executable: str = "result/bin/fastlivo_native"
    build_command: str | None = "nix build .#fastlivo_native"

    # Odometry is published as frame_id (fixed) -> sensor_frame_id (moving
    # sensor), and also broadcast on TF.
    frame_id: str = FRAME_ODOM
    sensor_frame_id: str = "mid360_link"

    debug: bool = False

    # --- FAST-LIVO2 tuning, passed to the binary as plain CLI args ---
    # common
    img_en: bool = True  # false = pure LIO (camera ports may then be left unwired)
    lidar_en: bool = True
    # vio
    normal_en: bool = True
    inverse_composition_en: bool = False
    max_iterations: int = 5
    img_point_cov: float = 100.0
    raycast_en: bool = False
    exposure_estimate_en: bool = True
    inv_expo_cov: float = 0.1
    grid_size: int = 5
    grid_n_height: int = 17
    patch_pyrimid_level: int = 3  # upstream's spelling
    patch_size: int = 8
    outlier_threshold: float = 1000.0
    # time offsets (s) applied to incoming stamps
    exposure_time_init: float = 0.0
    img_time_offset: float = 0.0
    imu_time_offset: float = 0.0
    lidar_time_offset: float = 0.0
    # imu — defaults are the robust vibration-tolerant setting (acc_cov 2.0 /
    # gyr_cov 1.0 tied Point-LIO on huge_loop_go2). On stable platforms
    # (handheld/wheeled) lower covs score better (0.5/0.3 gave 6.16m vs 7.68m
    # on huge_loop_realsense). On legged robots also consider img_en=False:
    # rolling-shutter cameras + vibration poison the visual updates long
    # before the LIO degrades.
    imu_en: bool = True
    gyr_cov: float = 1.0
    acc_cov: float = 2.0
    imu_int_frame: int = 3
    gravity_est_en: bool = True
    ba_bg_est_en: bool = True
    # preprocess (lidar_type 1 = AVIA/Livox branch — the Mid-360 CustomMsg path)
    lidar_type: int = 1
    scan_line: int = 4
    blind: float = 0.5  # spherical min range (m)
    point_filter_num: int = 3
    filter_size_surf: float = 0.2
    # lio / voxel map
    max_layer: int = 2
    voxel_size: float = 1.0
    min_eigen_value: float = 0.005
    sigma_num: float = 3.0
    beam_err: float = 0.02
    dept_err: float = 0.05
    layer_init_num: list[int] = Field(default_factory=lambda: [5, 5, 5, 5, 5])
    max_points_num: int = 50
    lio_max_iterations: int = 5
    # Local map sliding: bounds voxel-map memory (the map is otherwise
    # unbounded — a 60-min outdoor run grows past 25G RSS). FAST-LIVO2 has no
    # loop closure, so discarding far-away map costs nothing but map reuse on
    # revisit.
    map_sliding_en: bool = True
    half_map_size: int = 50
    sliding_thresh: float = 8.0
    # Same idea for the VIO visual map (not covered by map sliding): prune
    # visual voxels farther than this from the current pose (m); 0 disables.
    visual_map_radius: float = 60.0
    # Process every Nth camera frame (1 = all). sync_packages slices lidar
    # scans at image times, so a high camera rate fragments the lidar
    # geometry per LIO update; striding rebalances that.
    img_stride: int = 1
    # Camera-quality gate (gsc_pgo-style calibrated gates; 0 disables each).
    # Drop frames whose capture-window peak |gyro| exceeds this (rad/s):
    # rotation rate drives motion-blur smear AND rolling-shutter jello, the
    # two failure modes that poisoned the go2 camera (373m vs 13.3m).
    # Calibrated: realsense p95 = 0.86 (default barely triggers), go2 p95 =
    # 2.42 (drops its worst ~30%).
    img_max_gyro: float = 1.2
    # Drop frames whose variance-of-Laplacian sharpness falls below this
    # fraction of the rolling median — catches gross defocus/black frames.
    # Default OFF: rolling-shutter jello INFLATES Laplacian variance
    # (measured corr(sharpness, |gyro|) = +0.54 on go2), so it cannot detect
    # vibration blur.
    img_min_sharpness_ratio: float = 0.0
    # Content-based gate (works without IMU coverage of the camera): flag a
    # frame when the stddev of per-strip horizontal shifts vs the previous
    # frame exceeds this many pixels. A global-shutter view change moves all
    # strips equally; rolling-shutter jello moves top/bottom differently, so
    # this measures RS damage directly. Calibrated: go2 0.7px rest vs 13px
    # shaking; realsense p95 = 4.1px. ~0.2ms/frame. 0 disables.
    img_max_row_shift_std: float = 6.0
    # Gate mode: "soft" (default) feeds every frame but divides its visual
    # influence by img_bad_frame_penalty when flagged (LIVO-mode sync needs
    # images to advance — hard-dropping diverged on go2 because the
    # starvation guard eventually admits damaged frames at full weight).
    # "drop" = hard-drop flagged frames (experiments only).
    img_gate_mode: str = "soft"
    img_bad_frame_penalty: float = 10000.0
    # Starvation guard (drop mode only): always accept a frame after this
    # many seconds without one.
    img_accept_max_gap: float = 2.0
    # Debug/eval: accumulate the registered world cloud (voxel-downsampled)
    # and write a binary PLY here at shutdown. Empty = off.
    map_out: str = ""
    map_voxel: float = 0.25
    # extrinsics: lidar in IMU frame (Mid-360 built-in IMU)
    extrinsic_t: list[float] = Field(default_factory=lambda: [-0.011, -0.02329, 0.04412])
    extrinsic_r: list[float] = Field(
        default_factory=lambda: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    # extrinsics: lidar in camera optical frame (p_cam = Rcl @ p_lidar + Pcl).
    # Defaults from the Mid-360 + RealSense D435i rig (compute_extrinsics.py).
    pcl: list[float] = Field(default_factory=lambda: [0.0325, -0.081149, -0.047626])
    rcl: list[float] = Field(
        default_factory=lambda: [
            0.0,
            -1.0,
            0.0,
            0.587785,
            0.0,
            -0.809017,
            0.809017,
            0.0,
            0.587785,
        ]
    )
    # publish
    blind_rgb_points: float = 0.01
    pub_scan_num: int = 1
    pub_effect_point_en: bool = False
    dense_map_en: bool = False


class FastLivo(NativeModule, perception.Odometry):
    """FAST-LIVO2 LiDAR-inertial-visual odometry over the livox module's streams."""

    config: FastLivoConfig

    lidar: In[PointCloud2]
    imu: In[Imu]
    color_image: In[Image]
    camera_info: In[CameraInfo]
    odometry: Out[Odometry]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odom_for_tf, self.odometry))
        )

    def _on_odom_for_tf(self, msg: Odometry) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.frame_id,
                child_frame_id=self.config.sensor_frame_id,
                translation=Vector3(
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ),
                rotation=Quaternion(
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ),
                # Match the odometry ts exactly; no `or time.time()` fallback (a
                # real ts of 0.0 must not become wall time).
                ts=msg.ts,
            )
        )

    @rpc
    def stop(self) -> None:
        super().stop()


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    FastLivo()
