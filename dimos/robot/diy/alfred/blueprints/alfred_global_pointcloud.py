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

"""Alfred building a globally consistent point cloud from a D455 alone.

    dimos run alfred-global-pointcloud

``alfred_cuvslam`` localises and stops there. This adds the map: the camera's RGB-D cloud
is accumulated into a voxel map, and crucially it is accumulated against cuVSLAM's
**loop-closure-corrected** pose rather than its raw odometry. That is the whole difference
between a cloud that is locally tidy and one that still lines up after a revisit -- raw
odometry drifts without bound, so a corridor walked twice lands as two parallel walls.

Wiring, and why each edge exists:

* The IR stereo pair drives cuVSLAM, exactly as in ``alfred_cuvslam``.
* ``corrected_odometry`` (``map -> base_link``) feeds the voxel map, **not** ``odometry``
  (``odom -> base_link``). Remapped on the consumer side because ``CuvslamOdometry``
  already publishes a stream literally named ``odometry``; renaming its corrected output
  would collide with it.
* The colour + depth pair produces the cloud that gets accumulated.

``RayTracingVoxelMap`` transforms each cloud into the world itself (``rot * p + t`` from
the nearest-stamped pose) and raycasts to clear voxels that a later view saw through, so
things that move do not smear permanently into the map.

Two honest caveats, both worth knowing before trusting the output:

**The emitter is off, so depth is noisier than this camera can manage.** The projector's
dot pattern is what gives the D455 usable depth on blank indoor walls, but it also moves
rigidly with the camera, so feature trackers latch onto it and bias motion toward zero.
Those two wants are in direct conflict on one imager pair, and this blueprint resolves it
in favour of the pose, because a globally consistent map is bounded by localisation
quality first -- a beautiful cloud at the wrong pose is worthless. The cost is sparser,
noisier geometry on low-texture surfaces. Alternating the emitter per frame would get both
and is not currently exposed by the driver.

**The cloud and the pose are ~3 cm apart in frame.** The published cloud is built from
``_color_camera_info``, so it lives in the colour optical frame, while cuVSLAM's pose is
that of the left IR imager. The rotation between them is negligible (both are optical
frames) but the translation is not zero. It is well under the 5 cm default voxel, so it
does not smear the map, but it is a real systematic offset rather than a rounding error.
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.mapping.cuvslam_native.cuvslam import CuvslamOdometry
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevel
from dimos.visualization.vis_module import vis_module

CAMERA_NAME = "d455"
# Depth error grows with the square of range on a stereo camera; past about 6 m a D455's
# returns are dominated by subpixel noise and would thicken every far surface in the map.
MAX_MAP_RANGE_M = 6.0

alfred_global_pointcloud = (
    autoconnect(
        RealSenseCamera.blueprint(
            camera_name=CAMERA_NAME,
            width=848,
            height=480,
            fps=30,
            enable_infrared=True,
            emitter_enabled=False,
            # The cloud is built from colour + depth, so unlike alfred_cuvslam both are on.
            enable_color=True,
            enable_depth=True,
            enable_pointcloud=True,
            # Depth is deprojected with the colour intrinsics, so the two must be aligned
            # or every point lands offset by the colour-to-depth extrinsic.
            align_depth_to_color=True,
            # The map integrates far slower than the tracker; 5 Hz of cloud is plenty and
            # keeps the deprojection off the critical path.
            pointcloud_fps=5.0,
            enable_imu=False,
            base_transform=None,
        ),
        CuvslamOdometry.blueprint(
            enable_imu=False,
            async_sba=False,
            # Loop closure is the point of this blueprint, not an optimisation.
            enable_slam=True,
            base_frame="base_link",
            odom_frame="odom",
            map_frame="map",
        ),
        RayTracingVoxelMap.blueprint(
            voxel_size=0.05,
            max_range=MAX_MAP_RANGE_M,
        ),
        MovementManager.blueprint(),
        AlfredHighLevel.blueprint(),
        vis_module(global_config.viewer),
    )
    .remappings(
        [
            (RealSenseCamera, "infrared_left", "image_left"),
            (RealSenseCamera, "infrared_right", "image_right"),
            (RealSenseCamera, "infrared_left_camera_info", "camera_info"),
            (RealSenseCamera, "pointcloud", "lidar"),
            (RayTracingVoxelMap, "odometry", "corrected_odometry"),
        ]
    )
    .global_config(n_workers=7)
)
