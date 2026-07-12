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

"""Derive FAST-LIVO2 extrinsic defaults from the Mid-360 + RealSense rig frames.

FAST-LIVO2 wants:
- ``extrinsic_T``/``extrinsic_R``: lidar frame in IMU frame (p_imu = R * p_lidar + T)
- ``Rcl``/``Pcl``: lidar frame in camera *optical* frame (p_cam = Rcl * p_lidar + Pcl)

Both are computed from the same mount tree the recorder published on tf
(:data:`dimos.robot.assembly.mid360_realsense_30.FRAMES`), so the defaults in
``module.py`` stay consistent with the recorded data. Run manually::

    python -m dimos.hardware.sensors.lidar.fastlivo.scripts.compute_extrinsics
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.robot.assembly.mid360_realsense_30 import FRAMES


def _frame_map() -> dict[str, tuple[str | None, np.ndarray, Rotation]]:
    frames = {}
    for name, parent, xyz, rpy in FRAMES:
        frames[name] = (parent, np.asarray(xyz, dtype=float), Rotation.from_euler("xyz", rpy))
    return frames


def transform_to_root(frame: str) -> tuple[np.ndarray, Rotation]:
    """T_root<-frame as (translation, rotation): p_root = R @ p_frame + t."""
    frames = _frame_map()
    t = np.zeros(3)
    rot = Rotation.identity()
    current: str | None = frame
    while current is not None:
        parent, xyz, rpy = frames[current]
        t = rpy.apply(t) + xyz
        rot = rpy * rot
        current = parent
    return t, rot


def relative_transform(target: str, source: str) -> tuple[np.ndarray, Rotation]:
    """T_target<-source: p_target = R @ p_source + t."""
    t_ts, r_ts = transform_to_root(target)
    t_ss, r_ss = transform_to_root(source)
    r = r_ts.inv() * r_ss
    t = r_ts.inv().apply(t_ss - t_ts)
    return t, r


def main() -> None:
    p_li, r_li = relative_transform("imu_frame", "lidar_frame")
    print("extrinsic_T (lidar in IMU):", np.round(p_li, 6).tolist())
    print("extrinsic_R:", np.round(r_li.as_matrix().flatten(), 6).tolist())

    p_cl, r_cl = relative_transform("color_optical_frame", "lidar_frame")
    print("Pcl (lidar in camera optical):", np.round(p_cl, 6).tolist())
    print("Rcl:", np.round(r_cl.as_matrix().flatten(), 6).tolist())


if __name__ == "__main__":
    main()
