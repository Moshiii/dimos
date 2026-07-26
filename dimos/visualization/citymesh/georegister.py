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

"""Align the odometry track to the GPS track: where odom sits in ENU.

The Go2 has no compass — IMU yaw is relative, GPS fixes are headingless — so
the rotation between the odom frame and ENU is unobservable until the robot
*moves*: then the two tracks share a shape, and the rigid transform between
them is a 2D Procrustes solve. Pure math here, no topics and no rerun; the
:class:`~dimos.visualization.citymesh.layer.CityMeshLayer` feeds it and
consumes the estimate.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Registration:
    """``enu ≈ R(yaw) @ odom + t``, with the residual as a quality signal."""

    yaw: float  # radians, odom x-axis measured from east
    t_e: float
    t_n: float
    rms_m: float
    n_pairs: int

    def odom_from_enu(self) -> tuple[float, tuple[float, float]]:
        """The inverse (yaw, translation): where the ENU scene sits in odom."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        # R^-1 @ (p - t): rotation -yaw, translation -R^-1 t.
        te = -(c * self.t_e + s * self.t_n)
        tn = -(-s * self.t_e + c * self.t_n)
        return -self.yaw, (te, tn)


def rigid_align_2d(odom_xy: np.ndarray, enu_xy: np.ndarray) -> tuple[float, np.ndarray, float]:
    """Least-squares rotation+translation with ``enu ≈ R @ odom + t``.

    2D Procrustes without scale: the optimal yaw has the closed form
    ``atan2(sum(x*n' - y*e'), sum(x*e' + y*n'))`` over centered pairs.
    Returns ``(yaw, t, rms)``.
    """
    odom_c = odom_xy - odom_xy.mean(axis=0)
    enu_c = enu_xy - enu_xy.mean(axis=0)
    h = odom_c.T @ enu_c
    yaw = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s], [s, c]])
    t = enu_xy.mean(axis=0) - r @ odom_xy.mean(axis=0)
    residual = (r @ odom_xy.T).T + t - enu_xy
    rms = float(np.sqrt((residual**2).sum(axis=1).mean()))
    return yaw, t, rms


class TrackAligner:
    """Pairs a high-rate odometry track with 1 Hz fixes and solves when ready.

    ``add_odom`` is cheap (deque append) and safe at sensor rate. ``add_fix``
    interpolates the odom pose at the fix's timestamp — a fix without odometry
    bracketing it in time is dropped, so clock skew degrades coverage rather
    than corrupting pairs. The solve is gated on both tracks having *shape*:
    with GPS noise of metres, a track shorter than ``min_extent_m`` would fit
    mostly noise (yaw error ≈ noise/extent radians).
    """

    def __init__(
        self,
        # 5 m of spread with ~40 noisy pairs recovers yaw to a few degrees
        # (measured on a real 12.5 m Go2 walk with 2 m synthetic GPS noise);
        # the estimate keeps refining as the track grows.
        min_extent_m: float = 5.0,
        min_pairs: int = 8,
        max_pairs: int = 3600,
        odom_window_s: float = 30.0,
    ) -> None:
        self.min_extent_m = min_extent_m
        self.min_pairs = min_pairs
        self._odom: deque[tuple[float, float, float]] = deque()
        self._odom_window_s = odom_window_s
        self._pairs: deque[tuple[float, float, float, float]] = deque(maxlen=max_pairs)

    def add_odom(self, ts: float, x: float, y: float) -> None:
        self._odom.append((ts, x, y))
        while self._odom and ts - self._odom[0][0] > self._odom_window_s:
            self._odom.popleft()

    def add_fix(self, ts: float, e: float, n: float) -> None:
        odom = self._interpolate(ts)
        if odom is not None:
            self._pairs.append((odom[0], odom[1], e, n))

    def _interpolate(self, ts: float) -> tuple[float, float] | None:
        """Odom position at ``ts``, linearly interpolated between neighbours."""
        prev = None
        for t, x, y in self._odom:
            if t >= ts:
                if prev is None:
                    return None
                t0, x0, y0 = prev
                if t == t0:
                    return (x, y)
                a = (ts - t0) / (t - t0)
                return (x0 + a * (x - x0), y0 + a * (y - y0))
            prev = (t, x, y)
        return None

    def solve(self) -> Registration | None:
        """The current best registration, or None while under-determined."""
        if len(self._pairs) < self.min_pairs:
            return None
        pairs = np.asarray(self._pairs)
        odom_xy, enu_xy = pairs[:, 0:2], pairs[:, 2:4]
        if _extent(odom_xy) < self.min_extent_m or _extent(enu_xy) < self.min_extent_m:
            return None
        yaw, t, rms = rigid_align_2d(odom_xy, enu_xy)
        return Registration(
            yaw=yaw, t_e=float(t[0]), t_n=float(t[1]), rms_m=rms, n_pairs=len(pairs)
        )


def _extent(xy: np.ndarray) -> float:
    """Track spread: twice the RMS distance from the centroid.

    Cheap stand-in for the diameter that a single outlier can't inflate the
    way max-pairwise-distance could.
    """
    centered = xy - xy.mean(axis=0)
    return 2.0 * float(np.sqrt((centered**2).sum(axis=1).mean()))
