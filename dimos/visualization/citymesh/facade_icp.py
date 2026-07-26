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

"""Refine the GPS georegistration against building facades the lidar can see.

GPS gives the coarse lock (a few metres, a few degrees, and *biased* — weak
receivers skew a whole track coherently). Building walls are long straight
constraints in a known map, so matching them against OSM footprints pins the
transform to map accuracy: on a real balcony recording this took the GPS
answer 12 degrees and 6 metres to a 0.66 m rms facade fit.

Three pieces, pure numpy/scipy like :mod:`.georegister`:

* :class:`WallAccumulator` — folds odom-frame scans into 2D voxels; a voxel
  whose points span enough height is a wall. Viewpoint-independent: works
  from a balcony looking down as well as from the street.
* :func:`build_edges` — OSM footprints -> densely sampled edge points with
  outward normals, in ENU.
* :func:`refine` — annealed point-to-line Gauss-Newton ICP from the GPS
  seed, with an inlier-fraction gate so a bad scene keeps the GPS answer.
"""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import math
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from dimos.visualization.citymesh.frame import EnuFrame
    from dimos.visualization.citymesh.overture import Building


class WallAccumulator:
    """Streamed verticality detector over odom-frame points.

    Keeps per-voxel z extents and centroids; ``walls()`` returns the 2D
    centroids of voxels that grew a wall's worth of vertical span, and
    ``grounds()`` the flat voxels (for the DEM height estimate). Bounded by
    pruning voxels far from the last scan's origin.
    """

    def __init__(
        self,
        voxel_m: float = 0.75,
        wall_span_m: float = 4.0,
        min_points: int = 5,
        keep_radius_m: float = 150.0,
    ) -> None:
        self.voxel_m = voxel_m
        self.wall_span_m = wall_span_m
        self.min_points = min_points
        self.keep_radius_m = keep_radius_m
        # (ix, iy) -> [min_z, max_z, n, sum_x, sum_y, sum_z]
        self._cells: dict[tuple[int, int], list[float]] = {}

    def add_scan(self, points: np.ndarray, origin_xy: tuple[float, float]) -> None:
        """Fold one odom-frame scan in; ``origin_xy`` prunes far clutter."""
        if len(points) == 0:
            return
        key = np.round(points[:, :2] / self.voxel_m).astype(np.int64)
        order = np.lexsort((key[:, 1], key[:, 0]))
        key_s, pts_s = key[order], points[order]
        cuts = np.flatnonzero(np.any(np.diff(key_s, axis=0) != 0, axis=1)) + 1
        for group in np.split(np.arange(len(key_s)), cuts):
            k = (int(key_s[group[0], 0]), int(key_s[group[0], 1]))
            z = pts_s[group, 2]
            cell = self._cells.get(k)
            if cell is None:
                cell = [float(z.min()), float(z.max()), 0.0, 0.0, 0.0, 0.0]
                self._cells[k] = cell
            cell[0] = min(cell[0], float(z.min()))
            cell[1] = max(cell[1], float(z.max()))
            cell[2] += len(group)
            cell[3] += float(pts_s[group, 0].sum())
            cell[4] += float(pts_s[group, 1].sum())
            cell[5] += float(z.sum())
        self._prune(origin_xy)

    def _prune(self, origin_xy: tuple[float, float]) -> None:
        if len(self._cells) < 50_000:
            return
        ox, oy = origin_xy
        r_vox = self.keep_radius_m / self.voxel_m
        self._cells = {
            k: c
            for k, c in self._cells.items()
            if (k[0] - ox / self.voxel_m) ** 2 + (k[1] - oy / self.voxel_m) ** 2 <= r_vox**2
        }

    def _select(self, wall: bool) -> np.ndarray:
        out = []
        for cell in self._cells.values():
            if cell[2] < self.min_points:
                continue
            span = cell[1] - cell[0]
            if (span >= self.wall_span_m) == wall:
                n = cell[2]
                out.append((cell[3] / n, cell[4] / n, cell[5] / n))
        return np.array(out) if out else np.empty((0, 3))

    def walls(self) -> np.ndarray:
        """(N, 2) odom-frame centroids of wall voxels."""
        sel = self._select(wall=True)
        return sel[:, :2] if len(sel) else np.empty((0, 2))

    def grounds(self) -> np.ndarray:
        """(N, 3) centroids of flat voxels — candidates for the DEM z fit."""
        return self._select(wall=False)


@dataclass
class Edges:
    """Footprint boundaries as dense samples with unit normals, plus a tree."""

    samples: np.ndarray  # (M, 2) ENU
    normals: np.ndarray  # (M, 2)
    tree: object  # scipy cKDTree

    @property
    def n(self) -> int:
        return len(self.samples)


def build_edges(buildings: list[Building], frame: EnuFrame, sample_m: float = 0.5) -> Edges | None:
    """OSM footprints -> ENU edge samples + normals. None when there are none."""
    from scipy.spatial import cKDTree

    samples: list[np.ndarray] = []
    normals: list[np.ndarray] = []
    for b in buildings:
        for g in getattr(b.geometry, "geoms", [b.geometry]):
            coords = np.asarray(g.exterior.coords)
            enu = frame.geodetic_to_enu(coords[:, 1], coords[:, 0], 0.0, datum="ellipsoidal")[:, :2]
            for a, c in itertools.pairwise(enu):
                length = float(np.linalg.norm(c - a))
                if length < 0.5:
                    continue
                tangent = (c - a) / length
                normal = np.array([-tangent[1], tangent[0]])
                pts = np.linspace(a, c, max(2, int(length / sample_m)))
                samples.append(pts)
                normals.append(np.tile(normal, (len(pts), 1)))
    if not samples:
        return None
    s = np.vstack(samples)
    return Edges(samples=s, normals=np.vstack(normals), tree=cKDTree(s))


@dataclass(frozen=True)
class IcpResult:
    """``enu ≈ R(yaw) @ odom + t`` refined against facades."""

    yaw: float
    t_e: float
    t_n: float
    rms_m: float
    inlier_frac: float
    n_walls: int


def apply2d(yaw: float, t: np.ndarray, pts: np.ndarray) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(pts @ np.array([[c, s], [-s, c]]) + t)


RADII = (35.0, 25.0, 15.0, 10.0, 6.0, 4.0, 2.5, 1.5)


def refine(
    walls_xy: np.ndarray,
    edges: Edges,
    yaw0: float,
    t0: tuple[float, float],
    radii: tuple[float, ...] = RADII,
    min_walls: int = 80,
    min_inlier_frac: float = 0.45,
    inlier_m: float = 1.5,
) -> IcpResult | None:
    """Annealed point-to-line ICP from the GPS seed; None when unconvincing.

    The coarse-to-fine radius schedule is the convergence basin: a biased GPS
    seed can start ~20 m and ~15 degrees off. The final inlier fraction is
    the accept gate — a scene without enough agreeing walls (open field,
    unmapped buildings) keeps the GPS answer rather than a confident hallucination.
    """
    if len(walls_xy) < min_walls:
        return None
    yaw = yaw0
    t = np.asarray(t0, dtype=float).copy()
    for radius in radii:
        for _ in range(10):
            moved = apply2d(yaw, t, walls_xy)
            dist, idx = edges.tree.query(moved, distance_upper_bound=radius)  # type: ignore[attr-defined]
            ok = np.isfinite(dist)
            if ok.sum() < min_walls // 2:
                break
            p = moved[ok]
            q = edges.samples[idx[ok]]
            n = edges.normals[idx[ok]]
            residual = ((p - q) * n).sum(axis=1)
            c, s = math.cos(yaw), math.sin(yaw)
            dp = walls_xy[ok] @ np.array([[-s, c], [-c, -s]])
            jac = np.column_stack([(dp * n).sum(axis=1), n[:, 0], n[:, 1]])
            weight = np.minimum(1.0, radius * 0.5 / np.maximum(np.abs(residual), 1e-9))
            try:
                dx, *_ = np.linalg.lstsq(jac * weight[:, None], -(residual * weight), rcond=None)
            except np.linalg.LinAlgError:
                return None
            yaw += float(dx[0])
            t += dx[1:]
            if np.abs(dx).max() < 1e-4:
                break
    dist, _ = edges.tree.query(apply2d(yaw, t, walls_xy), distance_upper_bound=inlier_m)  # type: ignore[attr-defined]
    ok = np.isfinite(dist)
    frac = float(ok.mean())
    if frac < min_inlier_frac:
        return None
    return IcpResult(
        yaw=yaw,
        t_e=float(t[0]),
        t_n=float(t[1]),
        rms_m=float(np.sqrt((dist[ok] ** 2).mean())),
        inlier_frac=frac,
        n_walls=len(walls_xy),
    )
