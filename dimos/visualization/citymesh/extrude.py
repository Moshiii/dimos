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

"""Turn footprints and a terrain grid into ENU meshes.

The extrusion itself is trivial — polygon plus height equals prism. The part
worth reading carefully is where each prism's *base* comes from: a footprint is
placed on the terrain at its own location, so a street that climbs a hill
produces buildings that step up it rather than a row hanging off a flat plane.

All color decisions live in :mod:`citymesh.themes`. When the theme asks for
wireframe edges, this module also emits line strips along each prism's bottom
ring, top ring, and vertical corners.

The metric reference grid lives here too (:func:`grid_line_positions`,
:func:`grid_strips`): its lines are placed at exact ENU multiples and draped on
the terrain, so their spacing is a chosen quantity rather than an artifact of the
DEM's sampling.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np
from shapely.geometry import MultiPolygon, Polygon

from dimos.utils.logging_config import setup_logger

from .dem import Terrain
from .frame import EnuFrame
from .overture import Building
from .themes import THEMES, Theme

log = setup_logger()


@dataclass
class Mesh:
    """A triangle mesh in local ENU metres, plus optional wireframe strips."""

    vertices: np.ndarray  # (V, 3) float32
    triangles: np.ndarray  # (T, 3) uint32
    colors: np.ndarray | None = None  # (V, 3) or (V, 4) uint8
    edges: list[np.ndarray] = field(default_factory=list)  # each (K, 3) float32
    # Maps each building id to its slice of `triangles`, for per-entity logging.
    spans: dict[str, tuple[int, int]] = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.triangles)


def _polygon_to_enu(poly: Polygon, frame: EnuFrame, base_msl: float) -> Polygon:
    """Project a lon/lat polygon onto the ENU plane at a fixed base altitude."""

    def ring(coords: Any) -> np.ndarray:
        arr = np.asarray(coords, dtype=float)
        enu = frame.geodetic_to_enu(arr[:, 1], arr[:, 0], np.full(len(arr), base_msl), datum="msl")
        return enu[:, :2]

    return Polygon(
        ring(poly.exterior.coords),
        [ring(i.coords) for i in poly.interiors],
    )


def _ring_edges(ring_xy: np.ndarray, z0: float, z1: float) -> list[np.ndarray]:
    """Wireframe strips for one ring: bottom loop, top loop, vertical corners."""
    closed = ring_xy
    if not np.allclose(closed[0], closed[-1]):
        closed = np.vstack([closed, closed[:1]])
    bottom = np.column_stack([closed, np.full(len(closed), z0)])
    top = np.column_stack([closed, np.full(len(closed), z1)])
    strips = [bottom.astype(np.float32), top.astype(np.float32)]
    for x, y in closed[:-1]:
        strips.append(np.array([[x, y, z0], [x, y, z1]], dtype=np.float32))
    return strips


def extrude_buildings(
    buildings: list[Building],
    frame: EnuFrame,
    terrain: Terrain | None = None,
    flat_ground_msl: float = 0.0,
    theme: Theme = THEMES["day"],
) -> Mesh:
    """Extrude every footprint into a single merged mesh in ENU metres.

    Without ``terrain`` all buildings sit on ``flat_ground_msl``, which is only
    correct on genuinely flat ground.
    """
    import trimesh

    if not buildings:
        raise ValueError("no buildings to extrude")

    centroids = np.array([[b.geometry.centroid.x, b.geometry.centroid.y] for b in buildings])
    if terrain is not None:
        bases_msl = terrain.sample_ground_msl(centroids[:, 0], centroids[:, 1])
    else:
        bases_msl = np.full(len(buildings), flat_ground_msl)

    # One vectorized call for every building's base elevation, instead of a
    # per-building geodetic conversion inside the loop.
    bases_enu_up = frame.geodetic_to_enu(centroids[:, 1], centroids[:, 0], bases_msl, datum="msl")[
        :, 2
    ]

    fills = theme.building_colors(
        np.array([b.height_m for b in buildings]),
        np.array([b.height_is_estimated for b in buildings]),
    )

    verts: list[np.ndarray] = []
    tris: list[np.ndarray] = []
    cols: list[np.ndarray] = []
    edges: list[np.ndarray] = []
    spans: dict[str, tuple[int, int]] = {}
    offset = 0
    skipped = 0

    for b, base_msl, base_enu, fill in zip(buildings, bases_msl, bases_enu_up, fills, strict=True):
        thickness = b.height_m - b.min_height_m
        if thickness <= 0:
            skipped += 1
            continue

        # Base of the prism in ENU up-metres: the terrain here, plus any
        # documented gap between ground and the building's underside.
        z0 = base_enu + b.min_height_m

        polys = b.geometry.geoms if isinstance(b.geometry, MultiPolygon) else [b.geometry]
        tri_start = len(tris)
        for poly in polys:
            if not isinstance(poly, Polygon) or poly.is_empty:
                continue
            try:
                flat = _polygon_to_enu(poly, frame, base_msl)
                if not flat.is_valid:
                    flat = flat.buffer(0)
                if flat.is_empty or flat.area < 0.5:  # sub-square-metre noise
                    continue
                mesh = trimesh.creation.extrude_polygon(flat, height=thickness)
            except Exception as exc:
                log.debug("skipping footprint %s: %s", b.id, exc)
                skipped += 1
                continue

            v = np.asarray(mesh.vertices, dtype=np.float64)
            v[:, 2] += z0
            verts.append(v)
            tris.append(np.asarray(mesh.faces, dtype=np.uint32) + offset)
            cols.append(np.repeat(fill[None, :], len(v), axis=0))
            offset += len(v)

            if theme.edges and isinstance(flat, Polygon):
                for ring in [flat.exterior, *flat.interiors]:
                    edges.extend(_ring_edges(np.asarray(ring.coords), z0, z0 + thickness))

        if len(tris) > tri_start:
            first = sum(len(t) for t in tris[:tri_start])
            spans[b.id] = (first, sum(len(t) for t in tris))

    if not verts:
        raise ValueError("every footprint failed to extrude")
    if skipped:
        log.info("skipped %d degenerate footprints", skipped)

    return Mesh(
        vertices=np.vstack(verts).astype(np.float32),
        triangles=np.vstack(tris).astype(np.uint32),
        colors=np.vstack(cols).astype(np.uint8),
        edges=edges,
        spans=spans,
    )


def terrain_mesh(
    terrain: Terrain,
    frame: EnuFrame,
    stride: int = 1,
    theme: Theme = THEMES["day"],
) -> Mesh:
    """Triangulate the bare-earth grid into an ENU mesh."""
    lons = terrain.lons[::stride]
    lats = terrain.lats[::stride]
    ground = terrain.ground_msl[::stride, ::stride]

    lon_grid, lat_grid = np.meshgrid(lons, lats)
    enu = frame.geodetic_to_enu(lat_grid.ravel(), lon_grid.ravel(), ground.ravel(), datum="msl")

    h, w = ground.shape
    idx = np.arange(h * w).reshape(h, w)
    tl, tr = idx[:-1, :-1], idx[:-1, 1:]
    bl, br = idx[1:, :-1], idx[1:, 1:]
    tris = np.concatenate(
        [
            np.stack([tl, bl, br], axis=-1).reshape(-1, 3),
            np.stack([tl, br, tr], axis=-1).reshape(-1, 3),
        ]
    ).astype(np.uint32)

    return Mesh(
        vertices=enu.astype(np.float32),
        triangles=tris,
        colors=theme.terrain_colors(enu[:, 2]),
    )


def terrain_patch_mesh(
    terrain: Terrain,
    frame: EnuFrame,
    bounds_enu: tuple[float, float, float, float],
    samples: int = 9,
    theme: Theme = THEMES["day"],
    z_range: tuple[float, float] | None = None,
) -> Mesh:
    """Triangulate one ENU rectangle of ground, resampled from ``terrain``.

    Unlike :func:`terrain_mesh`, which follows the DEM's own lon/lat grid, this
    builds a mesh on an axis-aligned ENU grid. Streaming needs that: adjacent
    tiles must share their boundary vertices exactly, or the seams crack.
    """
    e_min, n_min, e_max, n_max = bounds_enu
    es = np.linspace(e_min, e_max, samples)
    ns = np.linspace(n_min, n_max, samples)
    e_grid, n_grid = np.meshgrid(es, ns)
    flat = np.column_stack([e_grid.ravel(), n_grid.ravel(), np.zeros(e_grid.size)])

    lat, lon, _ = frame.enu_to_geodetic(flat)
    ground = terrain.sample_ground_msl(lon, lat)
    enu = frame.geodetic_to_enu(lat, lon, ground, datum="msl")

    idx = np.arange(samples * samples).reshape(samples, samples)
    tl, tr = idx[:-1, :-1], idx[:-1, 1:]
    bl, br = idx[1:, :-1], idx[1:, 1:]
    tris = np.concatenate(
        [
            np.stack([tl, bl, br], axis=-1).reshape(-1, 3),
            np.stack([tl, br, tr], axis=-1).reshape(-1, 3),
        ]
    ).astype(np.uint32)

    return Mesh(
        vertices=enu.astype(np.float32),
        triangles=tris,
        colors=theme.terrain_colors(enu[:, 2], z_range=z_range),
    )


# Metric reference grid
#
# The grid is a property of the ENU frame, not of the terrain raster or of the
# tiling. Every line sits at an exact multiple of its spacing, which is what makes
# it readable as a ruler and what makes independently generated tiles line up: a
# tile computes only which multiples fall inside its own bounds, never a phase of
# its own.

GRID_LIFT_M = 0.15  # above the surface, so the lines do not z-fight the mesh
GRID_STEP_M = 32.0  # drape resolution along a line; ~the 30 m DEM's own spacing
CARPET_M = 100.0  # side of the micro carpet under the vehicle
CARPET_LIFT_M = 0.05

# Multiples are computed in units of the spacing, so this tolerance is in "line
# indices": it only absorbs the float error of dividing a bound by a spacing.
_PHASE_EPS = 1e-9


def grid_line_positions(lo: float, hi: float, spacing_m: float) -> np.ndarray:
    """Multiples of ``spacing_m`` in the half-open interval ``[lo, hi)``.

    Phase comes from the ENU origin, never from ``lo``: two adjacent tiles asking
    about their own bounds get line sets that continue each other exactly.

    Half-open for the same reason :func:`~citymesh.tiles.tile_of` floors — a line
    exactly on a shared border belongs to the tile it starts, so neighbours draw
    it once between them instead of twice on top of each other.
    """
    if spacing_m is None or spacing_m <= 0:
        raise ValueError(f"grid spacing must be positive, got {spacing_m!r}")
    first = math.ceil(lo / spacing_m - _PHASE_EPS)
    last = math.ceil(hi / spacing_m - _PHASE_EPS) - 1
    if last < first:
        return np.empty(0, dtype=float)
    return np.arange(first, last + 1, dtype=float) * spacing_m


def _drape_samples(span_m: float, step_m: float) -> int:
    """Vertices along one line: enough to follow the terrain, no more."""
    return max(math.ceil(abs(span_m) / max(step_m, 1e-6)) + 1, 2)


def grid_strips(
    frame: EnuFrame,
    bounds_enu: tuple[float, float, float, float],
    spacing_m: float,
    terrain: Terrain | None = None,
    lift_m: float = GRID_LIFT_M,
    step_m: float = GRID_STEP_M,
) -> list[np.ndarray]:
    """Grid lines over an ENU rectangle, draped on ``terrain``.

    One strip per line, cut to ``bounds_enu``. Without ``terrain`` the grid is
    flat at ``z = lift_m`` (the ``--flat-ground`` world).

    The drape is bilinear interpolation of the DEM, so a grid finer than the DEM
    is smooth interpolation rather than real relief. That is the intended
    trade — it is a reference grid, not a survey.
    """
    e_min, n_min, e_max, n_max = bounds_enu
    es = grid_line_positions(e_min, e_max, spacing_m)
    ns = grid_line_positions(n_min, n_max, spacing_m)
    if not len(es) and not len(ns):
        return []

    along_n = np.linspace(n_min, n_max, _drape_samples(n_max - n_min, step_m))
    along_e = np.linspace(e_min, e_max, _drape_samples(e_max - e_min, step_m))
    lines = [np.column_stack([np.full(along_n.size, e), along_n]) for e in es]
    lines += [np.column_stack([along_e, np.full(along_e.size, n)]) for n in ns]

    # One geodetic round trip for every vertex of every line, then split back.
    flat = np.vstack(lines)
    if terrain is not None:
        lat, lon, _ = frame.enu_to_geodetic(np.column_stack([flat, np.zeros(len(flat))]))
        enu = frame.geodetic_to_enu(lat, lon, terrain.sample_ground_msl(lon, lat), datum="msl")
    else:
        enu = np.column_stack([flat, np.zeros(len(flat))])
    enu[:, 2] += lift_m

    splits = np.cumsum([len(line) for line in lines])[:-1]
    return [chunk.astype(np.float32) for chunk in np.split(enu, splits)]


def carpet_anchor(e: float, n: float, spacing_m: float) -> tuple[float, float]:
    """Snap a vehicle position to the global ``k * spacing`` phase.

    The lines are phase-aligned anyway, so this does not move them; it keeps the
    carpet's *extent* on the same lattice, so re-anchoring adds and drops whole
    lines instead of shifting the edge by a fraction of a cell.
    """
    return (
        round(e / spacing_m) * spacing_m,
        round(n / spacing_m) * spacing_m,
    )


def carpet_bounds(
    anchor: tuple[float, float], size_m: float = CARPET_M
) -> tuple[float, float, float, float]:
    """``(e_min, n_min, e_max, n_max)`` of a carpet centred on ``anchor``."""
    ae, an = anchor
    half = size_m / 2.0
    return (ae - half, an - half, ae + half, an + half)


def carpet_needs_reanchor(
    anchor: tuple[float, float] | None,
    e: float,
    n: float,
    size_m: float = CARPET_M,
) -> bool:
    """Whether the vehicle has left the middle of the carpet at ``anchor``.

    Half a carpet of slack: re-logging on every fix would be pointless traffic,
    and any larger threshold would let the vehicle reach the carpet's edge.
    """
    if anchor is None:
        return True
    return math.hypot(e - anchor[0], n - anchor[1]) > size_m / 2.0
