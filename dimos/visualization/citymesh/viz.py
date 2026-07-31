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

"""Rerun logging for the georeferenced scene.

The rerun side of the :class:`~dimos.visualization.citymesh.tiles.TileSink`
seam: everything that turns citymesh's plain ENU geometry into ``rr.log`` calls
lives here, so the streaming machinery in :mod:`.tiles` stays renderer-neutral.

The entity layout under ``root`` (default ``world``)::

    <root>/
      tiles/{ix}_{iy}/
        terrain/              draped ground patch per tile
          grid/major, minor   metric reference grid
        buildings             extruded footprints
      grid/micro              the fine carpet, re-logged as the vehicle moves
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager

import numpy as np

from dimos.utils.logging_config import setup_logger

from .dem import Terrain
from .extrude import (
    CARPET_LIFT_M,
    CARPET_M,
    Mesh,
    carpet_anchor,
    carpet_bounds,
    carpet_needs_reanchor,
    grid_strips,
)
from .frame import EnuFrame
from .themes import THEMES, Theme
from .tiles import FLIGHT_TIMELINE, TileData, TileKey

log = setup_logger()

WORLD = "world"


def init_world(frame: EnuFrame, root: str = WORLD) -> None:
    """Declare the scene root's axes and anchor metadata, statically.

    No blueprint here — in dimos the viewer layout is composed by the robot
    blueprint that adds the city view, not by citymesh itself.
    """
    import rerun as rr

    rr.log(root, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        f"{root}/origin",
        rr.TextDocument(
            f"ENU origin\n"
            f"  lat {frame.lat0:.7f}\n"
            f"  lon {frame.lon0:.7f}\n"
            f"  ellipsoidal {frame.h0_ellipsoidal:.2f} m\n"
            f"  orthometric (MSL) {frame.origin_msl:.2f} m\n"
            f"  geoid undulation {frame.undulation:+.2f} m",
            media_type=rr.MediaType.TEXT,
        ),
        static=True,
    )


def log_mesh(path: str, mesh: Mesh, theme: Theme = THEMES["day"], static: bool = True) -> None:
    """Log a mesh, and its wireframe strips (if any) as a sibling entity.

    A theme alpha rides the vertex colors, but the viewer selects its
    translucent pipeline off the *material's* albedo alpha — vertex alpha on
    an opaque material renders solid. Move the alpha to ``albedo_factor`` and
    keep the vertices fully opaque so it applies exactly once.
    """
    import rerun as rr

    colors = mesh.colors
    albedo_factor = None
    if colors is not None and colors.ndim == 2 and colors.shape[1] == 4:
        albedo_factor = [255, 255, 255, int(colors[:, 3].max())]
        colors = np.column_stack([colors[:, :3], np.full(len(colors), 255, dtype=colors.dtype)])
    rr.log(
        path,
        rr.Mesh3D(
            vertex_positions=mesh.vertices,
            triangle_indices=mesh.triangles,
            vertex_colors=colors,
            albedo_factor=albedo_factor,
        ),
        static=static,
    )
    if mesh.edges:
        rr.log(
            f"{path}/edges",
            rr.LineStrips3D(
                mesh.edges,
                colors=[theme.edge_color],
                radii=[theme.edge_radius_m],
            ),
            static=static,
        )


def log_strips(
    path: str,
    strips: list[np.ndarray],
    color: tuple[int, int, int],
    radius_m: float,
    static: bool = True,
) -> None:
    """Log line strips with radii in *scene metres*.

    Metres, not ui points, is what gives the grid its zoom LOD for free: a 1 cm
    line is sub-pixel from altitude and materializes as the camera descends. Ui
    radii would keep every layer equally thick at every distance and turn the fine
    grids into a grey wash.
    """
    import rerun as rr

    rr.log(
        path,
        rr.LineStrips3D(strips, colors=[color], radii=[radius_m]),
        static=static,
    )


class MicroCarpet:
    """The finest grid layer, following the vehicle.

    A 1 m grid over a city is millions of strips, so the micro layer exists only
    as a small square carpet under the vehicle. It is re-logged (non-static) on
    one entity path, so each log replaces the previous carpet rather than piling
    up — and only when the vehicle has moved half a carpet, which is a couple of
    hundred strips every few seconds instead of on every fix.

    Its lines come from the same phase as the major and minor grids, so zooming in
    reveals a finer division of the same lattice rather than a second, unrelated
    one.
    """

    def __init__(
        self,
        frame: EnuFrame,
        theme: Theme = THEMES["day"],
        path: str = f"{WORLD}/grid/micro",
        size_m: float = CARPET_M,
        lift_m: float = CARPET_LIFT_M,
        static: bool = False,
    ) -> None:
        self.frame = frame
        self.path = path
        self.size_m = size_m
        self.lift_m = lift_m
        self.static = static
        self.layer = theme.micro_layer()
        self.anchor: tuple[float, float] | None = None
        self.n_strips = 0

    @property
    def enabled(self) -> bool:
        return self.layer is not None

    def update(self, e: float, n: float, terrain: Terrain | None = None) -> bool:
        """Re-anchor and re-log the carpet if the vehicle has left its middle.

        ``terrain`` drapes it; None gives a flat carpet at z = ``lift_m``, which
        is right for a ``--flat-ground`` world and wrong otherwise — callers with
        terrain that is not loaded yet should skip the call, not pass None.

        Returns True if the carpet was logged.
        """
        if self.layer is None:
            return False
        if not carpet_needs_reanchor(self.anchor, e, n, self.size_m):
            return False

        anchor = carpet_anchor(e, n, self.layer.spacing_m)
        strips = grid_strips(
            self.frame,
            carpet_bounds(anchor, self.size_m),
            self.layer.spacing_m,
            terrain=terrain,
            lift_m=self.lift_m,
        )
        if not strips:
            return False
        log_strips(
            self.path,
            strips,
            color=self.layer.color,
            radius_m=self.layer.radius_m,
            static=self.static,
        )
        self.anchor = anchor
        self.n_strips = len(strips)
        return True


def log_trail(path: str, points_enu: np.ndarray, theme: Theme = THEMES["day"]) -> None:
    """Log a travelled path as a line strip in world ENU."""
    import rerun as rr

    rr.log(
        path,
        rr.LineStrips3D(
            [np.asarray(points_enu, dtype=np.float32)],
            colors=[theme.trail_color],
            radii=[0.4],
        ),
    )


class RerunTileSink:
    """The rerun implementation of :class:`~dimos.visualization.citymesh.tiles.TileSink`.

    Owns everything renderer-specific the streamer used to reach for directly:
    the entity paths under ``root``, the micro carpet, and the backdating
    timeline dance. Restoring the timeline after a backdated tile matters as
    much as moving it: everything logged after ``update`` returns — the trail,
    the next fix — must land at the present, not wherever the last tile came
    from.
    """

    def __init__(
        self,
        frame: EnuFrame,
        theme: Theme = THEMES["day"],
        root: str = WORLD,
        timeline: str = FLIGHT_TIMELINE,
        carpet_m: float = CARPET_M,
    ) -> None:
        self.theme = theme
        self.root = root
        self.timeline = timeline
        self.micro = MicroCarpet(frame, theme=theme, path=f"{root}/grid/micro", size_m=carpet_m)

    @property
    def wants_carpet(self) -> bool:
        return self.micro.enabled

    def _entity(self, key: TileKey) -> str:
        return f"{self.root}/tiles/{key}"

    @contextmanager
    def _backdated_to(self, at_t: float | None, now_t: float | None) -> Iterator[None]:
        import rerun as rr

        if at_t is None or now_t is None:
            yield
            return
        rr.set_time(self.timeline, duration=at_t)
        try:
            yield
        finally:
            rr.set_time(self.timeline, duration=now_t)

    def tile(self, data: TileData, at_t: float | None, now_t: float | None) -> None:
        entity = self._entity(data.key)
        with self._backdated_to(at_t, now_t):
            if data.terrain is not None:
                log_mesh(f"{entity}/terrain", data.terrain, self.theme, static=False)
            for layer, strips in data.grid:
                log_strips(
                    f"{entity}/terrain/grid/{layer.name}",
                    strips,
                    color=layer.color,
                    radius_m=layer.radius_m,
                    static=False,
                )
            if data.buildings is not None:
                log_mesh(f"{entity}/buildings", data.buildings, self.theme, static=False)

    def clear(self, key: TileKey) -> None:
        import rerun as rr

        rr.log(self._entity(key), rr.Clear(recursive=True))

    def dead(self, key: TileKey, bounds: tuple[float, float, float, float]) -> None:
        """Draw a flat outline where a tile could not be fetched."""
        import rerun as rr

        e0, n0, e1, n1 = bounds
        ring = np.array(
            [[e0, n0, 0.0], [e1, n0, 0.0], [e1, n1, 0.0], [e0, n1, 0.0], [e0, n0, 0.0]],
            dtype=np.float32,
        )
        rr.log(
            f"{self._entity(key)}/failed",
            rr.LineStrips3D([ring], colors=[(220, 60, 60)], radii=[0.6]),
        )

    def carpet(self, e: float, n: float, terrain: Terrain | None) -> bool:
        return self.micro.update(e, n, terrain)
