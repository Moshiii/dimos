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

"""CityMesh as a dimos module: NavSatFix in, city geometry out.

The module owns no renderer and no pose math. Tiles stream from the OSM/DEM
builder into :class:`EntityMesh` messages whose vertices live in the ``enu``
frame — a local tangent frame anchored at the snapped origin of the first
fix, x = East, y = North, z = Up from sea level. Placing the city in the
robot's world is the job of whoever publishes a ``world -> enu`` transform
on tf (a compass-equipped platform's connection module, :class:`EnuSnapTF`,
or a future registration module); until one does, the city renders only in
a view rooted at the city path.
"""

from __future__ import annotations

from collections.abc import Callable
import math
from typing import TYPE_CHECKING, Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.visualization_msgs.EntityMesh import EntityMesh
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.visualization.citymesh.dem import Terrain
    from dimos.visualization.citymesh.extrude import Mesh
    from dimos.visualization.citymesh.tiles import Source, TileData, TileKey, TileStreamer

logger = setup_logger()


class MeshPublisher:
    """The :class:`~dimos.visualization.citymesh.tiles.TileSink` that publishes.

    Tile events become :class:`EntityMesh` messages on a topic instead of
    ``rr.log`` calls: one per mesh under ``<root>/tiles/<key>``, a recursive
    clear when the streamer unloads the tile.
    """

    wants_carpet = False

    def __init__(self, publish: Callable[[EntityMesh], None], root: str, frame_id: str) -> None:
        self.publish = publish
        self.root = root
        self.frame_id = frame_id

    def _entity(self, key: TileKey) -> str:
        return f"{self.root}/tiles/{key}"

    def _mesh(self, path: str, mesh: Mesh, ts: float | None) -> None:
        self.publish(
            EntityMesh(
                path=path,
                frame_id=self.frame_id,
                vertices=mesh.vertices,
                triangles=mesh.triangles,
                colors=mesh.colors,
                ts=ts,
            )
        )

    def tile(self, data: TileData, at_t: float | None, now_t: float | None) -> None:
        entity = self._entity(data.key)
        ts = now_t if now_t is not None else at_t
        if data.terrain is not None:
            self._mesh(f"{entity}/terrain", data.terrain, ts)
        if data.buildings is not None:
            self._mesh(f"{entity}/buildings", data.buildings, ts)

    def clear(self, key: TileKey) -> None:
        self.publish(EntityMesh.clear(self._entity(key)))

    def dead(self, key: TileKey, bounds: tuple[float, float, float, float]) -> None:
        logger.warning("city tile failed for good", key=str(key), bounds=bounds)

    def carpet(self, e: float, n: float, terrain: Terrain | None) -> bool:
        return False


class CityStream:
    """The engine: fixes in, meshes out. Owns the frame and the streamer."""

    def __init__(
        self,
        publish: Callable[[EntityMesh], None],
        theme: str = "blueprint",
        source: Source = "osm",
        root: str = "world/city",
        frame_id: str = "enu",
        flat_ground: bool = False,
        load_radius_m: float = 400.0,
        unload_radius_m: float = 700.0,
        workers: int = 2,
        default_building_height_m: float = 21.0,
    ) -> None:
        self.publish = publish
        self.theme = theme
        self.source: Source = source
        self.root = root
        self.frame_id = frame_id
        self.flat_ground = flat_ground
        self.load_radius_m = load_radius_m
        self.unload_radius_m = unload_radius_m
        self.workers = workers
        self.default_building_height_m = default_building_height_m

        self.frame: Any = None
        self.streamer: TileStreamer | None = None

    def on_fix(self, msg: NavSatFix) -> None:
        if not msg.has_fix or not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if self.streamer is None:
            self._anchor(msg)
        assert self.streamer is not None
        e, n, _ = self.frame.geodetic_to_enu(
            msg.latitude, msg.longitude, self.frame.origin_msl, datum="msl"
        )[0]
        self.streamer.update([float(e), float(n), 0.0], t_s=msg.ts)

    def _anchor(self, msg: NavSatFix) -> None:
        from dimos.visualization.citymesh.frame import EnuFrame, snap_origin
        from dimos.visualization.citymesh.themes import load_theme
        from dimos.visualization.citymesh.tiles import TileStreamer

        lat0, lon0 = snap_origin(msg.latitude, msg.longitude)
        # Sea-level origin, always: the scene is absolute-MSL and the origin —
        # hence the fetch cache — never depends on what altitude the fix carried.
        self.frame = EnuFrame.at(lat0, lon0, 0.0, datum="msl", undulation=0.0)
        self.streamer = TileStreamer(
            self.frame,
            MeshPublisher(self.publish, root=self.root, frame_id=self.frame_id),
            theme=load_theme(self.theme),
            source=self.source,
            load_radius_m=self.load_radius_m,
            unload_radius_m=self.unload_radius_m,
            workers=self.workers,
            flat_ground=self.flat_ground,
            default_height_m=self.default_building_height_m,
        )
        logger.info("citymesh anchored", lat=lat0, lon=lon0, frame=self.frame_id)

    def close(self) -> None:
        if self.streamer is not None:
            self.streamer.close(drain_timeout_s=5.0)
            self.streamer = None


class Config(ModuleConfig):
    theme: str = "blueprint"
    source: str = "osm"
    root: str = "world/city"
    frame: str = "enu"
    flat_ground: bool = False
    load_radius_m: float = 400.0
    unload_radius_m: float = 700.0
    workers: int = 2
    default_building_height_m: float = 21.0


class CityMeshModule(Module):
    """Streams extruded OSM/DEM city tiles around incoming GPS fixes."""

    config: Config
    gps: In[NavSatFix]
    city: Out[EntityMesh]

    @rpc
    def start(self) -> None:
        super().start()
        self._stream = CityStream(
            self.city.publish,
            theme=self.config.theme,
            source=self.config.source,  # type: ignore[arg-type]
            root=self.config.root,
            frame_id=self.config.frame,
            flat_ground=self.config.flat_ground,
            load_radius_m=self.config.load_radius_m,
            unload_radius_m=self.config.unload_radius_m,
            workers=self.config.workers,
            default_building_height_m=self.config.default_building_height_m,
        )
        self.register_disposable(self.gps.observable().subscribe(self._on_fix))  # type: ignore[no-untyped-call]

    @rpc
    def stop(self) -> None:
        self._stream.close()
        super().stop()

    def _on_fix(self, msg: NavSatFix) -> None:
        try:
            self._stream.on_fix(msg)
        except Exception:
            logger.exception("citymesh fix handling failed")
