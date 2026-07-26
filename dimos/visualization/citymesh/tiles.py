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

"""Dynamic tile streaming: load the world around a moving vehicle, unload behind it.

The ENU frame never moves. Re-anchoring it as the vehicle flew would rewrite
every transform already logged and destroy the timeline's meaning, so instead the
*world* is cut into fixed squares in the ENU plane and streamed in and out around
the vehicle. "Centre the world on the drone" is a viewer concern, handled by the
viewer blueprint, not by moving geometry.

Three pieces, deliberately separable:

* :func:`tile_of` / :func:`tile_bounds` — pure integer tile algebra.
* :class:`TilePolicy` — pure load/unload bookkeeping with hysteresis. No I/O, no
  rendering; this is the part worth unit-testing.
* :class:`TileStreamer` — glues the policy to the fetchers (on a small thread
  pool) and to a :class:`TileSink` (always on the calling thread, so the order of
  emitted geometry is deterministic).

Nothing in this module knows about rerun: built tiles leave through the
:class:`TileSink` protocol as plain :class:`TileData`, and the rerun
implementation lives in :mod:`.viz`. A different renderer implements the same
four events.

The reference grid follows the same principle in reverse: its lines are placed at
multiples of the theme's spacing *in the ENU frame* and merely cut to each tile's
bounds, so tiles built independently — in any order, on any thread — produce lines
that continue across their shared borders instead of each starting a phase of its
own.

Ownership rule: a footprint belongs to exactly one tile, the one containing its
*centroid*. Fetchers return everything intersecting a bbox, so a building
straddling a border would otherwise be extruded twice and z-fight itself. The
centroid rule gives a clean partition, at the cost of a building poking a few
metres past its tile's edge — which is invisible, and cheaper than clipping
geometry.

Precision note: positions are float32 in rerun, where the spacing at 30 km from
the origin is still ~2 mm. That bounds a session's usable area; flying much
further than that wants a new frame (and a new recording).
"""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable, Sequence
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
import math
import threading
import time
from typing import Generic, Literal, NamedTuple, Protocol, TypeVar

import numpy as np

from dimos.utils.logging_config import setup_logger

from .dem import Terrain, fetch_terrain
from .extrude import Mesh, extrude_buildings, grid_strips, terrain_patch_mesh
from .frame import EnuFrame
from .osm import fetch_buildings_osm
from .overture import Building, fetch_buildings
from .themes import THEMES, GridLayer, Theme

log = setup_logger()

TILE_M = 256.0
LOAD_RADIUS_M = 600.0
UNLOAD_RADIUS_M = 900.0

# Tiles are the unit of *ownership* and of load/unload; blocks are the unit of
# *fetching*. One block request serves BLOCK_TILES^2 tiles, which matters twice
# over:
#
# * Overpass. A 3 km flight touches ~100 tiles; a hundred round trips against a
#   public instance that is frequently returning 504 is both slow and rude. At
#   BLOCK_TILES=4 it is seven or eight requests. Set BLOCK_TILES=1 for the
#   literal per-tile behaviour.
# * The DEM. dem.estimate_ground is a windowed percentile filter, so neighbouring
#   tiles must see the same DEM window or their meshes disagree along the shared
#   edge. The pad is wider than the filter's reach, which makes the estimate at a
#   block border identical from either side.
BLOCK_TILES = 4
DEM_PAD_M = 512.0

# Name of the timeline the streamer backdates tiles on. It has to match what the
# caller sets per fix, or the backdated logs land on a timeline nobody replays.
FLIGHT_TIMELINE = "flight"

# Base of the per-tile retry backoff, in seconds (5, 10, 20 ...).
RETRY_BACKOFF_S = 5.0

# Samples per tile edge for the terrain mesh (~32 m at TILE_M=256, just finer
# than the 30 m DEM).
TERRAIN_SAMPLES = 9

Source = Literal["osm", "overture"]

# How the caller's clock relates to the recording's timeline. See TileStreamer.
Pacing = Literal["live", "offline"]


class TileKey(NamedTuple):
    """Integer tile coordinates in the ENU plane."""

    ix: int
    iy: int

    def __str__(self) -> str:  # entity-path friendly, and negatives stay readable
        return f"{self.ix}_{self.iy}"


def tile_of(e: float, n: float, tile_m: float = TILE_M) -> TileKey:
    """Tile containing the ENU point ``(e, n)``.

    Floor division, so the tiling is a true partition across the origin: a point
    exactly on a boundary belongs to the tile it starts, and negative coordinates
    are not folded onto positive ones.
    """
    return TileKey(math.floor(e / tile_m), math.floor(n / tile_m))


def tile_bounds(key: TileKey, tile_m: float = TILE_M) -> tuple[float, float, float, float]:
    """``(e_min, n_min, e_max, n_max)`` of a tile, in ENU metres."""
    return (
        key.ix * tile_m,
        key.iy * tile_m,
        (key.ix + 1) * tile_m,
        (key.iy + 1) * tile_m,
    )


def tile_center(key: TileKey, tile_m: float = TILE_M) -> tuple[float, float]:
    """ENU centre of a tile."""
    return ((key.ix + 0.5) * tile_m, (key.iy + 0.5) * tile_m)


def tiles_within(e: float, n: float, radius_m: float, tile_m: float = TILE_M) -> list[TileKey]:
    """Tiles whose centre lies within ``radius_m`` of ``(e, n)``, nearest first."""
    reach = math.ceil(radius_m / tile_m) + 1
    here = tile_of(e, n, tile_m)
    out: list[tuple[float, TileKey]] = []
    for dx in range(-reach, reach + 1):
        for dy in range(-reach, reach + 1):
            key = TileKey(here.ix + dx, here.iy + dy)
            cx, cy = tile_center(key, tile_m)
            d = math.hypot(cx - e, cy - n)
            if d <= radius_m:
                out.append((d, key))
    out.sort()
    return [key for _, key in out]


def enu_bbox_deg(
    frame: EnuFrame, e_min: float, n_min: float, e_max: float, n_max: float
) -> tuple[float, float, float, float]:
    """Lon/lat bbox covering an ENU rectangle.

    The rectangle maps to a slightly rotated quad on the ellipsoid; taking the
    extent of its four corners gives a bbox that contains it, which is what the
    fetchers (intersection tests) need.
    """
    corners = np.array(
        [
            [e_min, n_min, 0.0],
            [e_max, n_min, 0.0],
            [e_min, n_max, 0.0],
            [e_max, n_max, 0.0],
        ]
    )
    lat, lon, _ = frame.enu_to_geodetic(corners)
    return (float(lon.min()), float(lat.min()), float(lon.max()), float(lat.max()))


def tile_bbox_deg(
    frame: EnuFrame, key: TileKey, tile_m: float = TILE_M, pad_m: float = 0.0
) -> tuple[float, float, float, float]:
    """Lon/lat bbox for one tile, optionally padded outwards."""
    e0, n0, e1, n1 = tile_bounds(key, tile_m)
    return enu_bbox_deg(frame, e0 - pad_m, n0 - pad_m, e1 + pad_m, n1 + pad_m)


def partition_by_tile(
    buildings: Sequence[Building], frame: EnuFrame, tile_m: float = TILE_M
) -> dict[TileKey, list[Building]]:
    """Assign each building to the tile containing its centroid.

    Every building lands in exactly one tile, so neighbouring tiles never
    double-extrude a footprint that crosses their shared border, and none is
    dropped. Done once per fetched block rather than once per tile — the
    centroids of a block's few thousand footprints are not worth recomputing
    sixteen times.
    """
    out: dict[TileKey, list[Building]] = {}
    if not buildings:
        return out
    cents = np.array([[b.geometry.centroid.x, b.geometry.centroid.y] for b in buildings])
    enu = frame.geodetic_to_enu(cents[:, 1], cents[:, 0], 0.0, datum="ellipsoidal")
    ix = np.floor(enu[:, 0] / tile_m).astype(int)
    iy = np.floor(enu[:, 1] / tile_m).astype(int)
    for building, x, y in zip(buildings, ix, iy, strict=True):
        out.setdefault(TileKey(int(x), int(y)), []).append(building)
    return out


def buildings_in_tile(
    buildings: Sequence[Building],
    frame: EnuFrame,
    key: TileKey,
    tile_m: float = TILE_M,
) -> list[Building]:
    """The subset of ``buildings`` this tile owns, by centroid membership."""
    return partition_by_tile(buildings, frame, tile_m).get(key, [])


# Load/unload policy


@dataclass
class TilePolicy:
    """Decides which tiles should be live, with hysteresis.

    Two radii, not one: a tile is requested when its centre comes within
    ``load_radius_m`` and dropped only past ``unload_radius_m``. A vehicle
    loitering on a single threshold would otherwise load and unload the same tile
    on every fix.

    Pure bookkeeping — no fetching, no logging — so the behaviour that matters
    (never thrashing, never dropping a tile still in range) can be tested against
    a synthetic path.
    """

    tile_m: float = TILE_M
    load_radius_m: float = LOAD_RADIUS_M
    unload_radius_m: float = UNLOAD_RADIUS_M
    live: set[TileKey] = field(default_factory=set)

    def __post_init__(self) -> None:
        if self.unload_radius_m <= self.load_radius_m:
            raise ValueError(
                "unload_radius_m must exceed load_radius_m, otherwise tiles thrash "
                f"at the boundary (got {self.unload_radius_m} <= {self.load_radius_m})"
            )

    def wanted(self, e: float, n: float) -> list[TileKey]:
        """Tiles that should be loaded at this position, nearest first."""
        return tiles_within(e, n, self.load_radius_m, self.tile_m)

    def distance(self, key: TileKey, e: float, n: float) -> float:
        cx, cy = tile_center(key, self.tile_m)
        return math.hypot(cx - e, cy - n)

    def step(self, e: float, n: float) -> tuple[list[TileKey], list[TileKey]]:
        """Advance to a new position.

        Returns ``(to_load, to_unload)``: tiles newly in range (nearest first),
        and live tiles that have fallen outside the unload radius. Marking a tile
        live is the caller's job — call :meth:`mark_live` once it is actually
        logged, so a tile still being fetched is not mistaken for a loaded one.
        """
        return self.step_many([(e, n)])

    def step_many(self, foci: Sequence[tuple[float, float]]) -> tuple[list[TileKey], list[TileKey]]:
        """:meth:`step` over several points of interest at once.

        The wanted set is the union of each focus's disk; a live tile unloads
        only once it is beyond the unload radius of *every* focus. This is how
        "we care about these coordinates" pluralizes: extra foci are just more
        disks, with no per-focus bookkeeping.
        """
        wanted: list[TileKey] = []
        seen: set[TileKey] = set()
        for e, n in foci:
            for key in self.wanted(e, n):
                if key not in seen:
                    seen.add(key)
                    wanted.append(key)
        to_load = [k for k in wanted if k not in self.live]
        to_unload = [
            k
            for k in self.live
            if all(self.distance(k, e, n) > self.unload_radius_m for e, n in foci)
        ]
        return to_load, to_unload

    def mark_live(self, key: TileKey) -> None:
        self.live.add(key)

    def mark_dropped(self, key: TileKey) -> None:
        self.live.discard(key)


# Tile content


@dataclass
class TileData:
    """Everything a tile contributes to the scene, as plain geometry.

    Built on a worker thread and logged on the caller's thread, so this holds no
    rerun objects.
    """

    key: TileKey
    buildings: Mesh | None = None
    terrain: Mesh | None = None
    # One entry per enabled grid layer: the layer's style, and its strips inside
    # this tile.
    grid: list[tuple[GridLayer, list[np.ndarray]]] = field(default_factory=list)
    n_buildings: int = 0

    @property
    def is_empty(self) -> bool:
        return self.buildings is None and self.terrain is None


class TileSink(Protocol):
    """Where the streamer's scene events land.

    The rerun implementation is :class:`~dimos.visualization.citymesh.viz.RerunTileSink`;
    a different renderer implements these same events against its own scene API.
    ``at_t``/``now_t`` carry the backdating contract: when ``at_t`` is not None the
    tile belongs at that (earlier) timeline position, and the sink must return the
    timeline to ``now_t`` afterwards.
    """

    @property
    def wants_carpet(self) -> bool: ...

    def tile(self, data: TileData, at_t: float | None, now_t: float | None) -> None: ...

    def clear(self, key: TileKey) -> None: ...

    def dead(self, key: TileKey, bounds: tuple[float, float, float, float]) -> None: ...

    def carpet(self, e: float, n: float, terrain: Terrain | None) -> bool: ...


T = TypeVar("T")


class _BlockCache(Generic[T]):
    """Compute-once cache keyed by block index, safe under the worker pool.

    Two workers reaching for the same block must not both fetch it — that is a
    duplicated Overpass query and a duplicated DEM read. A per-key lock lets
    different blocks still be fetched in parallel.

    Failures are remembered too, for ``negative_ttl_s``. Without that, one dead
    block would be re-fetched once per tile it contains: sixteen 60-second
    timeouts for one outage. The TTL is deliberately shorter than the streamer's
    retry backoff, so a genuine retry always re-attempts the network.
    """

    def __init__(
        self,
        compute: Callable[[tuple[int, int]], T],
        max_items: int = 12,
        negative_ttl_s: float = 4.0,
    ) -> None:
        self._compute = compute
        self._max_items = max_items
        self._negative_ttl_s = negative_ttl_s
        self._items: OrderedDict[tuple[int, int], T] = OrderedDict()
        self._failures: dict[tuple[int, int], tuple[float, BaseException]] = {}
        self._locks: dict[tuple[int, int], threading.Lock] = {}
        self._guard = threading.Lock()

    def _cached(self, block: tuple[int, int]) -> T | None:
        """Return the value if present, raise a remembered failure, else None."""
        if block in self._items:
            self._items.move_to_end(block)
            return self._items[block]
        failed = self._failures.get(block)
        if failed is not None:
            expires, exc = failed
            if time.monotonic() < expires:
                raise exc
            del self._failures[block]
        return None

    def peek(self, block: tuple[int, int]) -> T | None:
        """The cached value, or None — never computes, never blocks.

        For callers on the logging thread (the micro carpet), where waiting on a
        DEM fetch would stall the pose stream.
        """
        with self._guard:
            return self._items.get(block)

    def get(self, block: tuple[int, int]) -> T:
        with self._guard:
            hit = self._cached(block)
            if hit is not None:
                return hit
            lock = self._locks.setdefault(block, threading.Lock())

        with lock:
            with self._guard:
                hit = self._cached(block)
                if hit is not None:
                    return hit
            try:
                value = self._compute(block)
            except BaseException as exc:
                with self._guard:
                    self._failures[block] = (
                        time.monotonic() + self._negative_ttl_s,
                        exc,
                    )
                raise
            with self._guard:
                self._items[block] = value
                self._items.move_to_end(block)
                self._failures.pop(block, None)
                while len(self._items) > self._max_items:
                    dropped, _ = self._items.popitem(last=False)
                    self._locks.pop(dropped, None)
            return value


class TileBuilder:
    """Fetches and meshes one tile. Safe to call from a worker thread."""

    def __init__(
        self,
        frame: EnuFrame,
        theme: Theme = THEMES["day"],
        source: Source = "osm",
        tile_m: float = TILE_M,
        flat_ground: bool = False,
        cache: bool = True,
        release: str | None = None,
        terrain_samples: int = TERRAIN_SAMPLES,
        block_tiles: int = BLOCK_TILES,
    ) -> None:
        self.frame = frame
        self.theme = theme
        self.source = source
        self.tile_m = tile_m
        self.flat_ground = flat_ground
        self.cache = cache
        self.release = release
        self.terrain_samples = terrain_samples
        self.block_tiles = max(1, int(block_tiles))
        self._dem = _BlockCache(self._fetch_dem_block)
        self._buildings = _BlockCache(self._fetch_building_block)
        self._z_range: tuple[float, float] | None = None
        self._z_lock = threading.Lock()

    # -- block geometry ---------------------------------------------------

    def block_of(self, key: TileKey) -> tuple[int, int]:
        return (
            math.floor(key.ix / self.block_tiles),
            math.floor(key.iy / self.block_tiles),
        )

    def block_bounds(self, block: tuple[int, int]) -> tuple[float, float, float, float]:
        bx, by = block
        span = self.block_tiles * self.tile_m
        return (bx * span, by * span, (bx + 1) * span, (by + 1) * span)

    # -- terrain ----------------------------------------------------------

    def _fetch_dem_block(self, block: tuple[int, int]) -> Terrain:
        e0, n0, e1, n1 = self.block_bounds(block)
        bbox = enu_bbox_deg(
            self.frame, e0 - DEM_PAD_M, n0 - DEM_PAD_M, e1 + DEM_PAD_M, n1 + DEM_PAD_M
        )
        return fetch_terrain(bbox, cache=self.cache)

    def peek_dem(self, key: TileKey) -> Terrain | None:
        """The DEM block under a tile if it is already in memory, else None."""
        return self._dem.peek(self.block_of(key))

    def _terrain_z_range(self, block_terrain: Terrain) -> tuple[float, float]:
        """Elevation range for terrain colouring, fixed after the first block.

        A per-tile min/max would stretch the theme's ramp differently in every
        tile and the ground would come out patchwork.
        """
        with self._z_lock:
            if self._z_range is None:
                origin = self.frame.origin_msl
                self._z_range = (
                    float(block_terrain.ground_msl.min()) - origin,
                    float(block_terrain.ground_msl.max()) - origin,
                )
            return self._z_range

    # -- buildings --------------------------------------------------------

    def _fetch_building_block(self, block: tuple[int, int]) -> dict[TileKey, list[Building]]:
        """One source request per block, split into per-tile ownership up front."""
        e0, n0, e1, n1 = self.block_bounds(block)
        bbox = enu_bbox_deg(self.frame, e0, n0, e1, n1)
        if self.source == "osm":
            found = fetch_buildings_osm(bbox, cache=self.cache)
        else:
            found = fetch_buildings(bbox, release=self.release, cache=self.cache)
        return partition_by_tile(found, self.frame, self.tile_m)

    # -- the whole tile ---------------------------------------------------

    def build(self, key: TileKey) -> TileData:
        block = self.block_of(key)

        terrain_block: Terrain | None = None
        terrain_patch: Mesh | None = None
        if not self.flat_ground:
            terrain_block = self._dem.get(block)
            terrain_patch = terrain_patch_mesh(
                terrain_block,
                self.frame,
                tile_bounds(key, self.tile_m),
                samples=self.terrain_samples,
                theme=self.theme,
                z_range=self._terrain_z_range(terrain_block),
            )

        # A block request returns everything intersecting it, already split by
        # ownership; this tile takes its share (often none — parks, water).
        owned = self._buildings.get(block).get(key, [])
        buildings_mesh = None
        if owned:
            try:
                buildings_mesh = extrude_buildings(
                    owned, self.frame, terrain=terrain_block, theme=self.theme
                )
            except ValueError as exc:
                # Every footprint here was degenerate (zero thickness, slivers).
                # An empty tile is normal — parks, water, the edge of town — and
                # must never take the flight down with it.
                log.debug("tile %s produced no extrudable geometry: %s", key, exc)

        # Grid lines are placed at multiples of their spacing in ENU, cut to this
        # tile's bounds — never laid out from the tile's own corner, or every tile
        # would start its own phase and the lines would step at each border.
        grid: list[tuple[GridLayer, list[np.ndarray]]] = []
        for layer in self.theme.grid_layers():
            strips = grid_strips(
                self.frame,
                tile_bounds(key, self.tile_m),
                layer.spacing_m,
                terrain=terrain_block,
            )
            if strips:
                grid.append((layer, strips))

        return TileData(
            key=key,
            buildings=buildings_mesh,
            terrain=terrain_patch,
            grid=grid,
            n_buildings=len(owned),
        )


# Streamer


@dataclass
class StreamStats:
    loaded: int = 0
    unloaded: int = 0
    reloaded_from_memory: int = 0
    # Tiles whose build finished after the vehicle had already left them:
    # backdated into the interval it was there and cleared again (offline pacing),
    # or discarded (live pacing, where that interval is already in the past).
    late: int = 0
    dropped: int = 0
    failed: int = 0
    dead: int = 0
    buildings: int = 0
    carpet_moves: int = 0


class TileStreamer:
    """Keeps the world loaded around a moving vehicle.

    ``update(enu, t_s)`` is called once per fix and never blocks on the network:
    fetching happens on a small thread pool, finished tiles queue up, and each
    ``update`` logs whatever has arrived.

    **Pacing** is the difference between the two clocks in play: the recording's
    timeline, which comes from the route, and the wall clock the workers actually
    run on.

    * ``"live"`` — they are the same clock. A tile logged when it finishes is
      logged at the moment it finished, which is the truth; one that finished
      after the vehicle left is stale and dropped.
    * ``"offline"`` — the recording is *generated* from a route, and generation
      runs far faster than the flight it describes (the 3 km demo is 200 s of
      flight in a few seconds of wall time, ~50x). A tile taking half a second to
      build would then be logged tens of flight-seconds after the vehicle asked
      for it: on 1x replay the vehicle flies through empty world and tiles pop in
      far behind it. So each tile is logged *backdated* to the flight time at
      which it was enqueued, which is where it belongs and where it would have
      appeared in the field. Rerun takes out-of-order logs without complaint — the
      index is explicit, not implied by call order.

    Backdating is a statement about how the recording was made, not a fudge: it
    is only correct when the timeline is a replayed route. Live flight must use
    ``"live"``, where wall time *is* flight time.

    Unloading emits :meth:`TileSink.clear` rather than deleting anything, so
    the recording stays replayable: a tile exists at the times it was live and is
    cleared from the moment the vehicle left it. A tile that arrives after the
    vehicle has already gone gets both at once — backdated in, cleared at the
    present — so replay shows it for exactly the interval the vehicle was there.
    """

    def __init__(
        self,
        frame: EnuFrame,
        sink: TileSink,
        theme: Theme = THEMES["day"],
        source: Source = "osm",
        tile_m: float = TILE_M,
        load_radius_m: float = LOAD_RADIUS_M,
        unload_radius_m: float = UNLOAD_RADIUS_M,
        workers: int = 2,
        flat_ground: bool = False,
        cache: bool = True,
        release: str | None = None,
        max_attempts: int = 3,
        memory_tiles: int = 96,
        block_tiles: int = BLOCK_TILES,
        max_logs_per_update: int = 1,
        pacing: Pacing = "live",
    ) -> None:
        self.frame = frame
        self.sink = sink
        self.theme = theme
        self.pacing: Pacing = pacing
        self.max_attempts = max_attempts
        self.memory_tiles = memory_tiles
        self.flat_ground = flat_ground
        # Logging a tile is not free — a dense one is a few hundred thousand
        # triangles plus its wireframe — and it happens on the caller's thread.
        # Spending that on at most one tile per fix keeps the pose cadence even;
        # the rest wait for the next fix, a few milliseconds later.
        self.max_logs_per_update = max(1, int(max_logs_per_update))
        self.policy = TilePolicy(
            tile_m=tile_m,
            load_radius_m=load_radius_m,
            unload_radius_m=unload_radius_m,
        )
        self.builder = TileBuilder(
            frame,
            theme=theme,
            source=source,
            tile_m=tile_m,
            flat_ground=flat_ground,
            cache=cache,
            release=release,
            block_tiles=block_tiles,
        )
        self.stats = StreamStats()

        self._pool = ThreadPoolExecutor(max_workers=workers, thread_name_prefix="tile")
        self._inflight: dict[TileKey, Future[TileData]] = {}
        self._ready: OrderedDict[TileKey, TileData] = OrderedDict()
        self._attempts: dict[TileKey, int] = {}
        self._retry_after: dict[TileKey, float] = {}
        self._dead: set[TileKey] = set()
        # Tiles built earlier in the session: re-entering an area must not need
        # the network again, not even the cache round-trip.
        self._memory: OrderedDict[TileKey, TileData] = OrderedDict()
        self._where: list[tuple[float, float]] | None = None
        # The flight time each pending tile was asked for, and the flight time of
        # the latest fix. Backdating is the difference between the two.
        self._enqueued: dict[TileKey, float | None] = {}
        self._now_t: float | None = None
        self._closed = False

    # -- public API -------------------------------------------------------

    def update(
        self,
        enu: np.ndarray | Sequence[float],
        t_s: float | None = None,
        extra_foci: Sequence[tuple[float, float]] = (),
    ) -> None:
        """Advance the stream to a new vehicle position. Never blocks.

        ``t_s`` is the fix's time on the recording's timeline — the same value the
        caller passed to ``rr.set_time``. Offline pacing needs it to backdate
        tiles to the moment they were asked for; without it there is nothing to
        backdate to and tiles land wherever the timeline happens to be.

        ``extra_foci`` are additional ENU ``(e, n)`` points to keep loaded —
        static anchors the session cares about beyond the vehicle itself. The
        carpet still follows only the vehicle.
        """
        if self._closed:
            raise RuntimeError("TileStreamer is closed")
        e, n = float(enu[0]), float(enu[1])
        foci = [(e, n), *extra_foci]
        self._where = foci
        if t_s is not None:
            self._now_t = float(t_s)

        self._collect(budget=self.max_logs_per_update)

        to_load, to_unload = self.policy.step_many(foci)
        for key in to_unload:
            self._unload(key)
        for key in to_load:
            self._request(key, now=time.monotonic())

        # The carpet tracks the vehicle, not a fetch, so it belongs at the
        # current fix — never backdated.
        self._update_carpet(e, n)

    def _update_carpet(self, e: float, n: float) -> None:
        """Move the micro grid with the vehicle, if the theme has one.

        Draping needs the DEM under the vehicle, and this runs on the logging
        thread, so it *peeks* the block cache instead of fetching: no terrain yet
        means no carpet yet, for the fraction of a second before the vehicle's own
        tile lands. Never a stall in the pose stream.
        """
        if not self.sink.wants_carpet:
            return
        terrain = None
        if not self.flat_ground:
            terrain = self.builder.peek_dem(tile_of(e, n, self.policy.tile_m))
            if terrain is None:
                return
        if self.sink.carpet(e, n, terrain):
            self.stats.carpet_moves += 1

    def close(self, drain_timeout_s: float = 60.0) -> None:
        """Log any tiles still in flight, then shut the pool down.

        Waiting here (and only here) means the recording ends complete without
        ever having stalled the pose stream.
        """
        if self._closed:
            return
        deadline = time.monotonic() + drain_timeout_s
        while self._inflight and time.monotonic() < deadline:
            self._collect(block_s=0.25)
        for future in self._inflight.values():
            future.cancel()
        self._inflight.clear()
        self._flush(budget=None)
        self._pool.shutdown(wait=False, cancel_futures=True)
        self._closed = True

    def live_tiles(self) -> set[TileKey]:
        return set(self.policy.live)

    # -- internals --------------------------------------------------------

    def _request(self, key: TileKey, now: float) -> None:
        if key in self.policy.live or key in self._inflight or key in self._ready:
            return
        if key in self._dead:
            # Dead for the session, but the marker was cleared when the vehicle
            # left. Coming back should show the gap again, not silence.
            self._log_dead(key)
            return
        if now < self._retry_after.get(key, 0.0):
            return
        # Whether it comes from the network or from memory, this is the moment in
        # the flight the tile was wanted — and, under offline pacing, the moment
        # it will be logged at however long the build actually takes.
        self._enqueued[key] = self._now_t
        remembered = self._memory.get(key)
        if remembered is not None:
            # Been here before this session: no fetch, no cache read, straight
            # back into the scene on the next flush.
            self._memory.move_to_end(key)
            self._ready[key] = remembered
            self.stats.reloaded_from_memory += 1
            return
        self._inflight[key] = self._pool.submit(self.builder.build, key)

    def _collect(self, budget: int | None = None, block_s: float = 0.0) -> None:
        """Harvest finished tiles, then log up to ``budget`` of them."""
        if block_s and self._inflight:
            # Cheap wait: sleep only if nothing is ready yet.
            if not any(f.done() for f in self._inflight.values()):
                time.sleep(block_s)
        for key in [k for k, f in self._inflight.items() if f.done()]:
            future = self._inflight.pop(key)
            try:
                data = future.result()
            except Exception as exc:
                self._fail(key, exc)
                continue
            self._remember(data)
            self._ready[key] = data
        self._flush(budget)

    def _flush(self, budget: int | None) -> None:
        """Log built tiles, oldest first, each at the flight time it was asked for.

        A tile that finished after the vehicle flew a kilometre past cannot simply
        be logged as it is: nothing revisits it to unload it, so it would pop into
        existence behind the vehicle and stay there. Under offline pacing it is
        instead put back where it belongs — logged at its enqueue time and cleared
        at the present — so replay shows it for exactly the interval the vehicle
        was in range. Under live pacing that interval is genuinely in the past and
        there is nothing useful to log, so it is dropped. Either way the tile stays
        in memory, so coming back is instant.
        """
        logged = 0
        while self._ready and (budget is None or logged < budget):
            key, data = self._ready.popitem(last=False)
            enqueued = self._enqueued.pop(key, None)
            late = self._is_stale(key)
            if late and self.pacing == "live":
                self.stats.dropped += 1
                continue
            self._log_tile(data, at_t=enqueued, keep=not late)
            if late:
                # Backdated in, and gone again by now.
                self._clear(key)
                self.stats.late += 1
            logged += 1

    def _is_stale(self, key: TileKey) -> bool:
        if self._where is None:
            return False
        return all(
            self.policy.distance(key, e, n) > self.policy.unload_radius_m for e, n in self._where
        )

    def _fail(self, key: TileKey, exc: BaseException) -> None:
        attempts = self._attempts.get(key, 0) + 1
        self._attempts[key] = attempts
        self.stats.failed += 1
        if attempts >= self.max_attempts:
            self._dead.add(key)
            self.stats.dead += 1
            log.warning("tile %s failed %d times, giving up: %s", key, attempts, exc)
            self._log_dead(key)
            return
        # Exponential backoff, so a rate-limited fetcher is not hammered.
        self._retry_after[key] = time.monotonic() + RETRY_BACKOFF_S * 2.0**attempts
        log.info("tile %s failed (attempt %d), will retry: %s", key, attempts, exc)

    def _remember(self, data: TileData) -> None:
        self._memory[data.key] = data
        self._memory.move_to_end(data.key)
        while len(self._memory) > self.memory_tiles:
            self._memory.popitem(last=False)

    def _log_tile(self, data: TileData, at_t: float | None = None, keep: bool = True) -> None:
        # Backdating only exists under offline pacing; live tiles always land at
        # the present, so the sink sees at_t=None and skips the timeline dance.
        self.sink.tile(
            data,
            at_t=at_t if self.pacing == "offline" else None,
            now_t=self._now_t,
        )
        # A tile that is already being cleared again was never live: leaving it in
        # the policy's live set would stop the vehicle ever re-requesting it.
        if keep:
            self.policy.mark_live(data.key)
        self.stats.loaded += 1
        self.stats.buildings += data.n_buildings

    def _clear(self, key: TileKey) -> None:
        self.sink.clear(key)

    def _log_dead(self, key: TileKey) -> None:
        """Mark where a tile could not be fetched.

        A silent hole reads as "nothing here"; a marker reads as "this failed",
        which is the difference between a bug you notice and one you do not.
        """
        self.sink.dead(key, tile_bounds(key, self.policy.tile_m))
        self.policy.mark_live(key)

    def _unload(self, key: TileKey) -> None:
        # At the current time, always: this is the vehicle leaving, which happens
        # now, not whenever the tile was built.
        self._clear(key)
        self.policy.mark_dropped(key)
        self.stats.unloaded += 1


def format_stats(stats: StreamStats) -> str:
    return (
        f"tiles: {stats.loaded} loaded ({stats.reloaded_from_memory} from memory), "
        f"{stats.unloaded} unloaded, {stats.buildings} buildings"
        + (
            f", {stats.late} backdated in and out again (arrived after the vehicle had left)"
            if stats.late
            else ""
        )
        + (f", {stats.dropped} arrived too late" if stats.dropped else "")
        + (f", {stats.dead} unreachable" if stats.dead else "")
        + (f", carpet moved {stats.carpet_moves}x" if stats.carpet_moves else "")
    )
