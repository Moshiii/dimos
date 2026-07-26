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

"""Offline tests for the streamer itself: empty tiles, failures, load/unload.

No network and no viewer — the fetchers are stubbed and rerun is initialised
without a sink, so ``rr.log`` calls are accepted and discarded.
"""

from __future__ import annotations

import numpy as np
import pytest
import rerun as rr
from shapely.geometry import Polygon

from dimos.visualization.citymesh import tiles as tiles_mod
from dimos.visualization.citymesh.extrude import grid_line_positions
from dimos.visualization.citymesh.frame import EnuFrame
from dimos.visualization.citymesh.overture import Building
from dimos.visualization.citymesh.themes import THEMES
from dimos.visualization.citymesh.tiles import TileBuilder, TileData, TileKey, TileStreamer
from dimos.visualization.citymesh.viz import RerunTileSink


@pytest.fixture
def frame() -> EnuFrame:
    return EnuFrame.at(37.9938, 23.7253, 0.0, datum="ellipsoidal", undulation=0.0)


@pytest.fixture(autouse=True)
def recording():
    """A recording stream with no sink, so logging is a no-op."""
    rr.init("citymesh-tests")


def test_empty_tile_is_not_an_error(frame, monkeypatch):
    """A tile over a park or the sea yields nothing, and that is fine.

    ``extrude_buildings`` raises on an empty list; a streamer that let that
    propagate would die the first time the drone crossed water.
    """
    monkeypatch.setattr(tiles_mod, "fetch_buildings_osm", lambda bbox, cache=True, **kw: [])
    builder = TileBuilder(frame, flat_ground=True)
    data = builder.build(TileKey(0, 0))
    assert data.n_buildings == 0
    assert data.buildings is None
    assert data.is_empty


def test_tile_of_only_degenerate_footprints_is_not_an_error(frame, monkeypatch):
    """Zero-height footprints make extrude_buildings raise; the tile survives."""
    flat = Building(
        id="flat",
        geometry=Polygon(
            [(23.7253, 37.9938), (23.7254, 37.9938), (23.7254, 37.9939), (23.7253, 37.9939)]
        ),
        height_m=0.0,
        min_height_m=0.0,
        height_is_estimated=True,
        name=None,
        building_class=None,
    )
    monkeypatch.setattr(tiles_mod, "fetch_buildings_osm", lambda bbox, cache=True, **kw: [flat])
    builder = TileBuilder(frame, flat_ground=True)
    data = builder.build(tiles_mod.tile_of(0.0, 0.0))
    assert data.n_buildings == 1
    assert data.buildings is None


def test_block_fetch_is_shared_by_the_tiles_it_covers(frame, monkeypatch):
    """One source request must serve every tile in its block."""
    calls: list[tuple] = []

    def counting_fetch(bbox, cache=True, **kw):
        calls.append(bbox)
        return []

    monkeypatch.setattr(tiles_mod, "fetch_buildings_osm", counting_fetch)
    builder = TileBuilder(frame, flat_ground=True, block_tiles=4)
    for ix in range(4):
        for iy in range(4):
            builder.build(TileKey(ix, iy))
    assert len(calls) == 1

    builder.build(TileKey(4, 0))  # next block over
    assert len(calls) == 2


def test_tile_grids_continue_across_a_shared_border(frame, monkeypatch):
    """Two tiles built independently must produce one continuous lattice."""
    monkeypatch.setattr(tiles_mod, "fetch_buildings_osm", lambda bbox, cache=True, **kw: [])
    builder = TileBuilder(frame, theme=THEMES["blueprint"], flat_ground=True)
    left = builder.build(TileKey(0, 0))
    right = builder.build(TileKey(1, 0))

    assert [layer.name for layer, _ in left.grid] == ["major", "minor"]
    for (layer, a), (_, b) in zip(left.grid, right.grid, strict=True):
        # Between them the two tiles must draw exactly the lines a single
        # 512 m-wide patch would, each of them once.
        joined = _ns_line_positions(a) + _ns_line_positions(b)
        assert len(joined) == len(set(joined)), "a line was drawn by both tiles"
        whole = grid_line_positions(0.0, 2 * tiles_mod.TILE_M, layer.spacing_m)
        assert sorted(joined) == pytest.approx(whole.tolist())


def _ns_line_positions(strips: list[np.ndarray]) -> list[float]:
    """East coordinates of the north-south lines in a strip list."""
    return [round(float(s[:, 0].mean()), 3) for s in strips if float(np.ptp(s[:, 0])) < 0.01]


def test_carpet_follows_the_vehicle_without_relogging_every_fix(frame):
    """The micro grid moves in half-carpet jumps, not once per fix."""
    sink = RerunTileSink(frame, theme=THEMES["blueprint"])
    streamer = TileStreamer(
        frame,
        sink,
        theme=THEMES["blueprint"],
        flat_ground=True,
        load_radius_m=300.0,
        unload_radius_m=450.0,
    )
    streamer.builder = _StubBuilder()

    fixes = list(range(0, 400, 10))
    for e in fixes:
        streamer.update([float(e), 0.0, 60.0])
    streamer.close(drain_timeout_s=5.0)

    # 390 m of travel, 100 m carpet: the first one plus a move every 60 m.
    assert streamer.stats.carpet_moves == 7
    assert streamer.stats.carpet_moves < len(fixes) / 4
    assert sink.micro.n_strips == 200, "a 100 m carpet at 1 m spacing"
    assert abs(sink.micro.anchor[0] - 390.0) <= 50.0, "the drone is on it"


def test_streamer_loads_then_unloads_along_a_traverse(frame):
    """End to end through the queue: tiles appear ahead and are cleared behind."""
    streamer = TileStreamer(frame, RerunTileSink(frame), load_radius_m=400.0, unload_radius_m=600.0)
    streamer.builder = _StubBuilder()

    for e in range(0, 2400, 100):
        streamer.update([float(e), 0.0, 60.0])
    streamer.close(drain_timeout_s=10.0)

    assert streamer.stats.loaded > 0
    assert streamer.stats.unloaded > 0
    assert streamer.stats.dead == 0
    # Tiles at the start of the traverse were dropped; the live set is bounded.
    assert TileKey(0, 0) not in streamer.live_tiles()
    assert len(streamer.live_tiles()) < 30


def test_revisited_tiles_come_from_memory_not_the_fetcher(frame):
    """Flying back over an unloaded area must not re-fetch it."""
    stub = _StubBuilder()
    streamer = TileStreamer(frame, RerunTileSink(frame), load_radius_m=300.0, unload_radius_m=450.0)
    streamer.builder = stub

    for e in list(range(0, 1600, 100)) + list(range(1600, -100, -100)):
        streamer.update([float(e), 0.0, 60.0])
        streamer._collect(block_s=0.05)
    streamer.close(drain_timeout_s=10.0)

    assert streamer.stats.reloaded_from_memory > 0
    assert stub.built == len(set(stub.keys)), "a tile was built twice"


def test_failing_tile_dies_after_max_attempts(frame):
    streamer = TileStreamer(
        frame,
        RerunTileSink(frame),
        load_radius_m=200.0,
        unload_radius_m=400.0,
        max_attempts=2,
    )
    streamer.builder = _ExplodingBuilder()
    # Backoff would otherwise stall the test for seconds.
    streamer._retry_after = _NoBackoff()

    for _ in range(40):
        streamer.update([0.0, 0.0, 60.0])
        streamer._collect(block_s=0.02)
    streamer.close(drain_timeout_s=5.0)

    assert streamer.stats.dead > 0
    assert streamer.stats.failed >= streamer.stats.dead * 2


class _StubBuilder:
    """Stands in for TileBuilder: instant, offline, and counts its calls."""

    def __init__(self) -> None:
        self.built = 0
        self.keys: list[TileKey] = []

    def build(self, key: TileKey) -> TileData:
        self.built += 1
        self.keys.append(key)
        return TileData(key=key, n_buildings=0)


class _ExplodingBuilder:
    def build(self, key: TileKey) -> TileData:
        raise RuntimeError("all Overpass instances failed")


class _NoBackoff(dict):
    """A retry-schedule that is always ready."""

    def get(self, key, default=0.0):
        return 0.0


def test_theme_alpha_reaches_the_material_not_just_vertices(frame, monkeypatch):
    """The viewer's translucent pipeline keys off albedo_factor alpha; vertex
    alpha alone renders solid. log_mesh must move the theme alpha across."""
    from dimos.visualization.citymesh.extrude import Mesh
    from dimos.visualization.citymesh.viz import log_mesh

    logged = {}

    def capture(path, archetype, **kw):
        logged[path] = archetype

    monkeypatch.setattr(rr, "log", capture)
    colors = np.tile(np.array([[20, 40, 80, 120]], dtype=np.uint8), (3, 1))
    mesh = Mesh(
        vertices=np.zeros((3, 3), dtype=np.float32),
        triangles=np.array([[0, 1, 2]], dtype=np.uint32),
        colors=colors,
    )
    log_mesh("x", mesh)
    arch = logged["x"]
    assert arch.albedo_factor is not None, "translucency must ride the material"
