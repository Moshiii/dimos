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

"""Offline tests for the CityMesh engine: fixes in, EntityMesh messages out.

No network, no rerun, no module runtime: the OSM fetcher is stubbed, and the
engine publishes into a plain list.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.visualization_msgs.EntityMesh import EntityMesh
from dimos.visualization.citymesh import tiles as tiles_mod
from dimos.visualization.citymesh.extrude import Mesh
from dimos.visualization.citymesh.frame import snap_origin
from dimos.visualization.citymesh.module import CityStream, MeshPublisher
from dimos.visualization.citymesh.tiles import TileData, TileKey

ATHENS = (37.9838, 23.7275)


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    monkeypatch.setattr(tiles_mod, "fetch_buildings_osm", lambda bbox, cache=True, **kw: [])


def _fix(lat=ATHENS[0], lon=ATHENS[1], alt=100.0, ts=1000.0):
    return NavSatFix(latitude=lat, longitude=lon, altitude=alt, status=NavSatFix.STATUS_FIX, ts=ts)


def _tile_mesh() -> Mesh:
    return Mesh(
        vertices=np.zeros((3, 3), dtype=np.float32),
        triangles=np.array([[0, 1, 2]], dtype=np.uint32),
        colors=np.tile(np.array([[10, 20, 30, 120]], dtype=np.uint8), (3, 1)),
    )


class _StubBuilder:
    """Instant, offline, and always yields one terrain and one buildings mesh."""

    def build(self, key: TileKey) -> TileData:
        return TileData(key=key, terrain=_tile_mesh(), buildings=_tile_mesh(), n_buildings=1)


# -- MeshPublisher: tile events to messages --------------------------------


def test_tile_becomes_two_entity_meshes():
    published: list[EntityMesh] = []
    sink = MeshPublisher(published.append, root="world/city", frame_id="enu")
    data = TileData(key=TileKey(3, -4), terrain=_tile_mesh(), buildings=_tile_mesh())
    sink.tile(data, at_t=None, now_t=42.0)

    assert [m.path for m in published] == [
        "world/city/tiles/3_-4/terrain",
        "world/city/tiles/3_-4/buildings",
    ]
    assert all(m.frame_id == "enu" for m in published)
    assert all(m.op == "set" for m in published)
    assert all(m.ts == 42.0 for m in published), "messages belong at the fix that asked"


def test_empty_tile_publishes_nothing():
    published: list[EntityMesh] = []
    sink = MeshPublisher(published.append, root="world/city", frame_id="enu")
    sink.tile(TileData(key=TileKey(0, 0)), at_t=None, now_t=1.0)
    assert published == []


def test_clear_publishes_a_recursive_clear():
    published: list[EntityMesh] = []
    sink = MeshPublisher(published.append, root="world/city", frame_id="enu")
    sink.clear(TileKey(1, 2))
    (msg,) = published
    assert msg.op == "clear"
    assert msg.path == "world/city/tiles/1_2"


# -- CityStream: anchoring and streaming -----------------------------------


def test_no_fix_starts_nothing():
    stream = CityStream(lambda m: None, flat_ground=True)
    stream.on_fix(NavSatFix(status=NavSatFix.STATUS_NO_FIX))
    assert stream.streamer is None


def test_first_fix_anchors_at_the_snapped_origin():
    stream = CityStream(lambda m: None, flat_ground=True)
    try:
        stream.on_fix(_fix())
        assert stream.streamer is not None
        assert (stream.frame.lat0, stream.frame.lon0) == snap_origin(*ATHENS)
        assert stream.frame.origin_msl == 0.0
    finally:
        stream.close()


def test_nan_altitude_does_not_poison_the_stream():
    stream = CityStream(lambda m: None, flat_ground=True)
    try:
        stream.on_fix(_fix(alt=float("nan")))
        assert stream.streamer is not None
    finally:
        stream.close()


def test_fixes_stream_tiles_and_walking_away_clears_them():
    published: list[EntityMesh] = []
    stream = CityStream(
        published.append, flat_ground=True, load_radius_m=300.0, unload_radius_m=450.0
    )
    try:
        stream.on_fix(_fix(ts=0.0))
        assert stream.streamer is not None
        stream.streamer.builder = _StubBuilder()

        lat0, lon0 = ATHENS
        for i in range(1, 40):
            stream.on_fix(_fix(lat=lat0, lon=lon0 + i * 0.0005, ts=float(i)))
            stream.streamer._collect(block_s=0.05)

        sets = [m for m in published if m.op == "set"]
        clears = [m for m in published if m.op == "clear"]
        assert sets, "tiles should have streamed in"
        assert all(m.path.startswith("world/city/tiles/") for m in sets)
        assert all(m.frame_id == "enu" for m in sets)
        assert clears, "the traverse left tiles behind; they must be cleared"
    finally:
        stream.close()
