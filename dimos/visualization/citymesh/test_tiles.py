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

"""Offline tests for the streaming machinery.

Everything here runs on fixture geometry with no network: the tile algebra, the
load/unload policy, and the centroid ownership rule are pure functions of
numbers, and they are exactly the parts whose bugs would be invisible in a
screenshot (a tile fetched twice, a building drawn twice, a boundary that
thrashes).

The frame is built with an explicit undulation so ``EnuFrame.at`` never reaches
for the EGM2008 grid.
"""

from __future__ import annotations

import math

import pytest
from shapely.geometry import Polygon

from dimos.visualization.citymesh.frame import EnuFrame
from dimos.visualization.citymesh.overture import Building
from dimos.visualization.citymesh.tiles import (
    TileKey,
    TilePolicy,
    buildings_in_tile,
    tile_bounds,
    tile_center,
    tile_of,
    tiles_within,
)

TILE = 256.0


@pytest.fixture
def frame() -> EnuFrame:
    return EnuFrame.at(37.9938, 23.7253, 0.0, datum="ellipsoidal", undulation=0.0)


# -- tile indexing ---------------------------------------------------------


def test_tile_of_is_floor_not_truncation():
    """Negative coordinates must not fold onto the positive tiles."""
    assert tile_of(0.0, 0.0, TILE) == TileKey(0, 0)
    assert tile_of(255.9, 255.9, TILE) == TileKey(0, 0)
    assert tile_of(256.0, 0.0, TILE) == TileKey(1, 0)
    assert tile_of(-0.1, -0.1, TILE) == TileKey(-1, -1)
    assert tile_of(-256.0, -256.0, TILE) == TileKey(-1, -1)
    assert tile_of(-256.1, 0.0, TILE) == TileKey(-2, 0)


def test_tile_boundary_belongs_to_exactly_one_tile():
    """A point on a shared edge is owned by the tile it starts, never both."""
    for e in (-512.0, -256.0, 0.0, 256.0, 512.0):
        key = tile_of(e, 0.0, TILE)
        e_min, _, e_max, _ = tile_bounds(key, TILE)
        assert e_min <= e < e_max


def test_tile_bounds_and_center_agree():
    for key in (TileKey(0, 0), TileKey(3, -2), TileKey(-7, 11)):
        e0, n0, e1, n1 = tile_bounds(key, TILE)
        assert (e1 - e0, n1 - n0) == (TILE, TILE)
        assert tile_center(key, TILE) == ((e0 + e1) / 2, (n0 + n1) / 2)
        assert tile_of(*tile_center(key, TILE), TILE) == key


def test_tiles_within_is_a_disc_nearest_first():
    # Off-centre inside its tile, so "nearest" is unambiguous — at an exact tile
    # corner the four neighbours are equidistant.
    e, n = 140.0, 130.0
    keys = tiles_within(e, n, 600.0, TILE)
    assert len(keys) == len(set(keys)), "no tile may be requested twice"
    assert keys[0] == tile_of(e, n, TILE), "the vehicle's own tile comes first"

    def dist(key: TileKey) -> float:
        cx, cy = tile_center(key, TILE)
        return math.hypot(cx - e, cy - n)

    dists = [dist(k) for k in keys]
    assert dists == sorted(dists), "nearest tiles must be requested first"
    assert max(dists) <= 600.0

    # Nothing inside the radius may be missed: check against a brute-force sweep.
    brute = {
        TileKey(ix, iy)
        for ix in range(-6, 7)
        for iy in range(-6, 7)
        if dist(TileKey(ix, iy)) <= 600.0
    }
    assert set(keys) == brute


# -- hysteresis ------------------------------------------------------------


def test_policy_rejects_inverted_radii():
    with pytest.raises(ValueError):
        TilePolicy(load_radius_m=900.0, unload_radius_m=600.0)


def test_tile_loads_inside_load_radius_and_survives_the_gap():
    """A tile between the two radii is neither re-requested nor dropped."""
    policy = TilePolicy(tile_m=TILE, load_radius_m=600.0, unload_radius_m=900.0)
    target = TileKey(0, 0)
    cx, cy = tile_center(target, TILE)

    # Approach until the tile is in range, then load it.
    to_load, _ = policy.step(cx + 500.0, cy)
    assert target in to_load
    for key in to_load:
        policy.mark_live(key)

    # Sitting in the hysteresis band: still live, and not requested again.
    to_load, to_unload = policy.step(cx + 750.0, cy)
    assert target not in to_load
    assert target not in to_unload
    assert target in policy.live

    # Past the unload radius it goes.
    _, to_unload = policy.step(cx + 950.0, cy)
    assert target in to_unload


def test_loitering_on_the_load_boundary_does_not_thrash():
    """Crossing the load radius back and forth must not churn a single tile."""
    policy = TilePolicy(tile_m=TILE, load_radius_m=600.0, unload_radius_m=900.0)
    target = TileKey(4, 0)
    cx, cy = tile_center(target, TILE)

    loads = unloads = 0
    for i in range(20):
        # Oscillate a metre either side of the load threshold.
        offset = 599.0 if i % 2 == 0 else 601.0
        to_load, to_unload = policy.step(cx + offset, cy)
        loads += target in to_load
        unloads += target in to_unload
        for key in to_load:
            policy.mark_live(key)
        for key in to_unload:
            policy.mark_dropped(key)

    assert loads == 1, "the tile should be requested once, not once per crossing"
    assert unloads == 0


def test_full_traverse_loads_ahead_and_unloads_behind():
    """Flying in a straight line: the live set moves with the vehicle."""
    policy = TilePolicy(tile_m=TILE, load_radius_m=600.0, unload_radius_m=900.0)
    seen: list[set[TileKey]] = []
    requested: list[TileKey] = []

    for step in range(0, 3000, 50):
        to_load, to_unload = policy.step(float(step), 0.0)
        requested.extend(to_load)
        for key in to_load:
            policy.mark_live(key)
        for key in to_unload:
            policy.mark_dropped(key)
        seen.append(set(policy.live))

    assert len(requested) == len(set(requested)), "no tile fetched twice"
    # The live set stays bounded — the whole point of unloading.
    assert max(len(s) for s in seen) < 40
    # Tiles behind the start are gone by the end; tiles ahead are present.
    assert TileKey(0, 0) not in seen[-1]
    assert tile_of(2950.0, 0.0, TILE) in seen[-1]


def test_live_set_covers_the_load_radius_after_settling():
    policy = TilePolicy(tile_m=TILE, load_radius_m=600.0, unload_radius_m=900.0)
    to_load, _ = policy.step(1000.0, -400.0)
    for key in to_load:
        policy.mark_live(key)
    assert set(policy.wanted(1000.0, -400.0)) <= policy.live


# -- centroid membership ---------------------------------------------------


def _building(frame: EnuFrame, e: float, n: float, size_m: float, bid: str) -> Building:
    """A square footprint centred on an ENU point, expressed in lon/lat."""
    import numpy as np

    half = size_m / 2.0
    corners = np.array(
        [
            [e - half, n - half, 0.0],
            [e + half, n - half, 0.0],
            [e + half, n + half, 0.0],
            [e - half, n + half, 0.0],
        ]
    )
    lat, lon, _ = frame.enu_to_geodetic(corners)
    return Building(
        id=bid,
        geometry=Polygon(zip(lon, lat, strict=True)),
        height_m=12.0,
        min_height_m=0.0,
        height_is_estimated=True,
        name=None,
        building_class=None,
    )


def test_building_belongs_to_the_tile_holding_its_centroid(frame):
    inside = _building(frame, 100.0, 100.0, 20.0, "inside")
    owned = buildings_in_tile([inside], frame, TileKey(0, 0), TILE)
    assert [b.id for b in owned] == ["inside"]
    assert buildings_in_tile([inside], frame, TileKey(1, 0), TILE) == []


def test_straddling_building_is_claimed_by_exactly_one_neighbour(frame):
    """A footprint across a tile border must not be extruded twice.

    Its centroid sits 4 m past the boundary, so the tile on that side owns the
    whole thing; the other side draws nothing.
    """
    straddler = _building(frame, 260.0, 100.0, 60.0, "straddler")
    left = buildings_in_tile([straddler], frame, TileKey(0, 0), TILE)
    right = buildings_in_tile([straddler], frame, TileKey(1, 0), TILE)
    assert [b.id for b in left] == []
    assert [b.id for b in right] == ["straddler"]


def test_tiling_a_neighbourhood_partitions_every_building(frame):
    """Over a patch of tiles: nothing duplicated, nothing dropped."""
    buildings = [
        _building(frame, e, n, 30.0, f"b{e}_{n}")
        # Deliberately includes centroids on and near tile boundaries, and
        # negative ENU coordinates.
        for e in (-300.0, -256.0, -10.0, 0.0, 5.0, 250.0, 256.0, 400.0)
        for n in (-260.0, -1.0, 0.0, 128.0, 256.0, 511.0)
    ]
    keys = [TileKey(ix, iy) for ix in range(-2, 3) for iy in range(-2, 3)]

    claimed: list[str] = []
    for key in keys:
        claimed.extend(b.id for b in buildings_in_tile(buildings, frame, key, TILE))

    assert len(claimed) == len(set(claimed)), "a building was claimed by two tiles"
    assert set(claimed) == {b.id for b in buildings}, "a building was claimed by none"


def test_empty_inputs_are_normal(frame):
    """An empty tile — a park, the sea — is not an error."""
    assert buildings_in_tile([], frame, TileKey(0, 0), TILE) == []
    far = _building(frame, 5000.0, 5000.0, 20.0, "far")
    assert buildings_in_tile([far], frame, TileKey(0, 0), TILE) == []


def test_step_many_unions_wants_and_intersects_unloads():
    """Multiple foci: the wanted set is the union of disks, and a live tile
    survives as long as *any* focus still covers it."""
    policy = TilePolicy(tile_m=100.0, load_radius_m=150.0, unload_radius_m=250.0)

    to_load, _ = policy.step_many([(0.0, 0.0), (1000.0, 0.0)])
    assert TileKey(0, 0) in to_load, "the first focus contributes its disk"
    assert TileKey(10, 0) in to_load, "and so does the second"
    for key in to_load:
        policy.mark_live(key)

    # The vehicle focus moves away; the anchor at (1000, 0) stays. Its tiles
    # must survive while the origin's are released.
    _, to_unload = policy.step_many([(2000.0, 0.0), (1000.0, 0.0)])
    assert TileKey(0, 0) in to_unload
    assert TileKey(10, 0) not in to_unload
    assert all(policy.distance(key, 1000.0, 0.0) > policy.unload_radius_m for key in to_unload)


def test_step_is_step_many_with_one_focus():
    policy = TilePolicy(tile_m=100.0, load_radius_m=150.0, unload_radius_m=250.0)
    assert policy.step(0.0, 0.0) == policy.step_many([(0.0, 0.0)])
