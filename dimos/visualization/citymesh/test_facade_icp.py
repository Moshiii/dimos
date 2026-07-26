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

"""Offline tests for the lidar-to-OSM facade refinement."""

import math

import numpy as np
import pytest
from shapely.geometry import Polygon

from dimos.visualization.citymesh.facade_icp import (
    WallAccumulator,
    apply2d,
    build_edges,
    fit_z,
    refine,
    refine_global,
)
from dimos.visualization.citymesh.frame import EnuFrame
from dimos.visualization.citymesh.overture import Building

FRAME = EnuFrame.at(37.9938, 23.7253, 0.0, datum="ellipsoidal", undulation=0.0)


def _building(bid: str, e0: float, n0: float, w: float, h: float) -> Building:
    """An axis-aligned rectangle footprint at ENU (e0, n0), built via lon/lat."""
    corners_enu = np.array(
        [[e0, n0, 0], [e0 + w, n0, 0], [e0 + w, n0 + h, 0], [e0, n0 + h, 0], [e0, n0, 0]]
    )
    lat, lon, _ = FRAME.enu_to_geodetic(corners_enu)
    return Building(
        id=bid,
        geometry=Polygon(zip(lon, lat, strict=True)),
        height_m=12.0,
        min_height_m=0.0,
        height_is_estimated=False,
        name=None,
        building_class=None,
    )


BLOCKS = [
    _building("a", -60, -50, 30, 20),
    _building("b", 20, -55, 25, 35),
    _building("c", -50, 30, 40, 25),
    _building("d", 25, 25, 30, 30),
]


def _walls_from_edges(edges, step: int = 3) -> np.ndarray:
    """Pretend the lidar saw every building outline (in some odom frame)."""
    return edges.samples[::step].copy()


def test_wall_accumulator_separates_walls_from_ground():
    rng = np.random.default_rng(1)
    ground = np.column_stack(
        [rng.uniform(0, 20, 4000), rng.uniform(0, 20, 4000), rng.normal(0.0, 0.05, 4000)]
    )
    # One wall along x at y=25: tall spread of z in a thin strip.
    wall = np.column_stack(
        [rng.uniform(0, 20, 3000), rng.normal(25.0, 0.1, 3000), rng.uniform(0, 9, 3000)]
    )
    acc = WallAccumulator()
    acc.add_scan(np.vstack([ground, wall]), (10.0, 10.0))
    walls = acc.walls()
    grounds = acc.grounds()
    assert len(walls) > 10
    assert np.all(np.abs(walls[:, 1] - 25.0) < 1.0), "walls found only on the wall"
    assert len(grounds) > 100
    assert np.all(np.abs(grounds[:, 2]) < 0.5), "ground centroids sit at ground level"


def test_wall_accumulator_is_incremental():
    rng = np.random.default_rng(2)
    pts = np.column_stack(
        [rng.uniform(0, 10, 2000), rng.normal(5.0, 0.1, 2000), rng.uniform(0, 8, 2000)]
    )
    one = WallAccumulator()
    one.add_scan(pts, (5.0, 5.0))
    many = WallAccumulator()
    for chunk in np.array_split(pts, 7):
        many.add_scan(chunk, (5.0, 5.0))
    assert len(one.walls()) == len(many.walls())


def test_refine_recovers_a_biased_seed():
    edges = build_edges(BLOCKS, FRAME)
    assert edges is not None
    # The odom origin (robot boot pose) sits near the walls, as in reality —
    # a yaw seed error displaces walls by ~lever-arm, and the lever arm is
    # tens of metres, not the distance to the ENU origin.
    true_yaw, true_t = math.radians(101.0), np.array([12.0, -7.0])
    # Lidar walls live in the odom frame: enu = R(yaw) @ odom + t  =>  odom = R^-1 (enu - t).
    enu_walls = _walls_from_edges(edges)
    c, s = math.cos(true_yaw), math.sin(true_yaw)
    odom_walls = (enu_walls - true_t) @ np.array([[c, s], [-s, c]]).T
    rng = np.random.default_rng(3)
    odom_walls = odom_walls + rng.normal(0, 0.15, odom_walls.shape)

    seed_yaw, seed_t = true_yaw + math.radians(13.0), true_t + np.array([4.0, -6.0])
    result = refine(odom_walls, edges, seed_yaw, (seed_t[0], seed_t[1]))
    assert result is not None
    assert abs(math.degrees(result.yaw - true_yaw)) < 1.0
    assert abs(result.t_e - true_t[0]) < 0.5 and abs(result.t_n - true_t[1]) < 0.5
    assert result.inlier_frac > 0.9
    assert result.rms_m < 0.4


def test_refine_refuses_scenes_that_do_not_match():
    edges = build_edges(BLOCKS, FRAME)
    rng = np.random.default_rng(4)
    noise_walls = rng.uniform(-400, 400, size=(300, 2))  # no building structure
    assert refine(noise_walls, edges, 0.0, (0.0, 0.0)) is None


def test_refine_refuses_too_few_walls():
    edges = build_edges(BLOCKS, FRAME)
    assert refine(np.zeros((10, 2)), edges, 0.0, (0.0, 0.0)) is None


def test_build_edges_empty():
    assert build_edges([], FRAME) is None


def test_apply2d_matches_rotation_convention():
    p = np.array([[1.0, 0.0]])
    out = apply2d(math.pi / 2, np.zeros(2), p)[0]
    assert out == pytest.approx([0.0, 1.0], abs=1e-12)


def test_refine_global_finds_orientation_without_a_seed():
    """One GPS fix + walls: the yaw sweep must find the basin on its own."""
    edges = build_edges(BLOCKS, FRAME)
    true_yaw, true_t = math.radians(101.0), np.array([12.0, -7.0])
    enu_walls = _walls_from_edges(edges)
    c, s = math.cos(true_yaw), math.sin(true_yaw)
    odom_walls = (enu_walls - true_t) @ np.array([[c, s], [-s, c]]).T
    rng = np.random.default_rng(5)
    odom_walls = odom_walls + rng.normal(0, 0.15, odom_walls.shape)

    # A robot standing at odom (3, 4); its fix in ENU per the true transform,
    # plus GPS-grade noise. The lidar only saw walls *facing* the robot, so
    # the fixture must too — the visibility test culls the rest.
    robot_odom = np.array([3.0, 4.0])
    robot_enu_true = np.array([[c, -s], [s, c]]) @ robot_odom + true_t
    facing = ((robot_enu_true - edges.samples[::3]) * edges.normals[::3]).sum(axis=1) > 0
    odom_walls = odom_walls[facing]
    robot_enu = [*robot_enu_true, 2.0, -3.0]
    result = refine_global(
        odom_walls, edges, (robot_odom[0], robot_odom[1]), (robot_enu[0], robot_enu[1])
    )
    assert result is not None
    assert abs(math.degrees(result.yaw - true_yaw)) < 1.0
    assert result.inlier_frac > 0.9


def test_fit_z_lowers_from_the_gps_height_onto_footprint_ground():
    """Wall bottoms meet their buildings' DEM ground; flat surfaces don't vote."""
    edges = build_edges(BLOCKS, FRAME, ground_at=lambda lat, lon: 80.0)
    # Walls in odom: bottoms at z=-15 (a balcony robot looking down); the true
    # offset therefore puts odom -15 at ENU 80 -> tz = 95. GPS says 87.
    xy = _walls_from_edges(edges)  # identity transform for simplicity
    spans = np.column_stack([xy, np.full(len(xy), -15.0), np.full(len(xy), -3.0)])
    tz = fit_z(spans, edges, 0.0, (0.0, 0.0), tz0=87.0)
    assert tz is not None
    assert abs(tz - 95.0) < 0.5


def test_fit_z_ignores_occluded_bottoms_when_a_consensus_exists():
    edges = build_edges(BLOCKS, FRAME, ground_at=lambda lat, lon: 80.0)
    xy = _walls_from_edges(edges)
    bottoms = np.full(len(xy), -15.0)
    bottoms[::4] = -8.0  # a quarter of the walls are occluded partway up
    spans = np.column_stack([xy, bottoms, np.full(len(xy), -3.0)])
    tz = fit_z(spans, edges, 0.0, (0.0, 0.0), tz0=87.0)
    assert tz is not None
    assert abs(tz - 95.0) < 0.5, "the majority's ground wins, not the occluded tail"


def test_fit_z_refuses_without_consensus():
    edges = build_edges(BLOCKS, FRAME, ground_at=lambda lat, lon: 80.0)
    xy = _walls_from_edges(edges)
    rng = np.random.default_rng(6)
    spans = np.column_stack([xy, rng.uniform(-80, 0, len(xy)), np.full(len(xy), 5.0)])
    assert fit_z(spans, edges, 0.0, (0.0, 0.0), tz0=87.0) is None


def test_fit_z_needs_known_grounds():
    edges = build_edges(BLOCKS, FRAME)  # no ground_at: all NaN
    xy = _walls_from_edges(edges)
    spans = np.column_stack([xy, np.full(len(xy), -15.0), np.full(len(xy), -3.0)])
    assert fit_z(spans, edges, 0.0, (0.0, 0.0), tz0=87.0) is None
