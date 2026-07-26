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

"""Offline tests for the metric terrain grid.

The grid's whole claim is that a line's position is a property of the ENU frame
and nothing else — not of the DEM's sampling, not of the tile it happens to be
drawn by. That is arithmetic, so it is testable without a viewer, a network or a
raster: the phase math below is checked directly, and the draped strips are built
against a synthetic tilted-plane :class:`~citymesh.dem.Terrain`.

The frame is built with an explicit undulation so ``EnuFrame.at`` never reaches
for the EGM2008 grid.
"""

from __future__ import annotations

import numpy as np
import pytest
import rerun as rr

from dimos.visualization.citymesh.dem import Terrain
from dimos.visualization.citymesh.extrude import (
    carpet_anchor,
    carpet_bounds,
    carpet_needs_reanchor,
    grid_line_positions,
    grid_strips,
)
from dimos.visualization.citymesh.frame import EnuFrame
from dimos.visualization.citymesh.themes import THEMES, Theme
from dimos.visualization.citymesh.tiles import TileKey, tile_bounds
from dimos.visualization.citymesh.viz import MicroCarpet

TILE = 256.0


@pytest.fixture
def frame() -> EnuFrame:
    return EnuFrame.at(37.9938, 23.7253, 0.0, datum="ellipsoidal", undulation=0.0)


@pytest.fixture
def terrain(frame: EnuFrame) -> Terrain:
    """A synthetic hillside: ground rising 1 m per 100 m of latitude.

    Deliberately coarse (~200 m between samples) so that a grid finer than the
    raster is exercised — the point being that spacing is unaffected by it.
    """
    lons = np.linspace(frame.lon0 - 0.02, frame.lon0 + 0.02, 21)
    lats = np.linspace(frame.lat0 - 0.02, frame.lat0 + 0.02, 21)
    ground = np.tile(((lats - frame.lat0) * 111_320.0 / 100.0)[:, None], (1, lons.size))
    return Terrain(lons=lons, lats=lats, surface_msl=ground, ground_msl=ground)


@pytest.fixture(autouse=True)
def recording():
    """A recording stream with no sink, so logging is a no-op."""
    rr.init("citymesh-tests")


# -- phase alignment -------------------------------------------------------


def test_lines_sit_on_exact_multiples_of_the_spacing():
    lines = grid_line_positions(-100.0, 100.0, 60.0)
    assert lines.tolist() == [-60.0, 0.0, 60.0]
    assert grid_line_positions(0.0, 61.0, 60.0).tolist() == [0.0, 60.0]


def test_phase_comes_from_the_origin_not_from_the_bounds():
    """Shifting the window must slide the lines, never the lattice."""
    for lo in (-1000.0, -333.0, 0.0, 7.5, 512.0):
        lines = grid_line_positions(lo, lo + 240.0, 15.0)
        assert np.allclose(lines % 15.0, 0.0)
        assert len(lines) == 16


def test_adjacent_tiles_tile_the_line_set_exactly_once():
    """The acceptance criterion for seams: no gap, no doubled line.

    A doubled line is not cosmetic — two strips at the same place z-fight and
    render at double brightness, which reads as a real feature of the terrain.
    """
    spacing = 15.0
    whole = grid_line_positions(0.0, 4 * TILE, spacing)
    per_tile = [grid_line_positions(i * TILE, (i + 1) * TILE, spacing) for i in range(4)]
    joined = np.concatenate(per_tile)
    assert len(joined) == len(set(joined.tolist())), "a line was drawn by two tiles"
    assert np.allclose(np.sort(joined), whole), "a line fell between two tiles"


def test_a_line_on_a_shared_border_belongs_to_the_tile_it_starts():
    # 256 is a multiple of 64: exactly the case where both neighbours could claim
    # the line at their shared edge.
    left = grid_line_positions(0.0, TILE, 64.0)
    right = grid_line_positions(TILE, 2 * TILE, 64.0)
    assert TILE not in left
    assert right[0] == TILE


def test_negative_coordinates_keep_the_same_lattice():
    lines = grid_line_positions(-512.0, -256.0, 60.0)
    assert np.allclose(lines % 60.0, 0.0)
    assert lines.min() >= -512.0 and lines.max() < -256.0


def test_a_window_narrower_than_the_spacing_may_hold_no_line():
    assert len(grid_line_positions(1.0, 59.0, 60.0)) == 0
    assert grid_line_positions(1.0, 121.0, 60.0).tolist() == [60.0, 120.0]


def test_bounds_landing_on_a_multiple_are_not_lost_to_float_error():
    """0.1-style spacings do not divide exactly; the tolerance must absorb that."""
    lines = grid_line_positions(-0.3, 0.3, 0.1)
    assert len(lines) == 6
    assert lines[0] == pytest.approx(-0.3)


def test_zero_spacing_is_rejected():
    with pytest.raises(ValueError):
        grid_line_positions(0.0, 100.0, 0.0)


# -- draped strips ---------------------------------------------------------


# A draped line is not perfectly axis-aligned: its vertices go through geodetic
# coordinates at the terrain's height, and a point 20 m up is a fraction of a
# millimetre further from the origin than the same point on the tangent plane. So
# "constant E" is a centimetre tolerance, not an exact equality — while the two
# families are 100+ m apart in their varying coordinate.
_AXIS_TOL_M = 0.01


def _line_positions(strips: list[np.ndarray]) -> tuple[set[float], set[float]]:
    """The constant-E and constant-N coordinates of a strip list."""
    es = {round(float(s[:, 0].mean()), 2) for s in strips if np.ptp(s[:, 0]) < _AXIS_TOL_M}
    ns = {round(float(s[:, 1].mean()), 2) for s in strips if np.ptp(s[:, 1]) < _AXIS_TOL_M}
    return es, ns


def test_one_shot_patch_has_lines_at_e_0_and_e_60(frame, terrain):
    """Acceptance 1: exact metres, whatever the DEM under them looks like."""
    strips = grid_strips(frame, (-300.0, -300.0, 300.0, 300.0), 60.0, terrain=terrain)
    es, ns = _line_positions(strips)
    assert {0.0, 60.0, -60.0, 240.0} <= es
    assert {0.0, 60.0} <= ns
    assert len(strips) == 20, "10 lines each way over a 600 m square at 60 m"


def test_grid_spacing_ignores_the_dem_resolution(frame, terrain):
    """A 15 m grid on a ~200 m raster is still a 15 m grid."""
    strips = grid_strips(frame, (0.0, 0.0, 120.0, 120.0), 15.0, terrain=terrain)
    es, _ = _line_positions(strips)
    assert sorted(es) == [15.0 * k for k in range(8)]


def test_strips_are_draped_on_the_terrain_and_lifted_clear_of_it(frame, terrain):
    lift = 0.15
    strips = grid_strips(frame, (-100.0, -100.0, 100.0, 100.0), 50.0, terrain=terrain, lift_m=lift)
    for strip in strips:
        lat, lon, _ = frame.enu_to_geodetic(np.column_stack([strip[:, :2], np.zeros(len(strip))]))
        ground = terrain.sample_ground_msl(lon, lat)
        expected = frame.geodetic_to_enu(lat, lon, ground, datum="msl")[:, 2] + lift
        assert np.allclose(strip[:, 2], expected, atol=1e-3)
    # The hillside rises northward, so a north-south line must climb.
    north_south = next(s for s in strips if np.ptp(s[:, 0]) < _AXIS_TOL_M)
    assert north_south[-1, 2] > north_south[0, 2] + 1.0


def test_a_tile_and_its_neighbour_agree_on_the_shared_edge(frame, terrain):
    """Acceptance 2: lines cross tile boundaries without an offset or a step."""
    spacing = 60.0
    left = grid_strips(frame, tile_bounds(TileKey(0, 0), TILE), spacing, terrain=terrain)
    right = grid_strips(frame, tile_bounds(TileKey(1, 0), TILE), spacing, terrain=terrain)

    def east_west(strips):
        return {round(float(s[:, 1].mean()), 2): s for s in strips if np.ptp(s[:, 1]) < _AXIS_TOL_M}

    shared = set(east_west(left)) & set(east_west(right))
    assert shared, "the two tiles should share every east-west line"
    for n in shared:
        a, b = east_west(left)[n], east_west(right)[n]
        assert a[-1, 0] == pytest.approx(TILE), "the left line must reach the border"
        assert b[0, 0] == pytest.approx(TILE), "the right line must start at it"
        assert a[-1, 2] == pytest.approx(b[0, 2], abs=1e-3), "no step at the seam"


def test_flat_ground_needs_no_terrain(frame):
    strips = grid_strips(frame, (0.0, 0.0, 100.0, 100.0), 25.0, lift_m=0.05)
    assert strips
    assert all(np.allclose(s[:, 2], 0.05) for s in strips)


# -- the micro carpet ------------------------------------------------------


def test_carpet_snaps_to_the_global_phase():
    assert carpet_anchor(123.4, -7.6, 1.0) == (123.0, -8.0)
    # Snapped or not, the lines themselves are on the lattice either way; this is
    # about the carpet's edges landing on it too.
    e_min, n_min, e_max, n_max = carpet_bounds(carpet_anchor(123.4, -7.6, 1.0))
    assert (e_min, n_min, e_max, n_max) == (73.0, -58.0, 173.0, 42.0)


def test_carpet_reanchors_only_past_half_a_carpet():
    anchor = (0.0, 0.0)
    assert carpet_needs_reanchor(None, 0.0, 0.0), "no carpet yet means log one"
    assert not carpet_needs_reanchor(anchor, 49.0, 0.0)
    assert not carpet_needs_reanchor(anchor, 35.0, 35.0)
    assert carpet_needs_reanchor(anchor, 51.0, 0.0)
    assert carpet_needs_reanchor(anchor, 40.0, 40.0)


def test_carpet_keeps_the_vehicle_inside_itself():
    """Whatever the threshold does, the drone must never fly off its own carpet."""
    anchor = None
    for step in range(0, 400, 7):
        e, n = float(step), float(step) * 0.5
        if carpet_needs_reanchor(anchor, e, n):
            anchor = carpet_anchor(e, n, 1.0)
        e_min, n_min, e_max, n_max = carpet_bounds(anchor)
        assert e_min <= e <= e_max and n_min <= n <= n_max


def test_carpet_strip_count_stays_sane(frame, terrain):
    """Acceptance 3: a 100 m carpet at 1 m is 200 strips, not thousands."""
    carpet = MicroCarpet(frame, THEMES["blueprint"])
    assert carpet.update(10.0, 10.0, terrain)
    assert carpet.n_strips == 200
    assert carpet.n_strips <= 250


def test_carpet_relogs_only_when_the_vehicle_has_moved(frame, terrain):
    carpet = MicroCarpet(frame, THEMES["blueprint"])
    moves = sum(carpet.update(float(e), 0.0, terrain) for e in range(0, 200, 5))
    # 200 m of travel at a 50 m threshold: the first carpet, then one every time
    # the drone gets 50 m past the last anchor.
    assert moves == 4
    assert carpet.anchor == (165.0, 0.0)


def test_carpet_is_disabled_by_a_theme_without_a_micro_layer(frame, terrain):
    carpet = MicroCarpet(frame, THEMES["day"])
    assert not carpet.enabled
    assert not carpet.update(0.0, 0.0, terrain)


# -- theme layers ----------------------------------------------------------


def test_blueprint_defines_three_resolutions():
    theme = THEMES["blueprint"]
    assert [layer.spacing_m for layer in theme.grid_layers()] == [60.0, 15.0]
    assert theme.micro_layer().spacing_m == 1.0
    major, minor = theme.grid_layers()
    assert minor.radius_m == major.radius_m / 2, "minor is half as thick"
    assert sum(minor.color) < sum(major.color), "minor is dimmer"
    assert theme.micro_layer().radius_m < minor.radius_m


def test_the_default_theme_has_no_grid():
    assert THEMES["day"].grid_layers() == []
    assert THEMES["day"].micro_layer() is None


def test_a_zero_spacing_disables_its_layer():
    import dataclasses

    theme = dataclasses.replace(THEMES["blueprint"], grid_minor_m=0.0, grid_micro_m=None)
    assert [layer.name for layer in theme.grid_layers()] == ["major"]
    assert theme.micro_layer() is None


def test_a_layer_carries_its_own_style():
    theme = Theme(name="t", grid_major_m=25.0, grid_major_color=(1, 2, 3), grid_major_radius_m=0.4)
    (layer,) = theme.grid_layers()
    assert (layer.spacing_m, layer.color, layer.radius_m) == (25.0, (1, 2, 3), 0.4)


# -- TOML migration --------------------------------------------------------


def _theme_file(tmp_path, body: str):
    path = tmp_path / "custom.toml"
    path.write_text(body)
    return path


def test_old_terrain_grid_keys_fail_naming_the_new_ones(tmp_path):
    """Acceptance 5: a theme written against the old grid must not load quietly.

    Ignoring the key would leave the user looking at a grid that silently
    disregards their settings, which is worse than an error.
    """
    from dimos.visualization.citymesh.themes import load_theme

    path = _theme_file(
        tmp_path,
        "base = 'blueprint'\nterrain_grid = true\nterrain_grid_color = [1, 2, 3]\n",
    )
    with pytest.raises(ValueError) as exc:
        load_theme(str(path))

    message = str(exc.value)
    assert "terrain_grid" in message and "terrain_grid_color" in message
    for new_key in ("grid_major_m", "grid_minor_m", "grid_micro_m", "grid_major_color"):
        assert new_key in message


def test_a_theme_using_the_new_keys_loads(tmp_path):
    from dimos.visualization.citymesh.themes import load_theme

    path = _theme_file(
        tmp_path,
        "base = 'blueprint'\ngrid_major_m = 100.0\ngrid_minor_m = 0\n"
        "grid_major_color = [10, 20, 30]\n",
    )
    theme = load_theme(str(path))
    (major,) = theme.grid_layers()
    assert major.spacing_m == 100.0
    assert major.color == (10, 20, 30)
    assert theme.micro_layer().spacing_m == 1.0, "inherited from the base theme"
