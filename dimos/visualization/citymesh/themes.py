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

"""Pluggable color schemes.

A :class:`Theme` owns every color decision in the scene: viewer background,
terrain ramp, building fill ramp, whether to draw wireframe edges, and what the
robot trail looks like. Builtin themes live in :data:`THEMES`; custom ones are
flat TOML files with the same field names::

    # ~/.config/citymesh/themes/mytheme.toml
    background = [6, 16, 48]
    building_low = [16, 31, 69]
    building_high = [34, 60, 110]
    building_alpha = 170
    edges = true
    edge_color = [212, 228, 255]
    grid_major_m = 60.0        # metres in ENU, 0 disables the layer
    grid_minor_m = 15.0
    grid_micro_m = 1.0         # carpet under the vehicle only

Every field is optional; anything omitted inherits from the ``base`` key
(default ``"day"``). Select with ``citymesh --theme mytheme`` — names are
resolved first against builtins, then against ``~/.config/citymesh/themes/``.
"""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from pathlib import Path

import numpy as np

RGB = tuple[int, int, int]

USER_THEME_DIR = Path.home() / ".config" / "citymesh" / "themes"

# Keys that were removed rather than renamed, with the migration for each. A
# custom theme carrying one of these was written against the DEM-sample grid; it
# must fail loudly, because silently ignoring it would leave the user staring at
# a grid that quietly ignores their settings.
RETIRED_KEYS: dict[str, str] = {
    "terrain_grid": (
        "Replace 'terrain_grid = true' with the spacings you want: "
        "grid_major_m = 60.0, grid_minor_m = 15.0, grid_micro_m = 1.0."
    ),
    "terrain_grid_color": (
        "Replace 'terrain_grid_color' with grid_major_color (plus "
        "grid_minor_color / grid_micro_color)."
    ),
    "terrain_grid_radius_m": (
        "Replace 'terrain_grid_radius_m' with grid_major_radius_m (plus "
        "grid_minor_radius_m / grid_micro_radius_m)."
    ),
}


@dataclass(frozen=True)
class GridLayer:
    """One resolution of the terrain grid, resolved from a theme.

    ``spacing_m`` is an ENU distance, so the lines land on exact multiples of it
    no matter how the terrain underneath was sampled or tiled.
    """

    name: str
    spacing_m: float
    color: RGB
    radius_m: float


@dataclass(frozen=True)
class Theme:
    name: str
    background: RGB = (255, 255, 255)

    # Building fill: linear ramp on height from `building_low` at 0 m to
    # `building_high` at `building_ramp_top_m` and above.
    building_low: RGB = (120, 120, 125)
    building_high: RGB = (210, 210, 215)
    building_ramp_top_m: float = 40.0
    building_alpha: int = 255  # <255 needs a viewer with translucency; worst case looks solid.
    # Multiplied into the fill of buildings whose height was guessed, so
    # estimates stay visually distinct. (1,1,1) disables the distinction.
    estimated_tint: tuple[float, float, float] = (1.0, 0.93, 0.84)

    # Wireframe edges (the blueprint look). Strips are generated at extrusion
    # time only when `edges` is on.
    edges: bool = False
    edge_color: RGB = (212, 228, 255)
    edge_radius_m: float = 0.12

    # Terrain fill: linear ramp on relative elevation.
    terrain_low: RGB = (70, 85, 70)
    terrain_high: RGB = (130, 140, 115)

    # Metric reference grid draped on the terrain (part of the blueprint look).
    # Spacings are ENU metres — a chosen quantity, not a multiple of whatever the
    # DEM's sample spacing happens to be. 0 or None disables that layer.
    #
    # Zoom LOD is free: radii are scene metres, so the finer layers are sub-pixel
    # from altitude and materialize as you descend.
    grid_major_m: float | None = None
    grid_major_color: RGB = (60, 96, 170)
    grid_major_radius_m: float = 0.15
    grid_minor_m: float | None = None
    grid_minor_color: RGB = (36, 58, 102)
    grid_minor_radius_m: float = 0.075
    # The micro layer is drawn only as a small carpet under the vehicle; at city
    # scale a 1 m grid would be millions of strips.
    grid_micro_m: float | None = None
    grid_micro_color: RGB = (28, 45, 80)
    grid_micro_radius_m: float = 0.01

    trail_color: RGB = (255, 140, 0)

    # -- grid layers -------------------------------------------------------

    def _layer(self, name: str) -> GridLayer | None:
        spacing = getattr(self, f"grid_{name}_m")
        if not spacing or spacing <= 0:
            return None
        return GridLayer(
            name=name,
            spacing_m=float(spacing),
            color=getattr(self, f"grid_{name}_color"),
            radius_m=float(getattr(self, f"grid_{name}_radius_m")),
        )

    def grid_layers(self) -> list[GridLayer]:
        """The enabled ground layers, coarsest first.

        Major and minor only: these are the ones generated per tile (and once per
        patch in build mode). The micro layer is :meth:`micro_layer`.
        """
        return [layer for layer in (self._layer("major"), self._layer("minor")) if layer]

    def micro_layer(self) -> GridLayer | None:
        """The vehicle-following carpet layer, or None if disabled."""
        return self._layer("micro")

    # -- color computation -------------------------------------------------

    def building_colors(self, heights_m: np.ndarray, estimated: np.ndarray) -> np.ndarray:
        """Per-building fill color. Returns (N, 3|4) uint8 (RGBA iff alpha<255)."""
        t = np.clip(np.asarray(heights_m, dtype=float) / self.building_ramp_top_m, 0.0, 1.0)[
            :, None
        ]
        lo = np.array(self.building_low, dtype=float)
        hi = np.array(self.building_high, dtype=float)
        rgb = lo + (hi - lo) * t
        rgb = np.where(
            np.asarray(estimated, dtype=bool)[:, None],
            rgb * np.array(self.estimated_tint),
            rgb,
        )
        rgb = np.clip(rgb, 0, 255)
        if self.building_alpha < 255:
            a = np.full((len(rgb), 1), self.building_alpha, dtype=float)
            rgb = np.concatenate([rgb, a], axis=1)
        return np.asarray(rgb, dtype=np.uint8)

    def terrain_colors(
        self, up_m: np.ndarray, z_range: tuple[float, float] | None = None
    ) -> np.ndarray:
        """Per-vertex terrain color from ENU up-coordinates. Returns (N, 3) uint8.

        ``z_range`` pins the ramp's endpoints. Streaming needs it: normalising
        each tile against its own min/max would give every tile the full ramp and
        the ground would come out patchwork.
        """
        z = np.asarray(up_m, dtype=float)
        lo_z, hi_z = z_range if z_range is not None else (z.min(), z.max())
        t = np.clip((z - lo_z) / max(hi_z - lo_z, 1.0), 0.0, 1.0)[:, None]
        lo = np.array(self.terrain_low, dtype=float)
        hi = np.array(self.terrain_high, dtype=float)
        return np.asarray(np.clip(lo + (hi - lo) * t, 0, 255), dtype=np.uint8)


THEMES: dict[str, Theme] = {
    "day": Theme(name="day"),
    # Modelled on the classic wireframe city-generator look: near-black navy
    # sky, translucent midnight-blue prisms, glowing ice-white edges, gridded
    # dark ground.
    "blueprint": Theme(
        name="blueprint",
        background=(5, 12, 40),
        building_low=(14, 28, 64),
        building_high=(30, 54, 104),
        building_alpha=170,
        estimated_tint=(1.0, 1.0, 1.0),  # the wireframe look doesn't mark guesses
        edges=True,
        edge_color=(214, 230, 255),
        edge_radius_m=0.12,
        terrain_low=(7, 16, 44),
        terrain_high=(13, 28, 68),
        grid_major_m=60.0,
        grid_major_color=(46, 82, 158),
        grid_major_radius_m=0.15,
        # Dimmer and half as thick as the major lines, but not the ~40% of major
        # the eye would suggest: the terrain ramp itself tops out at (13, 28, 68),
        # and a 40% line — (18, 33, 63) — vanishes into high ground. These are
        # the dimmest values that still read against the whole terrain ramp.
        grid_minor_m=15.0,
        grid_minor_color=(30, 53, 102),
        grid_minor_radius_m=0.075,
        grid_micro_m=1.0,
        grid_micro_color=(23, 41, 79),
        grid_micro_radius_m=0.01,
        trail_color=(255, 90, 60),
    ),
}


def available_themes() -> list[str]:
    names = set(THEMES)
    if USER_THEME_DIR.is_dir():
        names.update(p.stem for p in USER_THEME_DIR.glob("*.toml"))
    return sorted(names)


def load_theme(name_or_path: str) -> Theme:
    """Resolve a theme: builtin name -> user TOML by name -> explicit path."""
    if name_or_path in THEMES:
        return THEMES[name_or_path]
    candidates = [USER_THEME_DIR / f"{name_or_path}.toml", Path(name_or_path)]
    for path in candidates:
        if path.is_file():
            return _load_toml(path)
    raise KeyError(f"unknown theme {name_or_path!r}; available: {', '.join(available_themes())}")


def _load_toml(path: Path) -> Theme:
    import tomllib

    with open(path, "rb") as f:
        data = tomllib.load(f)

    base = THEMES[data.pop("base", "day")]
    data.setdefault("name", path.stem)

    retired = sorted(set(data) & set(RETIRED_KEYS))
    if retired:
        raise ValueError(
            f"{path}: {', '.join(retired)} no longer exist — the terrain grid is now "
            "metric and multi-resolution. "
            + " ".join(RETIRED_KEYS[key] for key in retired)
            + " Spacings are ENU metres; set one to 0 to disable that layer."
        )

    fields = {f.name: f for f in dataclasses.fields(Theme)}
    unknown = set(data) - set(fields) - {"base"}
    if unknown:
        raise ValueError(f"{path}: unknown theme keys: {', '.join(sorted(unknown))}")

    coerced = {}
    for key, value in data.items():
        if isinstance(value, list):
            value = tuple(value)
        coerced[key] = value
    return dataclasses.replace(base, **coerced)
