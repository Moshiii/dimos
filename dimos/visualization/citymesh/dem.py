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

"""Terrain elevation from the Copernicus 30 m DEM.

Two things about this data matter for getting the scene aligned:

* **Datum.** Copernicus DEM heights are orthometric — metres above the EGM2008
  geoid, not the ellipsoid. They go through :class:`~citymesh.frame.EnuFrame`
  with ``datum="msl"`` like any other sea-level altitude.

* **It is a DSM, not a DTM.** The published product is a *surface* model: in a
  city its values sit on rooftops and tree canopy, not the street. Stacking a
  building's relative height on a raw DSM sample double-counts the building.
  :func:`estimate_ground` approximates bare earth with a low-percentile
  morphological filter, which is good enough to keep a robot's feet on the road
  across a hillside but is NOT survey-grade. For anything better, substitute a
  real DTM (FABDEM, or a national LiDAR product) via :func:`ground_from_array`.

Tiles are 1x1 degree COGs on a public bucket, read over HTTP range requests, so
only the window you ask for crosses the network.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from dimos.utils.logging_config import setup_logger

log = setup_logger()

DEM_BUCKET_URL = "https://copernicus-dem-30m.s3.amazonaws.com"
CACHE_DIR = Path.home() / ".cache" / "citymesh" / "dem"

# Ground estimation: take a low percentile over a window a couple of city blocks
# wide. Too small and the filter sits on rooftops; too large and it slides down
# the hillside.
GROUND_PERCENTILE = 12
GROUND_WINDOW_PX = 9  # ~270 m at 30 m/px.


@dataclass
class Terrain:
    """A sampled terrain patch on a regular lon/lat grid.

    ``surface_msl`` is the raw DSM; ``ground_msl`` is the bare-earth estimate.
    Both are orthometric metres.
    """

    lons: np.ndarray  # (W,)
    lats: np.ndarray  # (H,)
    surface_msl: np.ndarray  # (H, W)
    ground_msl: np.ndarray  # (H, W)

    def sample_ground_msl(self, lon: np.ndarray, lat: np.ndarray) -> np.ndarray:
        """Bilinearly sample the bare-earth estimate at arbitrary lon/lat."""
        from scipy.ndimage import map_coordinates

        col = np.interp(lon, self.lons, np.arange(self.lons.size))
        row = np.interp(lat, self.lats, np.arange(self.lats.size))
        return np.asarray(map_coordinates(self.ground_msl, [row, col], order=1, mode="nearest"))


def _tile_name(lat_deg: int, lon_deg: int) -> str:
    ns = "N" if lat_deg >= 0 else "S"
    ew = "E" if lon_deg >= 0 else "W"
    return f"Copernicus_DSM_COG_10_{ns}{abs(lat_deg):02d}_00_{ew}{abs(lon_deg):03d}_00_DEM"


def _tile_url(lat_deg: int, lon_deg: int) -> str:
    name = _tile_name(lat_deg, lon_deg)
    return f"/vsicurl/{DEM_BUCKET_URL}/{name}/{name}.tif"


def fetch_terrain(
    bbox: tuple[float, float, float, float],
    resolution_m: float = 30.0,
    cache: bool = True,
) -> Terrain:
    """Sample the DEM over ``bbox`` = (min_lon, min_lat, max_lon, max_lat)."""
    import hashlib

    min_lon, min_lat, max_lon, max_lat = bbox
    key = hashlib.sha1(f"{bbox!r}|{resolution_m}".encode()).hexdigest()[:16]
    cache_file = CACHE_DIR / f"{key}.npz"

    if cache and cache_file.exists():
        log.info("using cached terrain: %s", cache_file)
        z = np.load(cache_file)
        return Terrain(z["lons"], z["lats"], z["surface_msl"], z["ground_msl"])

    # Target grid, roughly `resolution_m` spacing.
    deg_per_m_lat = 1.0 / 111_320.0
    deg_per_m_lon = deg_per_m_lat / max(np.cos(np.radians((min_lat + max_lat) / 2)), 1e-6)
    n_lon = max(int(np.ceil((max_lon - min_lon) / (resolution_m * deg_per_m_lon))) + 1, 2)
    n_lat = max(int(np.ceil((max_lat - min_lat) / (resolution_m * deg_per_m_lat))) + 1, 2)
    lons = np.linspace(min_lon, max_lon, n_lon)
    lats = np.linspace(min_lat, max_lat, n_lat)

    surface = _read_tiles(lons, lats)
    ground = estimate_ground(surface)

    terrain = Terrain(lons=lons, lats=lats, surface_msl=surface, ground_msl=ground)
    if cache:
        cache_file.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            cache_file,
            lons=lons,
            lats=lats,
            surface_msl=surface,
            ground_msl=ground,
        )
    return terrain


def _read_tiles(lons: np.ndarray, lats: np.ndarray) -> np.ndarray:
    """Sample every 1-degree tile the grid touches into one array."""
    import rasterio
    from rasterio.errors import RasterioIOError
    from scipy.ndimage import map_coordinates

    lon_grid, lat_grid = np.meshgrid(lons, lats)
    out = np.full(lon_grid.shape, np.nan, dtype=float)

    tiles = {
        (int(np.floor(la)), int(np.floor(lo)))
        for la in (lats.min(), lats.max())
        for lo in (lons.min(), lons.max())
    }

    env = rasterio.Env(
        GDAL_DISABLE_READDIR_ON_OPEN="EMPTY_DIR",
        CPL_VSIL_CURL_ALLOWED_EXTENSIONS=".tif",
        VSI_CACHE="TRUE",
    )
    with env:
        for lat_deg, lon_deg in sorted(tiles):
            url = _tile_url(lat_deg, lon_deg)
            in_tile = (
                (lat_grid >= lat_deg)
                & (lat_grid < lat_deg + 1)
                & (lon_grid >= lon_deg)
                & (lon_grid < lon_deg + 1)
            )
            if not in_tile.any():
                continue
            try:
                with rasterio.open(url) as src:
                    rows, cols = ~src.transform * (
                        lon_grid[in_tile],
                        lat_grid[in_tile],
                    )
                    r0, r1 = int(np.floor(cols.min())) - 1, int(np.ceil(cols.max())) + 2
                    c0, c1 = int(np.floor(rows.min())) - 1, int(np.ceil(rows.max())) + 2
                    r0, c0 = max(r0, 0), max(c0, 0)
                    r1 = min(r1, src.height)
                    c1 = min(c1, src.width)
                    window = rasterio.windows.Window(c0, r0, c1 - c0, r1 - r0)
                    block = src.read(1, window=window, masked=True).filled(np.nan)
                    out[in_tile] = map_coordinates(
                        np.nan_to_num(block, nan=0.0),
                        [cols - r0, rows - c0],
                        order=1,
                        mode="nearest",
                    )
            except RasterioIOError as exc:
                log.warning("DEM tile %s unavailable (%s); leaving as NaN", url, exc)

    if np.isnan(out).all():
        raise RuntimeError(
            "no Copernicus DEM coverage for this area. Re-run with --flat-ground "
            "to place everything on a plane instead."
        )
    if np.isnan(out).any():
        out = np.nan_to_num(out, nan=float(np.nanmedian(out)))
    return out


def estimate_ground(
    surface: np.ndarray,
    percentile: int = GROUND_PERCENTILE,
    window_px: int = GROUND_WINDOW_PX,
) -> np.ndarray:
    """Approximate bare earth from a DSM.

    Takes a low percentile over a moving window — in a built-up area the low tail
    of a neighbourhood is street level — then smooths so buildings on the result
    do not step between adjacent windows.
    """
    from scipy.ndimage import gaussian_filter, percentile_filter

    ground = percentile_filter(surface, percentile=percentile, size=window_px)
    return np.asarray(gaussian_filter(ground, sigma=window_px / 4.0))


def ground_from_array(lons: np.ndarray, lats: np.ndarray, ground_msl: np.ndarray) -> Terrain:
    """Wrap an externally supplied DTM (FABDEM, national LiDAR) as a Terrain."""
    return Terrain(lons=lons, lats=lats, surface_msl=ground_msl, ground_msl=ground_msl)
