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

"""Fetch building footprints from Overture Maps.

Overture publishes GeoParquet on a public S3 bucket, spatially sorted and with a
``bbox`` struct column, so DuckDB can prune row groups and pull a city block
without downloading the planet. No credentials are needed; requests are
unsigned.

Building ``height`` here is a RELATIVE measurement — metres from the building's
own ground to its top. It carries no vertical datum, so it never needs geoid
correction; it just needs to be stacked on a correct base elevation (see
:mod:`citymesh.dem`).
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import requests
from shapely import from_wkb
from shapely.geometry.base import BaseGeometry

from dimos.utils.logging_config import setup_logger

log = setup_logger()

BUCKET = "overturemaps-us-west-2"
LIST_URL = f"https://{BUCKET}.s3.us-west-2.amazonaws.com/"
CACHE_DIR = Path.home() / ".cache" / "citymesh" / "buildings"

# Fallbacks for buildings with no height and no floor count, in metres. Overture's
# `class` is sparse, so most misses land on the default.
FLOOR_HEIGHT_M = 3.2
DEFAULT_HEIGHT_M = 9.0
CLASS_HEIGHT_M = {
    "residential": 9.0,
    "apartments": 15.0,
    "house": 6.5,
    "detached": 6.5,
    "garage": 3.0,
    "shed": 2.5,
    "hut": 2.5,
    "roof": 3.0,
    "church": 12.0,
    "industrial": 8.0,
    "warehouse": 8.0,
    "retail": 6.0,
    "commercial": 12.0,
    "office": 18.0,
    "hotel": 18.0,
    "school": 9.0,
    "hospital": 15.0,
}


@dataclass
class Building:
    """One footprint plus the numbers needed to extrude it."""

    id: str
    geometry: BaseGeometry  # Polygon in lon/lat degrees.
    height_m: float  # Relative: ground of this building to its top.
    min_height_m: float  # Relative: ground to the underside (usually 0).
    height_is_estimated: bool
    name: str | None
    building_class: str | None


def latest_release(timeout: float = 30.0) -> str:
    """Discover the newest Overture release tag, e.g. ``2026-07-22.0``."""
    resp = requests.get(
        LIST_URL,
        params={"list-type": "2", "prefix": "release/", "delimiter": "/"},
        timeout=timeout,
    )
    resp.raise_for_status()

    ns = {"s3": "http://s3.amazonaws.com/doc/2006-03-01/"}
    root = ET.fromstring(resp.text)
    versions = []
    for prefix in root.findall(".//s3:CommonPrefixes/s3:Prefix", ns):
        if prefix.text and (m := re.fullmatch(r"release/(\d{4}-\d{2}-\d{2}\.\d+)/", prefix.text)):
            versions.append(m.group(1))
    if not versions:
        raise RuntimeError(f"no Overture releases found at {LIST_URL}")
    return sorted(versions)[-1]


def _cache_path(bbox: tuple[float, float, float, float], release: str) -> Path:
    key = hashlib.sha1(f"{release}|{bbox!r}".encode()).hexdigest()[:16]
    return CACHE_DIR / f"{key}.parquet"


def fetch_buildings(
    bbox: tuple[float, float, float, float],
    release: str | None = None,
    cache: bool = True,
) -> list[Building]:
    """Fetch every building intersecting ``bbox`` = (min_lon, min_lat, max_lon, max_lat).

    Results are cached to ``~/.cache/citymesh`` keyed by bbox and release, so
    repeat runs are offline.
    """
    import duckdb

    release = release or latest_release()
    min_lon, min_lat, max_lon, max_lat = bbox
    cache_file = _cache_path(bbox, release)

    con = duckdb.connect()
    if cache and cache_file.exists():
        log.info("using cached buildings: %s", cache_file)
        rows = con.execute(f"SELECT * FROM read_parquet('{cache_file}')").fetchall()
    else:
        con.execute("INSTALL httpfs; LOAD httpfs; INSTALL spatial; LOAD spatial;")
        con.execute("SET s3_region='us-west-2';")
        source = f"s3://{BUCKET}/release/{release}/theme=buildings/type=building/*"
        # The bbox struct drives row-group pruning; the four-way overlap test then
        # keeps any footprint touching the window, not just those contained in it.
        query = f"""
            SELECT
                id,
                ST_AsWKB(geometry) AS wkb,
                height,
                min_height,
                num_floors,
                class,
                names.primary AS name
            FROM read_parquet('{source}', hive_partitioning=1)
            WHERE bbox.xmin <= {max_lon} AND bbox.xmax >= {min_lon}
              AND bbox.ymin <= {max_lat} AND bbox.ymax >= {min_lat}
              AND COALESCE(is_underground, false) = false
        """
        log.info("querying Overture %s ...", release)
        rows = con.execute(query).fetchall()
        if cache:
            cache_file.parent.mkdir(parents=True, exist_ok=True)
            con.execute(f"COPY ({query}) TO '{cache_file}' (FORMAT PARQUET)")
            log.info("cached %d buildings to %s", len(rows), cache_file)

    buildings = []
    for bid, wkb, height, min_height, num_floors, cls, name in rows:
        geom = from_wkb(bytes(wkb))
        if geom is None or geom.is_empty:
            continue
        h, estimated = _resolve_height(height, num_floors, cls)
        buildings.append(
            Building(
                id=bid,
                geometry=geom,
                height_m=h,
                min_height_m=float(min_height or 0.0),
                height_is_estimated=estimated,
                name=name,
                building_class=cls,
            )
        )
    return buildings


def _resolve_height(
    height: float | None, num_floors: int | None, cls: str | None
) -> tuple[float, bool]:
    """Best available height, and whether it had to be guessed.

    Preference: measured height -> floor count x storey height -> class default.
    """
    if height is not None and np.isfinite(height) and height > 0:
        return float(height), False
    if num_floors is not None and num_floors > 0:
        return float(num_floors) * FLOOR_HEIGHT_M, True
    return CLASS_HEIGHT_M.get(cls or "", DEFAULT_HEIGHT_M), True


def height_stats(buildings: list[Building]) -> dict[str, float]:
    """Summarise how much of the skyline is measured vs guessed."""
    if not buildings:
        return {"count": 0, "measured": 0, "measured_frac": 0.0}
    measured = sum(1 for b in buildings if not b.height_is_estimated)
    return {
        "count": len(buildings),
        "measured": measured,
        "measured_frac": measured / len(buildings),
    }
