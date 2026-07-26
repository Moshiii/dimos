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

"""Fetch building footprints from OpenStreetMap via the Overpass API.

For a city-block-sized bbox this returns in seconds, versus minutes for a
DuckDB scan over Overture's global parquet — Overpass has a spatial index and
sends only the window you asked for. Same footprints (Overture's building layer
is largely OSM-derived), same relative-height semantics.
"""

from __future__ import annotations

import hashlib
import json
import re
import threading
from typing import Any

import requests
from shapely.geometry import MultiPolygon, Polygon

from dimos.utils.logging_config import setup_logger

from .overture import CACHE_DIR, DEFAULT_HEIGHT_M, Building, _resolve_height

log = setup_logger()

# Tried in order; the main instance is frequently overloaded (504) and rejects
# requests without a User-Agent (406).
OVERPASS_URLS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
]
USER_AGENT = "citymesh/0.1 (github.com/leshy; robotics visualization)"

# Session-sticky instance order: a streaming run fetches many blocks, and the
# primary rate-limits after the first couple (504/429), so paying its timeout
# again on every block wastes 2-15 s each. Success promotes an instance to the
# front, failure demotes it to the back; the healthiest answers first.
_instance_order = list(OVERPASS_URLS)
_instance_lock = threading.Lock()


def _reorder(url: str, worked: bool) -> None:
    with _instance_lock:
        if url in _instance_order:
            _instance_order.remove(url)
            _instance_order.insert(0 if worked else len(_instance_order), url)


def _instances() -> list[str]:
    with _instance_lock:
        return list(_instance_order)


_NUM = re.compile(r"-?\d+(?:\.\d+)?")


def _parse_metres(value: str | None) -> float | None:
    """Parse OSM height-ish tags: '12', '12.5 m', '40 ft'."""
    if not value:
        return None
    m = _NUM.search(value)
    if not m:
        return None
    metres = float(m.group())
    if "ft" in value or "'" in value:
        metres *= 0.3048
    return metres


def fetch_buildings_osm(
    bbox: tuple[float, float, float, float],
    cache: bool = True,
    urls: list[str] | None = None,
    timeout: float = 60.0,
    default_height_m: float | None = None,
) -> list[Building]:
    """Fetch buildings intersecting ``bbox`` = (min_lon, min_lat, max_lon, max_lat)."""
    min_lon, min_lat, max_lon, max_lat = bbox
    key = hashlib.sha1(f"osm|{bbox!r}".encode()).hexdigest()[:16]
    cache_file = CACHE_DIR / f"osm-{key}.json"

    if cache and cache_file.exists():
        log.info("using cached OSM buildings: %s", cache_file)
        data = json.loads(cache_file.read_text())
    else:
        # Overpass bbox order is (south, west, north, east).
        bb = f"{min_lat},{min_lon},{max_lat},{max_lon}"
        query = f"""
            [out:json][timeout:{int(timeout)}];
            (
              way["building"]({bb});
              relation["building"]["type"="multipolygon"]({bb});
            );
            out tags geom;
        """
        data = None
        errors = []
        for url in urls or _instances():
            log.info("querying %s ...", url)
            try:
                resp = requests.post(
                    url,
                    data={"data": query},
                    headers={"User-Agent": USER_AGENT},
                    timeout=timeout + 30,
                )
                resp.raise_for_status()
                data = resp.json()
                _reorder(url, worked=True)
                break
            except requests.RequestException as exc:
                errors.append(f"{url}: {exc}")
                _reorder(url, worked=False)
                log.warning("Overpass instance failed, trying next: %s", exc)
        if data is None:
            raise RuntimeError("all Overpass instances failed:\n  " + "\n  ".join(errors))
        if cache:
            cache_file.parent.mkdir(parents=True, exist_ok=True)
            cache_file.write_text(json.dumps(data))
            log.info("cached to %s", cache_file)

    buildings = []
    for el in data.get("elements", []):
        geom = _element_geometry(el)
        if geom is None or geom.is_empty:
            continue
        tags = el.get("tags", {})
        cls = tags.get("building") or None
        if cls in ("no", "entrance"):
            continue
        levels = _parse_metres(tags.get("building:levels"))
        h, estimated = _resolve_height(
            _parse_metres(tags.get("height")) or _parse_metres(tags.get("building:height")),
            int(levels) if levels else None,
            None if cls in ("yes", None) else cls,
            default_m=default_height_m if default_height_m is not None else DEFAULT_HEIGHT_M,
        )
        min_h = _parse_metres(tags.get("min_height")) or 0.0
        buildings.append(
            Building(
                id=f"{el['type']}/{el['id']}",
                geometry=geom,
                height_m=h,
                min_height_m=min_h,
                height_is_estimated=estimated,
                name=tags.get("name"),
                building_class=cls,
            )
        )
    return buildings


def _element_geometry(el: dict[str, Any]) -> Polygon | MultiPolygon | None:
    """Build a shapely polygon from an Overpass ``out geom`` element."""
    try:
        if el["type"] == "way":
            ring = [(p["lon"], p["lat"]) for p in el.get("geometry") or []]
            if len(ring) < 4:
                return None
            return Polygon(ring)

        if el["type"] == "relation":
            outers: list[list[tuple[float, float]]] = []
            inners: list[list[tuple[float, float]]] = []
            for member in el.get("members", []):
                ring = [(p["lon"], p["lat"]) for p in member.get("geometry") or []]
                if len(ring) < 4 or ring[0] != ring[-1]:
                    continue  # open fragment; assembling those isn't worth it here
                (outers if member.get("role") != "inner" else inners).append(ring)
            if not outers:
                return None
            polys = []
            for outer in outers:
                shell = Polygon(outer)
                holes = [i for i in inners if shell.contains(Polygon(i).representative_point())]
                polys.append(Polygon(outer, holes))
            return polys[0] if len(polys) == 1 else MultiPolygon(polys)
    except Exception as exc:
        log.debug("skipping OSM element %s/%s: %s", el.get("type"), el.get("id"), exc)
    return None
