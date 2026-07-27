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

"""Warm the citymesh fetch cache for an area, so live runs never wait on Overpass.

Builds every tile within the radius through the same frame origin a live
session would use (:func:`~dimos.visualization.citymesh.frame.snap_origin`),
so the block bboxes — and hence the on-disk cache keys — match exactly.
Sequential on purpose: one polite Overpass request at a time.

    uv run python -m dimos.visualization.citymesh.prefetch 37.9938 23.7253 --radius 1500
"""

from __future__ import annotations

import argparse

from dimos.visualization.citymesh.frame import EnuFrame, snap_origin
from dimos.visualization.citymesh.tiles import Source, TileBuilder, tiles_within


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("lat", type=float)
    ap.add_argument("lon", type=float)
    ap.add_argument("--radius", type=float, default=1000.0, help="metres (default 1000)")
    ap.add_argument("--source", choices=("osm", "overture"), default="osm")
    ap.add_argument("--flat-ground", action="store_true", help="skip the DEM")
    args = ap.parse_args()

    origin = snap_origin(args.lat, args.lon)
    frame = EnuFrame.at(origin[0], origin[1], 0.0, datum="msl", undulation=0.0)
    source: Source = args.source
    builder = TileBuilder(frame, source=source, flat_ground=args.flat_ground)
    center = frame.geodetic_to_enu(args.lat, args.lon, 0.0)[0]
    keys = tiles_within(float(center[0]), float(center[1]), args.radius)

    built = failed = buildings = 0
    for i, key in enumerate(keys, 1):
        try:
            data = builder.build(key)
            built += 1
            buildings += data.n_buildings
            print(f"[{i}/{len(keys)}] {key}: {data.n_buildings} buildings")
        except Exception as exc:
            failed += 1
            print(f"[{i}/{len(keys)}] {key}: FAILED ({exc})")
    print(
        f"done: {built} tiles ({buildings} buildings), {failed} failed; "
        f"origin {origin[0]:.2f},{origin[1]:.2f} — a live session anchoring in "
        "this cell now runs from cache"
    )


if __name__ == "__main__":
    main()
