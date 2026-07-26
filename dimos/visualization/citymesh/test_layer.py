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

"""Offline tests for the CityMeshLayer visual override.

No network and no viewer: the OSM fetcher is stubbed, ``flat_ground`` skips the
DEM, and rerun logs into a sink-less recording.
"""

from __future__ import annotations

import pickle

import pytest
import rerun as rr

from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.visualization.citymesh import tiles as tiles_mod
from dimos.visualization.citymesh.layer import CityMeshLayer

ATHENS = (37.9838, 23.7275)


@pytest.fixture(autouse=True)
def recording():
    rr.init("citymesh-layer-tests")


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    monkeypatch.setattr(tiles_mod, "fetch_buildings_osm", lambda bbox, cache=True: [])


@pytest.fixture
def layer():
    lay = CityMeshLayer(flat_ground=True)
    yield lay
    if lay._runtime is not None:
        lay._runtime.streamer.close(drain_timeout_s=5.0)


def _fix(lat=ATHENS[0], lon=ATHENS[1], alt=100.0, status=NavSatFix.STATUS_FIX):
    return NavSatFix(latitude=lat, longitude=lon, altitude=alt, status=status)


def test_no_fix_returns_none_and_starts_nothing(layer):
    assert layer.on_fix(_fix(status=NavSatFix.STATUS_NO_FIX)) is None
    assert layer._runtime is None


def test_first_fix_anchors_the_frame_and_returns_geopoints(layer):
    out = layer.on_fix(_fix())
    assert type(out).__name__ == "GeoPoints"
    runtime = layer._runtime
    assert runtime is not None
    # The origin is the fix's snapped cell corner, at sea level — so the same
    # neighborhood always produces the same frame (and the same fetch cache).
    from dimos.visualization.citymesh.layer import snap_origin

    assert (runtime.frame.lat0, runtime.frame.lon0) == snap_origin(*ATHENS)
    assert runtime.frame.origin_msl == 0.0
    enu = runtime.frame.geodetic_to_enu(*ATHENS, 100.0)[0]
    assert abs(enu[0]) < 2000 and abs(enu[1]) < 2000, "fix lands near the origin"


def test_nearby_fixes_share_a_frame_origin():
    """Two sessions metres apart must hit the same fetch cache."""
    from dimos.visualization.citymesh.layer import snap_origin

    a = snap_origin(37.99372, 23.725171)
    b = snap_origin(37.99379, 23.72534)
    assert a == b


def test_anchor_wins_over_the_fix_as_frame_origin():
    lay = CityMeshLayer(flat_ground=True, anchors=[ATHENS])
    try:
        lay.on_fix(_fix(lat=37.99, lon=23.73))
        runtime = lay._runtime
        assert runtime is not None
        assert runtime.frame.lat0 == ATHENS[0]
        assert runtime.anchors_enu == [(0.0, 0.0)]
        # The anchor stays a focus: its tile is requested even though the robot
        # is several hundred metres away.
        assert len(runtime.streamer.policy.wanted(0.0, 0.0)) > 0
    finally:
        if lay._runtime is not None:
            lay._runtime.streamer.close(drain_timeout_s=5.0)


def test_render_failure_degrades_to_plain_geopoints(layer, monkeypatch):
    def boom(msg):
        raise RuntimeError("no shapely in this venv")

    monkeypatch.setattr(layer, "_render", boom)
    out = layer.on_fix(_fix())
    assert type(out).__name__ == "GeoPoints", "the MapView must keep working"
    assert layer._disabled
    assert type(layer.on_fix(_fix())).__name__ == "GeoPoints"


def test_pickle_carries_config_not_runtime(layer):
    layer.on_fix(_fix())
    assert layer._runtime is not None
    clone = pickle.loads(pickle.dumps(layer.on_fix)).__self__
    assert clone._runtime is None
    assert clone.flat_ground is True


def test_nan_altitude_from_altitudeless_receiver(layer):
    """Go2's rt/gnss carries no altitude, so fixes arrive with alt=NaN.

    NaN must not poison the ENU frame or disable the layer; the marker falls
    back to z=0 until the DEM block streams in.
    """
    import math

    out = layer.on_fix(_fix(alt=float("nan")))
    assert type(out).__name__ == "GeoPoints"
    assert not layer._disabled
    runtime = layer._runtime
    assert runtime is not None
    assert runtime.frame.origin_msl == 0.0
    e, n, z = runtime.trail[-1]
    assert math.isfinite(e) and math.isfinite(n)
    assert z == 0.0, "flat fallback until terrain arrives"
