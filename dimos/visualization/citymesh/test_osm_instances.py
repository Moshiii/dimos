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

"""The Overpass instance order is session-sticky: failures demote, successes
promote, so a rate-limited primary stops taxing every subsequent block."""

import pytest

from dimos.visualization.citymesh import osm


@pytest.fixture(autouse=True)
def fresh_order():
    with osm._instance_lock:
        osm._instance_order[:] = list(osm.OVERPASS_URLS)
    yield
    with osm._instance_lock:
        osm._instance_order[:] = list(osm.OVERPASS_URLS)


def test_failure_demotes_to_the_back():
    primary = osm.OVERPASS_URLS[0]
    osm._reorder(primary, worked=False)
    order = osm._instances()
    assert order[-1] == primary
    assert order[:2] == osm.OVERPASS_URLS[1:]


def test_success_promotes_to_the_front():
    third = osm.OVERPASS_URLS[2]
    osm._reorder(third, worked=True)
    assert osm._instances()[0] == third


def test_unknown_url_is_ignored():
    osm._reorder("https://nowhere/api", worked=True)
    assert osm._instances() == osm.OVERPASS_URLS
