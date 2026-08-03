# Copyright 2025-2026 Dimensional Inc.
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

import pytest

from dimos.e2e_tests.dim_sim_client import DimSimClient
from dimos.e2e_tests.scene_contract import PlanarBounds


def test_planar_bounds_distance_uses_nearest_point() -> None:
    bounds = PlanarBounds(min_x=1.0, min_y=2.0, max_x=3.0, max_y=4.0)

    assert bounds.distance_to(2.0, 3.0) == 0.0
    assert bounds.distance_to(4.0, 5.0) == pytest.approx(2**0.5)


def test_dimsim_client_maps_browser_bounds_to_dimos_world(mocker) -> None:
    mocker.patch("dimos.e2e_tests.dim_sim_client.make_transport")
    browser_client = mocker.Mock()
    browser_client.get_semantic_object_bounds.return_value = {
        "min": {"x": -2.0, "y": 0.0, "z": 3.0},
        "max": {"x": 1.0, "y": 2.0, "z": 5.0},
    }
    client = DimSimClient()
    client._client = browser_client

    assert client.semantic_object_bounds("bed") == PlanarBounds(
        min_x=3.0,
        min_y=-2.0,
        max_x=5.0,
        max_y=1.0,
    )
