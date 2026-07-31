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

"""Deterministic task generation from a private DimSim scene oracle."""

from dimos.benchmark.dimsim.bundle import generate_smoke_release, load_public_tasks
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture
from dimos.benchmark.dimsim.generation import compile_smoke_tasks
from dimos.benchmark.dimsim.models import SceneOracleView
from dimos.benchmark.dimsim.oracle import InMemorySceneOracleProvider, SceneOracleProvider

__all__ = [
    "InMemorySceneOracleProvider",
    "SceneOracleProvider",
    "SceneOracleView",
    "apartment_oracle_fixture",
    "compile_smoke_tasks",
    "generate_smoke_release",
    "load_public_tasks",
]
