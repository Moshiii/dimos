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

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GO2Config:
    """Physical metadata used by Go2 navigation blueprints.

    The Unitree Go2 standing envelope is 0.70 x 0.31 x 0.40 m; clearances
    include the leg stance and a safety margin.
    """

    name: str
    height_clearance: float
    width_clearance: float
    rotation_diameter: float


GO2 = GO2Config(
    name="unitree_go2",
    height_clearance=0.45,
    width_clearance=0.5,
    rotation_diameter=0.75,
)

__all__ = ["GO2", "GO2Config"]
