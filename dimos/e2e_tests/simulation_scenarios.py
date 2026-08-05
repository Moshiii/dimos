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
import math


@dataclass(frozen=True)
class SemanticNavigationScenario:
    """Provider-neutral contract for one semantic navigation task."""

    scenario_id: str
    command: str
    target_query: str
    max_target_distance_m: float
    navigation_timeout_s: float = 180.0

    def __post_init__(self) -> None:
        if not self.scenario_id.strip():
            raise ValueError("scenario ID must not be empty")
        if not self.command.strip() or not self.target_query.strip():
            raise ValueError("semantic navigation text must not be empty")
        if not math.isfinite(self.max_target_distance_m) or self.max_target_distance_m <= 0:
            raise ValueError("target distance must be finite and positive")
        if not math.isfinite(self.navigation_timeout_s) or self.navigation_timeout_s <= 0:
            raise ValueError("navigation timeout must be finite and positive")


# Canonical DimOS world-frame route used to populate spatial memory. Both
# simulator providers must present the apartment in this frame.
APARTMENT_EXPLORATION_ROUTE: tuple[tuple[float, float], ...] = (
    (3.881, 4.803),
    (4.160, 1.615),
    (1.596, 1.505),
    (1.649, 0.137),
    (-3.644, -0.064),
    (-3.759, -2.661),
    (-4.186, -4.830),
    (-3.759, -2.661),
    (-1.070, -3.285),
    (-2.504, -2.452),
    (-2.647, 5.243),
    (-3.663, 3.591),
    (-1.178, 1.974),
    (-2.416, 2.629),
    (-2.581, 0.164),
    (1.834, 0.072),
    (3.010, -3.883),
    (1.756, -3.742),
    (6.336, -4.077),
    (8.264, -5.119),
    (6.258, -0.964),
    (6.453, 5.327),
)

# The former browser-local workflows all started at Three.js (0, 0.5, 3),
# which is canonical DimOS (3, 0, 0.5). Use a slightly settled root height for
# both providers and restore this neutral start after exploration.
APARTMENT_TASK_START: tuple[float, float, float] = (3.0, 0.0, 0.52)

GO_TO_BED = SemanticNavigationScenario(
    scenario_id="bed",
    command="go to the bed",
    target_query="queen size bed",
    max_target_distance_m=2.0,
)

APARTMENT_SEMANTIC_NAVIGATION_SCENARIOS: tuple[SemanticNavigationScenario, ...] = (
    SemanticNavigationScenario(
        scenario_id="couch",
        command="go to the couch",
        target_query="sectional",
        max_target_distance_m=2.0,
    ),
    SemanticNavigationScenario(
        scenario_id="kitchen",
        command="go to the kitchen",
        target_query="refrigerator",
        max_target_distance_m=3.0,
    ),
    SemanticNavigationScenario(
        scenario_id="television",
        command="go to the TV",
        target_query="television",
        max_target_distance_m=2.0,
    ),
)


__all__ = [
    "APARTMENT_EXPLORATION_ROUTE",
    "APARTMENT_SEMANTIC_NAVIGATION_SCENARIOS",
    "APARTMENT_TASK_START",
    "GO_TO_BED",
    "SemanticNavigationScenario",
]
