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
class PlanarBounds:
    """Axis-aligned bounds in the canonical DimOS world XY plane."""

    min_x: float
    min_y: float
    max_x: float
    max_y: float

    def __post_init__(self) -> None:
        values = (self.min_x, self.min_y, self.max_x, self.max_y)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("planar bounds must be finite")
        if self.min_x > self.max_x or self.min_y > self.max_y:
            raise ValueError("planar bounds minimum must not exceed maximum")

    def distance_to(self, x: float, y: float) -> float:
        """Return planar distance from a point to the filled bounds."""
        dx = max(self.min_x - x, 0.0, x - self.max_x)
        dy = max(self.min_y - y, 0.0, y - self.max_y)
        return math.hypot(dx, dy)


__all__ = ["PlanarBounds"]
