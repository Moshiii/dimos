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

"""Tags for a case, taken from its endpoints. Shape tags would need an
occupancy map, and the only map available is the one under test."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from dimos.navigation.nav_3d.evaluator.cases import Suite

# A pair of endpoints this far apart in z or beyond is a climb, not a flat
# traverse. Half the body height.
STAIRS_DZ_M = 0.5
# A climb earns long past either bound: a tall rise or a long walk.
LONG_STAIRS_DZ_M = 1.5
LONG_STAIRS_WALKED_M = 20.0

# The tags a retag recomputes. Provenance tags are left untouched.
GEOMETRIC_TAGS = frozenset({"flat", "up", "down", "stairs", "long"})


def elevation_tags(
    start: tuple[float, float, float], goal: tuple[float, float, float]
) -> list[str]:
    """Elevation from the case endpoints, not the recovered route: a case's
    climb is defined by where it starts and ends."""
    dz = goal[2] - start[2]
    euclid = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
    if abs(dz) < STAIRS_DZ_M:
        return ["flat"]
    tags = ["stairs", "up" if dz > 0 else "down"]
    if abs(dz) >= LONG_STAIRS_DZ_M or euclid >= LONG_STAIRS_WALKED_M:
        tags.append("long")
    return tags


def retag_suite(suite: Suite) -> dict[str, list[str]]:
    """New tag lists keyed by case id, for the auto cases. Curated cases are
    left out, so they keep the tags they have."""
    out: dict[str, list[str]] = {}
    for case in suite.cases:
        if "auto" not in case.tags:
            continue
        provenance = [t for t in case.tags if t not in GEOMETRIC_TAGS]
        geo = elevation_tags(case.start, case.goal)
        out[case.id] = provenance + [t for t in geo if t not in provenance]
    return out
