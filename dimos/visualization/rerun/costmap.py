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

from typing import Any

import numpy as np

# Classic costmap palette, indexed by grid value + 1:
# transparent unknown, blue free, orange occupied, red lethal.
COSTMAP_LOOKUP_TABLE = np.zeros((102, 4), dtype=np.uint8)
COSTMAP_LOOKUP_TABLE[0] = (0, 0, 0, 0)
COSTMAP_LOOKUP_TABLE[1] = (72, 73, 129, 255)
COSTMAP_LOOKUP_TABLE[2:101] = (255, 140, 0, 255)
COSTMAP_LOOKUP_TABLE[101] = (220, 30, 30, 255)


def classic_costmap(grid: Any, z_offset: float = 0.02) -> Any:
    """Render an OccupancyGrid with the classic costmap palette.

    The default z_offset lifts the mesh 2cm off the floor plane to avoid
    z-fighting with the ground.
    """
    return grid.to_rerun(color_lookup_table=COSTMAP_LOOKUP_TABLE, z_offset=z_offset)


__all__ = ["COSTMAP_LOOKUP_TABLE", "classic_costmap"]
