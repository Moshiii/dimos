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

"""Provider-neutral input for grasp proposal modules."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class GraspProposalInput:
    """Segmented object geometry presented to one grasp proposal provider."""

    object_pointcloud: PointCloud2
    object_center: Vector3
    object_size: Vector3

    def __post_init__(self) -> None:
        object.__setattr__(self, "object_center", Vector3(self.object_center))
        object.__setattr__(self, "object_size", Vector3(self.object_size))

    @classmethod
    def from_pointcloud(cls, pointcloud: PointCloud2) -> GraspProposalInput:
        """Construct demo/test geometry from a point cloud's axis-aligned bounds."""
        points = pointcloud.points_f32()
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            raise ValueError("object pointcloud must contain at least one XYZ point")
        minimum = np.min(points, axis=0)
        maximum = np.max(points, axis=0)
        return cls(pointcloud, Vector3((minimum + maximum) / 2.0), Vector3(maximum - minimum))
