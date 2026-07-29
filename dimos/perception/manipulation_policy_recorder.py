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

"""Memory2 recorder for raw and derived manipulation observations."""

from __future__ import annotations

from dimos.core.stream import In
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.object import Object as DetObject
from dimos.perception.worldbelief_recorder import (
    WorldBeliefRecorder,
    WorldBeliefRecorderConfig,
)


class ManipulationPolicyRecorderConfig(WorldBeliefRecorderConfig):
    timestamp_db_path: bool = False


class ManipulationPolicyRecorder(WorldBeliefRecorder):
    """Record the complete observation surface used by manipulation policies."""

    detections_2d: In[Detection2DArray]
    detections_3d: In[Detection3DArray]
    objects: In[list[DetObject]]
    pointcloud: In[PointCloud2]
    config: ManipulationPolicyRecorderConfig

    def _prepare_streams(self) -> None:
        super()._prepare_streams()
        stream_types_and_codecs = {
            "detections_2d": (Detection2DArray, "lz4+lcm"),
            "detections_3d": (Detection3DArray, "lz4+lcm"),
            # Memory2 validates payloads with isinstance(), so store the
            # collection's runtime type rather than list[Object].
            "objects": (list, "lz4+pickle"),
            "pointcloud": (PointCloud2, "lz4+lcm"),
        }
        for port_name, (data_type, codec) in stream_types_and_codecs.items():
            stream_name = self.config.stream_remapping.get(port_name, port_name)
            self.store.stream(stream_name, data_type, codec=codec)


manipulation_policy_recorder = ManipulationPolicyRecorder.blueprint
