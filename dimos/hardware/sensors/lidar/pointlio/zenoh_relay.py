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

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic as LCMTopic
from dimos.protocol.pubsub.impl.zenohpubsub import (
    QOS_LATEST_WINS,
    Topic as ZenohTopic,
    ZenohPubSubBase,
)


class PointLioZenohRelayConfig(ModuleConfig):
    lidar_topic: str = "lidar"
    odometry_topic: str = "odometry"


class PointLioZenohRelay(Module):
    config: PointLioZenohRelayConfig

    @rpc
    def start(self) -> None:
        super().start()
        lcm = LCMPubSubBase()
        zenoh = ZenohPubSubBase()
        lcm.start()
        zenoh.start()

        lidar_name = _topic_name(self.config.lidar_topic)
        odometry_name = _topic_name(self.config.odometry_topic)
        lidar = ZenohTopic(lidar_name, PointCloud2, qos=QOS_LATEST_WINS)
        odometry = ZenohTopic(odometry_name, Odometry)
        self.register_disposable(
            Disposable(
                lcm.subscribe(
                    LCMTopic(f"{lidar_name}/{PointCloud2.msg_name}"),
                    lambda payload, _: zenoh.publish(lidar, payload),
                )
            )
        )
        self.register_disposable(
            Disposable(
                lcm.subscribe(
                    LCMTopic(f"{odometry_name}/{Odometry.msg_name}"),
                    lambda payload, _: zenoh.publish(odometry, payload),
                )
            )
        )
        self.register_disposable(Disposable(lcm.stop))
        self.register_disposable(Disposable(zenoh.stop))


def _topic_name(topic: str) -> str:
    name = topic.strip("/")
    if name.startswith("dimos/"):
        name = name.removeprefix("dimos/")
    if not name:
        raise ValueError("PointLIO relay topics cannot be empty")
    return f"dimos/{name}"
