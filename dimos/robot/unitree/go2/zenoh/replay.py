#!/usr/bin/env python3
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

"""GO2Zenoh's streams replayed from a memory2 recording — no robot needed.

Drop-in source for the go2-zenoh blueprints: the same Outs GO2Zenoh has,
fed from a :class:`GO2ZenohRecorder` db at recorded cadence (the replay's
shared wall-clock anchor keeps lidar/odometry/gps mutually in time). The
recorded ``tf`` stream is folded back onto the live tf topic, so the mount
frames and the odometry edge come along.

Selected by ``--replay`` (with ``--replay-db <name-or-path>``) on any
go2-zenoh blueprint.
"""

from __future__ import annotations

from functools import partial
from pathlib import Path
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.memory2.replay import resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.NavSatFix import NavSatFix
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GO2ZenohReplayConfig(ModuleConfig):
    dataset: str | Path = "recording_go2_zenoh.db"
    # Demos usually want the recording to keep going; the anchor restarts,
    # downstream consumers just see the walk again.
    loop: bool = True
    speed: float = 1.0


class GO2ZenohReplay(Module):
    """Publishes a GO2ZenohRecorder recording as the live topics."""

    dedicated_worker = True
    config: GO2ZenohReplayConfig

    odometry: Out[Odometry]
    lidar: Out[PointCloud2]
    pointlio_map: Out[PointCloud2]
    video: Out[CompressedVideo]
    gps: Out[NavSatFix]

    _store: SqliteStore | None = None
    _subscriptions: list[Any]

    @rpc
    def start(self) -> None:
        super().start()
        path = resolve_db_path(self.config.dataset)
        self._store = SqliteStore(path=str(path), must_exist=True)
        self._store.start()
        replay = self._store.replay(loop=self.config.loop, speed=self.config.speed)
        available = replay.list_streams()
        self._subscriptions = []
        # Aliases: stream naming differs across recorders (go2 vs drone), the
        # message types don't. First present name wins.
        outs: dict[tuple[str, ...], Any] = {
            ("odometry", "odom"): self.odometry,
            ("lidar",): self.lidar,
            ("pointlio_map",): self.pointlio_map,
            ("video", "color_image"): self.video,
            ("gps", "gps_location"): self.gps,
        }
        for names, out in outs.items():
            name = next((n for n in names if n in available), None)
            if name is None:
                logger.warning("replay: stream missing from recording", stream=names[0])
                continue
            self._subscriptions.append(
                replay.stream(name).observable().subscribe(partial(self._publish_restamped, out))
            )
        if "tf" in available:
            self._subscriptions.append(
                replay.stream("tf").observable().subscribe(self._republish_tf)
            )
        logger.info("replaying", dataset=str(path), streams=sorted(available))

    def _publish_restamped(self, out: Any, msg: Any) -> None:
        """Replayed messages pretend to be live.

        Consumers pair streams by timestamp (EnuSnapTF matches each fix to a
        tf pose); recorded stamps against a wall-clock tf buffer never pair,
        so everything leaves on one clock: now.
        """
        msg.ts = time.time()
        out.publish(msg)

    def _republish_tf(self, msg: TFMessage) -> None:
        self.tf.publish(*(t.now() for t in msg.transforms))

    @rpc
    def stop(self) -> None:
        for sub in getattr(self, "_subscriptions", []):
            sub.dispose()
        self._subscriptions = []
        if self._store is not None:
            self._store.stop()
            self._store = None
        super().stop()
