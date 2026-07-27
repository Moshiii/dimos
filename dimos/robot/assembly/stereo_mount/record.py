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

"""Record the stereo_mount rig (ZED eyes + Point-LIO odom/lidar) into a memory2 db.

Extends :class:`~dimos.hardware.sensors.lidar.pointlio.recorder.PointlioRecorder`
(``pointlio_odometry`` / ``pointlio_lidar`` with odometry poses baked in) with the
SDK-free ZED module's ``color_image_left`` / ``color_image_right`` — names already
match, so autoconnect wires them straight in. Point-LIO publishes the moving
``world -> lidar_link`` edge onto tf and the rig's static urdf frames tie the
cameras into that tree, so every stream lands world-anchored.
"""

from __future__ import annotations

import asyncio
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.pointlio.recorder import PointlioRecorder
from dimos.memory2.module import pose_setter_for
from dimos.memory2.stream import Stream
from dimos.memory2.type.observation import Observation
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# How long pointlio_lidar may be silent before the loss banner starts repeating.
LIDAR_LOSS_TIMEOUT = 5.0

# The high-rate IMU port that gets the batched, tf-free recording path below.
IMU_PORT = "zed_imu"
# How often to flush buffered IMU samples as one transaction (~20 samples/flush
# at 800 Hz) — far under the recorder's per-message drain rate, so nothing is
# dropped, unlike the default LATEST-coalescing single-message dispatch.
IMU_FLUSH_INTERVAL = 0.05


class StereoMountRecorder(PointlioRecorder):
    # pointlio_odometry / pointlio_lidar are inherited from PointlioRecorder.
    color_image_left: In[CompressedVideo]  # h264; decode with H264Decoder
    color_image_right: In[CompressedVideo]
    camera_info_left: In[CameraInfo]
    camera_info_right: In[CameraInfo]
    zed_imu: In[Imu]  # ZED-M onboard IMU, ~800 Hz

    _watchdog_running: bool = False
    _last_lidar_at: float | None = None
    _started_at: float = 0.0

    @rpc
    def start(self) -> None:
        self._imu_buf: list[tuple[float, Any]] = []
        self._imu_lock = threading.Lock()
        self._imu_backend: Any = None
        super().start()
        self._started_at = time.time()
        self._last_lidar_at = None
        self._watchdog_running = True
        self.spawn(self._lidar_loss_watchdog())
        if self._imu_backend is not None:
            self.spawn(self._imu_flush_loop())

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Record most ports the normal way; give the ~800 Hz IMU a custom path.

        The base recorder dispatches every message through a LATEST-coalescing
        mailbox (drops intermediate messages when the handler can't keep up) and
        does a tf pose-lookup + a committed insert per message — which caps the
        IMU at ~70 Hz. For ``zed_imu`` instead: subscribe raw (no coalescing, so
        no drops), skip the tf lookup entirely (IMU carries no world pose), and
        buffer off the event loop; :meth:`_imu_flush_loop` writes each batch as a
        single transaction (N inserts, one commit).
        """
        if name != IMU_PORT:
            super()._port_to_stream(name, input_topic, stream)
            return
        self._imu_backend = stream._source

        def _buffer(msg: Any) -> None:
            with self._imu_lock:
                self._imu_buf.append((time.time(), msg))

        self.register_disposable(input_topic.pure_observable().subscribe(_buffer))
        logger.info(
            "Recording %s -> %s (%s) [batched, tf-free]",
            name,
            stream.name,
            input_topic.type.__name__,
        )

    def _flush_imu(self) -> int:
        """Write all buffered IMU samples as one transaction. Runs on the event
        loop, so it is serialised against the other streams' writes."""
        backend = self._imu_backend
        if backend is None:
            return 0
        with self._imu_lock:
            batch = self._imu_buf
            self._imu_buf = []
        if not batch:
            return 0
        for recv_ts, msg in batch:
            ts = getattr(msg, "ts", None) or recv_ts
            obs = Observation(id=-1, ts=ts, pose=None, tags={"reception_ts": recv_ts}, _data=msg)
            encoded = backend.codec.encode(msg)
            row_id = backend.metadata_store.insert(obs)  # no commit
            if backend.blob_store is not None:
                backend.blob_store.put(backend.name, row_id, encoded)  # no commit
        backend.metadata_store.commit()  # single commit for the whole batch
        return len(batch)

    async def _imu_flush_loop(self) -> None:
        while self._watchdog_running:
            await asyncio.sleep(IMU_FLUSH_INTERVAL)
            try:
                self._flush_imu()
            except Exception:
                logger.exception("ZED IMU batch flush failed")

    @pose_setter_for("pointlio_lidar")
    async def _lidar_pose(self, msg: Any) -> Any:
        # Piggyback on the recording path itself: this runs once per stored
        # lidar message, so it doubles as the loss-watchdog liveness signal.
        self._last_lidar_at = time.time()
        return await super()._lidar_pose(msg)

    async def _lidar_loss_watchdog(self) -> None:
        """Loudly and repeatedly complain while no lidar data is arriving.

        Without pointlio_lidar there is no odometry, no ``world`` tf edge, and
        every recorded stream is missing its world pose — a silent, ruined
        recording. Make it impossible to miss.
        """
        while self._watchdog_running:
            await asyncio.sleep(LIDAR_LOSS_TIMEOUT)
            if not self._watchdog_running:
                return
            silent_for = time.time() - (self._last_lidar_at or self._started_at)
            if silent_for <= LIDAR_LOSS_TIMEOUT:
                continue
            never = self._last_lidar_at is None
            logger.error("█" * 70)
            logger.error(
                "██ LIDAR DATA %s — %.0fs without pointlio_lidar messages!",
                "NEVER RECEIVED" if never else "LOST",
                silent_for,
            )
            logger.error("██ No lidar -> no odometry -> no world poses in this recording.")
            logger.error(
                "██ Check the Mid-360 power + ethernet link (ethtool <nic>) and"
                " DIMOS_POINTLIO_LIDAR_IP."
            )
            logger.error("█" * 70)

    @rpc
    def stop(self) -> None:
        self._watchdog_running = False  # also ends _imu_flush_loop
        try:
            flushed = self._flush_imu()  # drain the last buffered IMU samples
            if flushed:
                logger.info("ZED IMU: flushed final %d samples on stop", flushed)
        except Exception:
            logger.exception("ZED IMU final flush failed")
        super().stop()
