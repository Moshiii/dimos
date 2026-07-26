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

from __future__ import annotations

import time

from dimos_lcm.sensor_msgs.NavSatFix import NavSatFix as LCMNavSatFix

from dimos.types.timestamped import Timestamped


class NavSatFix(Timestamped):
    """GPS fix mirroring ROS sensor_msgs/NavSatFix.

    ``frame_id`` is the antenna's mount frame (the fix itself is global WGS84);
    ``altitude`` is NaN when the receiver doesn't report one.
    """

    msg_name = "sensor_msgs.NavSatFix"

    STATUS_NO_FIX = -1
    STATUS_FIX = 0
    STATUS_SBAS_FIX = 1
    STATUS_GBAS_FIX = 2
    SERVICE_GPS = 1
    SERVICE_GLONASS = 2
    SERVICE_COMPASS = 4
    SERVICE_GALILEO = 8
    COVARIANCE_TYPE_UNKNOWN = 0
    COVARIANCE_TYPE_APPROXIMATED = 1
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
    COVARIANCE_TYPE_KNOWN = 3

    def __init__(
        self,
        latitude: float = 0.0,
        longitude: float = 0.0,
        altitude: float = float("nan"),
        status: int = STATUS_NO_FIX,
        service: int = SERVICE_GPS,
        position_covariance: list[float] | None = None,
        position_covariance_type: int = COVARIANCE_TYPE_UNKNOWN,
        frame_id: str = "gps",
        ts: float | None = None,
    ) -> None:
        self.ts = ts if ts is not None else time.time()
        self.frame_id = frame_id
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.status = status
        self.service = service
        self.position_covariance = position_covariance or [0.0] * 9
        self.position_covariance_type = position_covariance_type

    @property
    def has_fix(self) -> bool:
        return self.status >= self.STATUS_FIX

    def lcm_encode(self) -> bytes:
        msg = LCMNavSatFix()
        [msg.header.stamp.sec, msg.header.stamp.nsec] = self.ros_timestamp()
        msg.header.frame_id = self.frame_id
        msg.status.status = self.status
        msg.status.service = self.service
        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude
        msg.position_covariance = self.position_covariance
        msg.position_covariance_type = self.position_covariance_type
        return msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes) -> NavSatFix:
        msg = LCMNavSatFix.lcm_decode(data)
        ts = msg.header.stamp.sec + (msg.header.stamp.nsec / 1_000_000_000)
        return cls(
            latitude=msg.latitude,
            longitude=msg.longitude,
            altitude=msg.altitude,
            status=msg.status.status,
            service=msg.status.service,
            position_covariance=list(msg.position_covariance),
            position_covariance_type=msg.position_covariance_type,
            frame_id=msg.header.frame_id,
            ts=ts,
        )

    def __str__(self) -> str:
        fix = "fix" if self.has_fix else "no fix"
        return (
            f"NavSatFix(frame_id='{self.frame_id}', {fix}, "
            f"lat={self.latitude:.6f}, lon={self.longitude:.6f}, alt={self.altitude:.1f})"
        )

    def __repr__(self) -> str:
        return (
            f"NavSatFix(ts={self.ts}, frame_id='{self.frame_id}', status={self.status}, "
            f"latitude={self.latitude}, longitude={self.longitude}, altitude={self.altitude})"
        )
