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

"""Rebuild a raw Mid-360 lidar stream (with per-point timing) from a pcap.

Streams a Livox Mid-360 UDP capture and regenerates the raw ``PointCloud2``
frames exactly the way the live driver (``livox/cpp/main.cpp``) builds them —
same mm/cm scaling, same per-point ``offset_time`` math
(``time_interval * 100 / dot_num`` ns spacing, offsets relative to the frame's
first packet) — then writes them into a memory2 db stream. Use it to backfill
recordings made before the driver published per-point timing::

    python -m dimos.hardware.sensors.lidar.livox.scripts.pcap_to_lidar_stream \
        --pcap recordings/raw_mid360.pcap --db recordings/mem2.db \
        --stream livox_lidar --replace

Unlike ``pointlio/scripts/pcap_to_db.py`` this does NOT replay through the SDK
or run any estimator: it is a deterministic offline parse (frames are cut on
sensor time, not wall time), streams the pcap instead of loading it into RAM,
and leaves every other stream in the db untouched.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterator
from pathlib import Path
import struct
import sys
import time

import numpy as np

from dimos.hardware.sensors.lidar.livox.ports import SDK_HOST_POINT_DATA_PORT
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# pcap magics (classic format; we only need the LE microsecond/nanosecond ones
# tcpdump writes on this platform).
_PCAP_MAGIC_USEC_LE = b"\xd4\xc3\xb2\xa1"
_PCAP_MAGIC_NSEC_LE = b"\x4d\x3c\xb2\xa1"
_PCAP_GLOBAL_HEADER_LEN = 24
_PCAP_RECORD_HEADER_LEN = 16
# pcap link types we can walk.
_LINKTYPE_ETHERNET = 1
_LINKTYPE_LINUX_SLL = 113
_LINKTYPE_LINUX_SLL2 = 276
_ETHERNET_HEADER_LEN = 14
_SLL_HEADER_LEN = 16
_SLL2_HEADER_LEN = 20
_ETHERTYPE_IPV4 = 0x0800
_IP_PROTO_UDP = 17
_UDP_HEADER_LEN = 8

# Livox SDK2 ethernet packet (livox_lidar_def.h, #pragma pack(1)):
# version u8, length u16, time_interval u16 (0.1us), dot_num u16, udp_cnt u16,
# frame_cnt u8, data_type u8, time_type u8, rsvd[12], crc32 u32, timestamp[8].
_LIVOX_HEADER = struct.Struct("<BHHHHBBB12xI8s")
_LIVOX_HEADER_LEN = 36
_DATA_TYPE_CARTESIAN_HIGH = 0x01
_DATA_TYPE_CARTESIAN_LOW = 0x02
# High-precision point: i32 x,y,z (mm) + u8 reflectivity + u8 tag.
_POINT_HIGH_DTYPE = np.dtype(
    [("x", "<i4"), ("y", "<i4"), ("z", "<i4"), ("reflectivity", "u1"), ("tag", "u1")]
)
# Low-precision point: i16 x,y,z (cm) + u8 reflectivity + u8 tag.
_POINT_LOW_DTYPE = np.dtype(
    [("x", "<i2"), ("y", "<i2"), ("z", "<i2"), ("reflectivity", "u1"), ("tag", "u1")]
)
_MM_TO_M = 1000.0
_CM_TO_M = 100.0
_REFLECTIVITY_MAX = 255.0
# time_interval unit is 0.1us -> *100 ns (matches livox_ros_driver2 + main.cpp).
_TIME_INTERVAL_TO_NS = 100

_PROGRESS_EVERY_FRAMES = 2000


def _iter_livox_point_packets(pcap_path: Path, host_port: int) -> Iterator[tuple[float, bytes]]:
    """Yield (capture_wall_ts, livox_udp_payload) for point-data packets, streaming."""
    with open(pcap_path, "rb") as handle:
        global_header = handle.read(_PCAP_GLOBAL_HEADER_LEN)
        if len(global_header) < _PCAP_GLOBAL_HEADER_LEN:
            raise ValueError(f"not a pcap: {pcap_path}")
        magic = global_header[:4]
        if magic not in (_PCAP_MAGIC_USEC_LE, _PCAP_MAGIC_NSEC_LE):
            raise ValueError(f"unsupported pcap magic {magic.hex()} (need classic LE): {pcap_path}")
        subsec_divisor = 1e9 if magic == _PCAP_MAGIC_NSEC_LE else 1e6
        link_type = struct.unpack("<I", global_header[20:24])[0]
        if link_type == _LINKTYPE_ETHERNET:
            l2_len = _ETHERNET_HEADER_LEN
            ethertype_offset = 12
        elif link_type == _LINKTYPE_LINUX_SLL:
            l2_len = _SLL_HEADER_LEN
            ethertype_offset = 14
        elif link_type == _LINKTYPE_LINUX_SLL2:
            l2_len = _SLL2_HEADER_LEN
            ethertype_offset = 0
        else:
            raise ValueError(f"unsupported pcap link type {link_type}")

        while True:
            record_header = handle.read(_PCAP_RECORD_HEADER_LEN)
            if len(record_header) < _PCAP_RECORD_HEADER_LEN:
                return
            sec, subsec, incl_len, _orig_len = struct.unpack("<IIII", record_header)
            frame = handle.read(incl_len)
            if len(frame) < incl_len:
                return
            wall_ts = sec + subsec / subsec_divisor

            if len(frame) < l2_len + 20 + _UDP_HEADER_LEN:
                continue
            ethertype = struct.unpack_from(">H", frame, ethertype_offset)[0]
            if ethertype != _ETHERTYPE_IPV4:
                continue
            ip_start = l2_len
            version_ihl = frame[ip_start]
            ihl = (version_ihl & 0x0F) * 4
            if frame[ip_start + 9] != _IP_PROTO_UDP:
                continue
            udp_start = ip_start + ihl
            dst_port = struct.unpack_from(">H", frame, udp_start + 2)[0]
            if dst_port != host_port:
                continue
            udp_len = struct.unpack_from(">H", frame, udp_start + 4)[0]
            payload = frame[udp_start + _UDP_HEADER_LEN : udp_start + udp_len]
            if len(payload) < _LIVOX_HEADER_LEN:
                continue
            yield wall_ts, payload


class _FrameAccumulator:
    """Mirror of the live driver's accumulator, cut on sensor time.

    The live driver assigns whole packets to the current frame and flips frames
    on a wall-clock tick; offline we flip when the incoming packet's sensor
    timestamp has moved >= frame_ns past the frame start. Same per-point offset
    math, deterministic output.

    Frames are *cut* on the sensor clock (payload timestamps: drift-free spacing)
    but can be *stamped* with the capture wall clock of the frame's first packet:
    Mid-360s that never PTP-synced stamp payloads with device uptime, while every
    other stream in a recording is on host wall time — the pcap record header
    carries the wall time that keeps the rebuilt stream aligned with them.
    """

    def __init__(self, frame_ns: int) -> None:
        self.frame_ns = frame_ns
        self.start_ns: int | None = None
        self.start_wall_ts: float | None = None
        self.xyz: list[np.ndarray] = []
        self.intensity: list[np.ndarray] = []
        self.offset_ns: list[np.ndarray] = []
        self.tag: list[np.ndarray] = []

    def add_packet(
        self,
        ts_ns: int,
        wall_ts: float,
        time_interval: int,
        points: np.ndarray,
        scale: float,
    ) -> None:
        if self.start_ns is None:
            self.start_ns = ts_ns
            self.start_wall_ts = wall_ts
        dot_num = len(points)
        if dot_num == 0:
            return
        point_interval_ns = time_interval * _TIME_INTERVAL_TO_NS // dot_num
        xyz = np.empty((dot_num, 3), dtype=np.float32)
        xyz[:, 0] = points["x"].astype(np.float32) / scale
        xyz[:, 1] = points["y"].astype(np.float32) / scale
        xyz[:, 2] = points["z"].astype(np.float32) / scale
        self.xyz.append(xyz)
        self.intensity.append(points["reflectivity"].astype(np.float32) / _REFLECTIVITY_MAX)
        packet_offset = ts_ns - self.start_ns
        self.offset_ns.append(
            (packet_offset + np.arange(dot_num, dtype=np.uint64) * point_interval_ns).astype(
                np.uint32
            )
        )
        self.tag.append(points["tag"].copy())

    def should_flush(self, next_ts_ns: int) -> bool:
        return self.start_ns is not None and next_ts_ns - self.start_ns >= self.frame_ns

    def flush(self, frame_id: str, stamp_clock: str) -> PointCloud2 | None:
        if self.start_ns is None or not self.xyz:
            self.start_ns = None
            self.start_wall_ts = None
            return None
        if stamp_clock == "capture":
            timestamp = self.start_wall_ts
        else:
            timestamp = self.start_ns / 1e9
        cloud = PointCloud2.from_numpy(
            np.concatenate(self.xyz),
            frame_id=frame_id,
            timestamp=timestamp,
            intensities=np.concatenate(self.intensity),
            offset_times=np.concatenate(self.offset_ns),
            tags=np.concatenate(self.tag),
            lines=np.zeros(sum(len(a) for a in self.tag), dtype=np.uint8),
        )
        self.start_ns = None
        self.start_wall_ts = None
        self.xyz.clear()
        self.intensity.clear()
        self.offset_ns.clear()
        self.tag.clear()
        return cloud


def _parse_livox_payload(payload: bytes) -> tuple[int, int, np.ndarray, float] | None:
    """(ts_ns, time_interval, points_structured, scale_divisor) or None to skip."""
    (
        _version,
        _length,
        time_interval,
        dot_num,
        _udp_cnt,
        _frame_cnt,
        data_type,
        _time_type,
        _crc32,
        timestamp_bytes,
    ) = _LIVOX_HEADER.unpack_from(payload)
    ts_ns = struct.unpack("<Q", timestamp_bytes)[0]
    body = payload[_LIVOX_HEADER_LEN:]
    if data_type == _DATA_TYPE_CARTESIAN_HIGH:
        dtype, scale = _POINT_HIGH_DTYPE, _MM_TO_M
    elif data_type == _DATA_TYPE_CARTESIAN_LOW:
        dtype, scale = _POINT_LOW_DTYPE, _CM_TO_M
    else:
        return None
    count = min(dot_num, len(body) // dtype.itemsize)
    if count == 0:
        return None
    points = np.frombuffer(body, dtype=dtype, count=count)
    return ts_ns, time_interval, points, scale


def _run(args: argparse.Namespace) -> int:
    from dimos.memory2.store.sqlite import SqliteStore

    pcap_path = Path(args.pcap).expanduser().resolve()
    db_path = Path(args.db).expanduser().resolve()
    if not pcap_path.exists():
        print(f"[pcap_to_lidar_stream] pcap not found: {pcap_path}", file=sys.stderr)
        return 2
    if not db_path.exists():
        print(f"[pcap_to_lidar_stream] db not found: {db_path}", file=sys.stderr)
        return 2

    frame_ns = int(1e9 / args.frequency)
    store = SqliteStore(path=str(db_path), must_exist=True)
    started = time.time()
    frames = 0
    points_total = 0
    try:
        if args.stream in store.list_streams():
            if not args.replace:
                print(
                    f"[pcap_to_lidar_stream] stream '{args.stream}' already exists in {db_path}; "
                    "pass --replace to rewrite it",
                    file=sys.stderr,
                )
                return 1
            store.delete_stream(args.stream)
            print(f"[pcap_to_lidar_stream] deleted existing stream '{args.stream}'", flush=True)

        stream = store.stream(args.stream, PointCloud2)
        accumulator = _FrameAccumulator(frame_ns)

        def flush_into_stream() -> None:
            nonlocal frames, points_total
            cloud = accumulator.flush(args.frame_id, args.stamp_clock)
            if cloud is None:
                return
            stream.append(cloud, ts=cloud.ts, pose=None)
            frames += 1
            points_total += len(cloud)
            if frames % _PROGRESS_EVERY_FRAMES == 0:
                elapsed = time.time() - started
                print(
                    f"[pcap_to_lidar_stream] {frames} frames, {points_total} points, "
                    f"{elapsed:.0f}s elapsed",
                    flush=True,
                )

        for wall_ts, payload in _iter_livox_point_packets(pcap_path, args.host_point_port):
            parsed = _parse_livox_payload(payload)
            if parsed is None:
                continue
            ts_ns, time_interval, points, scale = parsed
            if accumulator.should_flush(ts_ns):
                flush_into_stream()
            accumulator.add_packet(ts_ns, wall_ts, time_interval, points, scale)
            if args.max_frames > 0 and frames >= args.max_frames:
                break
        if not (args.max_frames > 0 and frames >= args.max_frames):
            flush_into_stream()
    finally:
        store.stop()

    elapsed = time.time() - started
    print(
        f"[pcap_to_lidar_stream] done: {frames} frames, {points_total} points, "
        f"stream '{args.stream}' in {db_path.name}, {elapsed:.0f}s",
        flush=True,
    )
    return 0 if frames > 0 else 1


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcap", required=True, help="raw Mid-360 pcap capture")
    parser.add_argument("--db", required=True, help="existing memory2 SQLite db to write into")
    parser.add_argument(
        "--stream", default="livox_lidar", help="db stream name to write (default livox_lidar)"
    )
    parser.add_argument(
        "--replace", action="store_true", help="delete the stream first if it already exists"
    )
    parser.add_argument(
        "--frequency", type=float, default=10.0, help="frame rate in sensor time (default 10 Hz)"
    )
    parser.add_argument(
        "--frame-id", default="lidar_link", help="frame_id to stamp clouds with (driver default)"
    )
    parser.add_argument(
        "--stamp-clock",
        choices=("capture", "sensor"),
        default="capture",
        help="frame stamp source: 'capture' = pcap record wall time (aligns with the other "
        "streams in a recording even when the lidar never clock-synced; default), 'sensor' = "
        "the Livox payload timestamp (what the live driver stamps)",
    )
    parser.add_argument(
        "--host-point-port",
        type=int,
        default=SDK_HOST_POINT_DATA_PORT,
        help="UDP port the point data was captured on",
    )
    parser.add_argument(
        "--max-frames", type=int, default=0, help="stop after N frames (0 = whole pcap)"
    )
    args = parser.parse_args(argv)
    return _run(args)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
