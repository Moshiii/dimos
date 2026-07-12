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

"""Feed a memory2 recording to a running fastlivo_native over LCM and record its odometry.

Streams lidar/imu/color_image/camera_info observations from a memory2 db in
global timestamp order, paced by their recorded spacing (``--rate`` scales
playback speed), while capturing the odometry the binary publishes into a TUM
trajectory file (``ts x y z qx qy qz qw`` per line).

The camera_info stream is additionally published once at startup so the binary
can construct its camera model before the first images arrive.

Run the binary first (or use --spawn to let this script own it), e.g.::

    ./result/bin/fastlivo_native \
        --lidar '/fl_lidar#sensor_msgs.PointCloud2' \
        --imu '/fl_imu#sensor_msgs.Imu' \
        --color_image '/fl_img#sensor_msgs.Image' \
        --camera_info '/fl_caminfo#sensor_msgs.CameraInfo' \
        --odometry '/fl_odom#nav_msgs.Odometry' \
        --frame_id odom --sensor_frame_id mid360_link

    python -m dimos.hardware.sensors.lidar.fastlivo.scripts.replay_eval \
        --db /path/to/mem2.db --out /tmp/fastlivo_odom.tum
"""

from __future__ import annotations

import argparse
import heapq
import sys
import threading
import time
from typing import Any

import lcm

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _stream_iter(
    store: Any,
    name: str,
    msg_type: type,
    channel: str,
    start: float | None,
    stop: float | None,
    restamp: tuple[float, float] | None = None,
    fix_domain_latency: float | None = None,
) -> Any:
    stream = store.stream(name, msg_type).order_by("ts")
    if start is not None:
        stream = stream.after(start)
    for obs in stream:
        if stop is not None and obs.ts > stop:
            return
        msg = obs.data
        if restamp is not None:
            slope, intercept = restamp
            msg.ts = slope * msg.ts + intercept
        elif fix_domain_latency is not None and abs(msg.ts - obs.ts) > _CLOCK_DOMAIN_TOLERANCE_S:
            # Stray message stamped in a foreign clock domain (huge_loop has
            # ~4% of images carrying the lidar's uptime clock): reconstruct
            # from the store row time minus the stream's typical pipeline
            # latency.
            msg.ts = obs.ts - fix_domain_latency
        yield (obs.ts, channel, msg)


# An in-message stamp this far (s) from the store row ts means the message
# carries a different clock (the Mid-360's un-synced internal uptime clock).
_CLOCK_DOMAIN_TOLERANCE_S = 1e6


def _fit_imu_clock(store: Any, name: str, sample_stride: int = 200) -> tuple[float, float] | None:
    """Map the imu messages' internal clock onto the store's UTC row timestamps.

    The Mid-360 stamps IMU packets with its internal uptime clock unless
    PTP-synced. A least-squares linear fit uptime→row-ts recovers UTC stamps
    while keeping the sensor clock's clean 5 ms spacing (row timestamps alone
    carry recorder-arrival jitter) and absorbing clock drift over the run.
    Returns None when the stamps already match the row clock.
    """
    import numpy as np

    from dimos.msgs.sensor_msgs.Imu import Imu

    msg_ts = []
    row_ts = []
    for i, obs in enumerate(store.stream(name, Imu).order_by("ts")):
        if i % sample_stride != 0:
            continue
        msg_ts.append(obs.data.ts)
        row_ts.append(obs.ts)
    if len(msg_ts) < 2:
        return None
    if abs(msg_ts[0] - row_ts[0]) < _CLOCK_DOMAIN_TOLERANCE_S:
        return None
    slope, intercept = np.polyfit(np.asarray(msg_ts), np.asarray(row_ts), 1)
    residual = np.asarray(row_ts) - (slope * np.asarray(msg_ts) + intercept)
    print(
        f"[replay] imu clock fit: slope={slope:.9f} offset={intercept:.3f} "
        f"residual std={residual.std() * 1000:.2f}ms over {len(msg_ts)} samples",
        flush=True,
    )
    return float(slope), float(intercept)


def _fit_image_latency(store: Any, name: str, sample_stride: int = 25) -> float:
    """Median store-row-ts minus in-message capture stamp for in-domain images."""
    import numpy as np

    from dimos.msgs.sensor_msgs.Image import Image

    latencies = []
    for i, obs in enumerate(store.stream(name, Image).order_by("ts")):
        if i % sample_stride != 0:
            continue
        if abs(obs.data.ts - obs.ts) < _CLOCK_DOMAIN_TOLERANCE_S:
            latencies.append(obs.ts - obs.data.ts)
    latency = float(np.median(latencies)) if latencies else 0.0
    print(
        f"[replay] image pipeline latency: {latency * 1000:.1f}ms ({len(latencies)} samples)",
        flush=True,
    )
    return latency


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", required=True)
    parser.add_argument("--out", required=True, help="TUM trajectory output path")
    parser.add_argument("--rate", type=float, default=1.0, help="playback speed multiplier")
    parser.add_argument(
        "--start", type=float, default=None, help="absolute start ts (default: stream begin)"
    )
    parser.add_argument("--duration", type=float, default=None, help="seconds of data to replay")
    parser.add_argument("--lidar-stream", default="livox_lidar")
    parser.add_argument("--imu-stream", default="livox_imu")
    parser.add_argument("--image-stream", default="color_image")
    parser.add_argument("--camera-info-stream", default="realsense_camera_info")
    parser.add_argument("--lidar-channel", default="/fl_lidar#sensor_msgs.PointCloud2")
    parser.add_argument("--imu-channel", default="/fl_imu#sensor_msgs.Imu")
    parser.add_argument("--image-channel", default="/fl_img#sensor_msgs.Image")
    parser.add_argument("--camera-info-channel", default="/fl_caminfo#sensor_msgs.CameraInfo")
    parser.add_argument("--odom-channel", default="/fl_odom#nav_msgs.Odometry")
    parser.add_argument(
        "--tail-wait",
        type=float,
        default=20.0,
        help="seconds to keep capturing odom after the feed ends",
    )
    args = parser.parse_args()

    from dimos.memory2.store.sqlite import SqliteStore

    store = SqliteStore(path=args.db, must_exist=True)

    lc = lcm.LCM()
    odom_rows: list[tuple[float, Odometry]] = []
    odom_lock = threading.Lock()

    def on_odom(_channel: str, data: bytes) -> None:
        msg = Odometry.lcm_decode(data)
        with odom_lock:
            odom_rows.append((msg.ts, msg))

    lc.subscribe(args.odom_channel, on_odom)
    capture_running = True

    def capture_loop() -> None:
        while capture_running:
            lc.handle_timeout(100)

    capture_thread = threading.Thread(target=capture_loop, daemon=True)
    capture_thread.start()

    pub = lcm.LCM()

    # Publish camera_info up front so the binary can build its camera model
    # before any image arrives.
    first_ci = next(
        iter(store.stream(args.camera_info_stream, CameraInfo).order_by("ts").limit(1)), None
    )
    if first_ci is not None:
        pub.publish(args.camera_info_channel, first_ci.data.lcm_encode())

    first_lidar = next(
        iter(store.stream(args.lidar_stream, PointCloud2).order_by("ts").limit(1)), None
    )
    if first_lidar is None:
        print(f"no data in stream {args.lidar_stream}", file=sys.stderr)
        sys.exit(1)
    start_ts = args.start if args.start is not None else first_lidar.ts
    stop_ts = start_ts + args.duration if args.duration is not None else None

    imu_restamp = _fit_imu_clock(store, args.imu_stream)
    image_latency = _fit_image_latency(store, args.image_stream)

    sources = [
        _stream_iter(store, args.lidar_stream, PointCloud2, args.lidar_channel, start_ts, stop_ts),
        _stream_iter(
            store, args.imu_stream, Imu, args.imu_channel, start_ts, stop_ts, restamp=imu_restamp
        ),
        _stream_iter(
            store,
            args.image_stream,
            Image,
            args.image_channel,
            start_ts,
            stop_ts,
            fix_domain_latency=image_latency,
        ),
        _stream_iter(
            store, args.camera_info_stream, CameraInfo, args.camera_info_channel, start_ts, stop_ts
        ),
    ]
    merged = heapq.merge(*sources, key=lambda item: item[0])

    wall_t0 = time.monotonic()
    data_t0: float | None = None
    published = 0
    last_report = time.monotonic()

    for ts, channel, msg in merged:
        if data_t0 is None:
            data_t0 = ts
        target_wall = wall_t0 + (ts - data_t0) / args.rate
        delay = target_wall - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        pub.publish(channel, msg.lcm_encode())
        published += 1
        now = time.monotonic()
        if now - last_report > 10.0:
            with odom_lock:
                n_odom = len(odom_rows)
            print(
                f"[replay] data t+{ts - data_t0:8.1f}s  published={published}  odom_rx={n_odom}",
                flush=True,
            )
            last_report = now

    print(
        f"[replay] feed done ({published} msgs), waiting {args.tail_wait}s for trailing odometry",
        flush=True,
    )
    time.sleep(args.tail_wait)
    capture_running = False
    capture_thread.join(timeout=2.0)

    with odom_lock:
        rows = sorted(odom_rows, key=lambda row: row[0])
    with open(args.out, "w") as f:
        for ts, msg in rows:
            p = msg.pose.position
            q = msg.pose.orientation
            f.write(
                f"{ts:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
            )
    print(f"[replay] wrote {len(rows)} odometry rows to {args.out}", flush=True)


if __name__ == "__main__":
    main()
