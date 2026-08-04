#!/usr/bin/env python3
"""Subscribe to /global_map via Zenoh and save as .pcd."""
import signal, sys, time
import zenoh
import numpy as np
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from pathlib import Path

TOPIC = "dimos/global_map/sensor_msgs.PointCloud2"
ZENOH_CONNECT = "tcp/10.21.31.103:7447"
OUT = Path(__file__).resolve().parent / "pcd" / "m20_global_map.pcd"

latest: PointCloud2 | None = None

def handler(sample) -> None:
    global latest
    try:
        data = sample.payload.to_bytes()
        latest = PointCloud2.lcm_decode(data)
        pts, _ = latest.as_numpy()
        n = len(pts) if pts is not None else 0
        print(f"\r[map-saver] global_map: {n} points", end="", flush=True)
    except Exception as e:
        print(f"\r[map-saver] decode error: {e}", end="", flush=True)

def save() -> None:
    if latest is None:
        print("\nNo global_map received")
        return
    pts, _ = latest.as_numpy()
    if pts is None or len(pts) == 0:
        print("\nNo points in cloud")
        return
    with open(OUT, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write(f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(pts)}\nDATA ascii\n")
        for p in pts:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
    print(f"\nSaved {len(pts)} points to {OUT}")

def main() -> None:
    signal.signal(signal.SIGINT, lambda *_: (save(), sys.exit(0)))
    signal.signal(signal.SIGTERM, lambda *_: (save(), sys.exit(0)))

    config = zenoh.Config()
    config.insert_json5('connect/endpoints', f'["{ZENOH_CONNECT}"]')
    session = zenoh.open(config)

    sub = session.declare_subscriber(TOPIC, handler)
    print(f"[map-saver] Listening on {TOPIC} via Zenoh ({ZENOH_CONNECT}) — Ctrl+C to stop and save")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        save()
        sys.exit(0)

if __name__ == "__main__":
    main()
