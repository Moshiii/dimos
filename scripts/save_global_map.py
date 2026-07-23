#!/usr/bin/env python3
"""Subscribe to /global_map during an M20 replay and save the last cloud as .pcd."""
from __future__ import annotations
import signal, sys
import lcm
import numpy as np
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

TOPIC = "/global_map#sensor_msgs.PointCloud2"
OUT = "/home/zengxianwei/Desktop/work_resource/project_resource/code_folder/cat_m20_WD/dimos-test/m20_global_map.pcd"
latest: PointCloud2 | None = None

def handler(channel: str, data: bytes) -> None:
    global latest
    latest = PointCloud2.lcm_decode(data)
    pts, _ = latest.as_numpy()
    n = len(pts) if pts is not None else 0
    print(f"\r[map-saver] global_map: {n} points", end="", flush=True)

def save() -> None:
    if latest is None:
        print("\nNo global_map received")
        return
    pts, _ = latest.as_numpy()
    if pts is None or len(pts) == 0:
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
    lc = lcm.LCM()
    sub = lc.subscribe(TOPIC, handler)
    print(f"[map-saver] Listening on {TOPIC} — Ctrl+C to stop and save")
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        save()
        sys.exit(0)

if __name__ == "__main__":
    main()
