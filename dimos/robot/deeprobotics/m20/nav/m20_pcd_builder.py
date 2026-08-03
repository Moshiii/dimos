#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Minimal M20 PCD map builder — accumulate SLAM clouds and save a global PCD.

Usage (on the robot):
  dimos --listen-host 0.0.0.0 --transport=zenoh \\
        --zenoh-connect tcp/10.21.31.103:7447 \\
        --no-build-native run m20-pcd-builder -d

  # Push / remote-control the robot through the office.
  dimos stop
  ls m20_office_map.pcd   # ← done
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.mapping.ray_tracing.module import RayTracingVoxelMap
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.deeprobotics.m20.blueprints.basic import m20
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PCD_PATH = str(Path(__file__).resolve().parents[1] / "m20_office_map.pcd")


class PCDSaverConfig(ModuleConfig):
    save_path: str = PCD_PATH


class PCDSaver(Module):
    """Accumulate the latest global_map and write it as a .pcd on stop."""

    config: PCDSaverConfig
    global_map: In[PointCloud2]

    _latest_points: np.ndarray | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._latest_points = None
        self.register_disposable(Disposable(self.global_map.subscribe(self._on_global_map)))
        logger.info("PCDSaver started, will save to %s", self.config.save_path)

    @rpc
    def stop(self) -> None:
        if self._latest_points is not None and len(self._latest_points) > 0:
            _write_pcd(self._latest_points, self.config.save_path)
            logger.info(
                "Wrote %d points to %s", len(self._latest_points), self.config.save_path
            )
        else:
            logger.warning("No global_map received — nothing saved")
        super().stop()

    def _on_global_map(self, msg: PointCloud2) -> None:
        pts = msg.points_f32()
        if pts is not None and len(pts) > 0:
            self._latest_points = pts.copy()


def _write_pcd(points: np.ndarray, path: str) -> None:
    p = Path(path)
    header = (
        f"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n"
        f"TYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nDATA ascii\n"
    )
    with open(p, "w") as f:
        f.write(header)
        for x, y, z in points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


_m20_ray_tracer = RayTracingVoxelMap.blueprint(
    executable="target/release/voxel_ray_tracing",
    build_command=None,
    voxel_size=0.05,
    max_range=8.0,
    shadow_depth=0.1,
    min_health=-1,
    max_health=10,
    emit_every=2,
    global_emit_every=1,
    auto_build=False,
    support_min=0,
    registered_clouds=True,
).remappings(
    [
        (RayTracingVoxelMap, "lidar", "slam_aligned_points"),
        (RayTracingVoxelMap, "odometry", "slam_odom"),
    ]
)

m20_pcd_builder = autoconnect(
    m20,
    _m20_ray_tracer,
    PCDSaver.blueprint(save_path=PCD_PATH),
).global_config(n_workers=8)
