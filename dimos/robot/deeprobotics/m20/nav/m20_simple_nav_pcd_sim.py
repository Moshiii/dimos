#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

"""PCD-based offline simulation of the M20 simple-nav stack."""

import threading
import time
from pathlib import Path

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.pointclouds.occupancy import HeightCostConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.deeprobotics.m20.blueprints.basic import rerun
from dimos.robot.deeprobotics.m20.nav.m20_simple_nav import M20_SIMPLE_NAV_PLANNER_CONFIG
from dimos.robot.deeprobotics.m20.nav.pcd_sim import FakeRobotSim
from dimos.robot.deeprobotics.m20.tf import M20TF


class PCDMapConfig(ModuleConfig):
    pcd_path: str = str(Path(__file__).resolve().parents[1] / "m20_global_map.pcd")
    publish_hz: float = 2.0


class PCDMap(Module):
    """Load a .pcd map file and publish it as a PointCloud2 on /global_map."""

    config: PCDMapConfig
    global_map: Out[PointCloud2]

    _running: bool = False

    @rpc
    def start(self) -> None:
        super().start()
        pts = _load_pcd(self.config.pcd_path)
        msg = PointCloud2.from_numpy(pts, frame_id="map", timestamp=0.0)
        self._running = True
        period = 1.0 / max(self.config.publish_hz, 0.1)
        def _loop() -> None:
            while self._running:
                self.global_map.publish(msg)
                time.sleep(period)
        t = threading.Thread(target=_loop, daemon=True)
        t.start()

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()


def _load_pcd(path: str) -> np.ndarray:
    """Load a .pcd file (ASCII or binary), returning (N,3) float32 array.

    Parses the FIELDS / SIZE / TYPE / COUNT / DATA header lines to locate the
    x, y, z fields regardless of their position or surrounding extra fields.
    """
    import numpy as np

    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"PCD map not found: {p}")

    raw = p.read_bytes()
    data_marker = raw.find(b"DATA ")
    if data_marker < 0:
        raise ValueError("PCD missing DATA line")
    header_end = raw.index(b"\n", data_marker) + 1
    header_text = raw[:header_end].decode("utf-8", errors="replace")

    fields: list[str] = []
    size: list[int] = []
    type_: list[str] = []
    count_: list[int] = []
    num_points = 0
    binary = False

    for line in header_text.splitlines():
        parts = line.split()
        if not parts:
            continue
        key = parts[0]
        if key == "FIELDS":
            fields = parts[1:]
        elif key == "SIZE":
            size = [int(v) for v in parts[1:]]
        elif key == "TYPE":
            type_ = parts[1:]
        elif key == "COUNT":
            count_ = [int(v) for v in parts[1:]]
        elif key == "POINTS":
            num_points = int(parts[1])
        elif key == "DATA":
            binary = (parts[1] == "binary")

    if "x" not in fields or "y" not in fields or "z" not in fields:
        raise ValueError(f"PCD must contain x, y, z fields. Found: {fields}")

    ix = fields.index("x")
    iy = fields.index("y")
    iz = fields.index("z")

    strides = [count_[j] * size[j] for j in range(len(fields))]
    point_bytes = sum(strides)
    offset_x = sum(strides[:ix])
    offset_y = sum(strides[:iy])
    offset_z = sum(strides[:iz])

    out = np.empty((num_points, 3), dtype=np.float32)

    if binary:
        data = raw[header_end:]
        dtype_map = {"F": "f4", "U": "u1", "I": "i4"}
        x_dt = np.dtype(dtype_map.get(type_[ix], "f4"))
        y_dt = np.dtype(dtype_map.get(type_[iy], "f4"))
        z_dt = np.dtype(dtype_map.get(type_[iz], "f4"))
        for k in range(num_points):
            base = k * point_bytes
            if type_[ix] == "U" and count_[ix] == 1:
                out[k, 0] = float(data[base + offset_x])
            else:
                out[k, 0] = np.frombuffer(data, x_dt, 1, base + offset_x)[0]
            out[k, 1] = np.frombuffer(data, y_dt, 1, base + offset_y)[0]
            if type_[iz] == "U":
                out[k, 2] = float(data[base + offset_z])
            else:
                out[k, 2] = np.frombuffer(data, z_dt, 1, base + offset_z)[0]
        return out

    text = raw[header_end:].decode("utf-8", errors="replace")
    for i, line in enumerate(text.splitlines()):
        if i >= num_points:
            break
        parts = line.split()
        if len(parts) >= max(ix, iy, iz) + 1:
            out[i, 0] = float(parts[ix])
            out[i, 1] = float(parts[iy])
            out[i, 2] = float(parts[iz])
    return out


voxel_size = 0.05
m20_width_clearance = 0.45
m20_height_clearance = 0.8
m20_overhead_safety_margin = 0.2
m20_max_step_height = 0.15
m20_rotation_diameter = 1.2
m20_safe_radius_margin = 0.1


m20_simple_nav_pcd_sim = autoconnect(
    rerun,
    PCDMap.blueprint(),
    CostMapper.blueprint(
        config=HeightCostConfig(
            resolution=voxel_size,
            can_pass_under=m20_height_clearance + m20_overhead_safety_margin,
            can_climb=m20_max_step_height,
            ignore_noise=0.08,
            smoothing=1.5,
            min_gradient_neighbors=2,
            ignore_overhead_only=True,
        ),
        initial_safe_radius_meters=m20_width_clearance + m20_safe_radius_margin,
        inflation_radius_m=m20_width_clearance + m20_safe_radius_margin,
        gradient_distance_m=1.5,
    ),
    ReplanningAStarPlanner.blueprint(
        robot_width=m20_width_clearance,
        robot_rotation_diameter=m20_rotation_diameter,
        **M20_SIMPLE_NAV_PLANNER_CONFIG,
    ).remappings([(ReplanningAStarPlanner, "odometry", "slam_odom")]),
    MovementManager.blueprint(),
    FakeRobotSim.blueprint(
        initial_x=-1.0,
        initial_y=-3.5,
        initial_yaw=0.0,
    ).remappings([(FakeRobotSim, "slam_odom", "slam_odom")]),
    M20TF.blueprint().remappings([(M20TF, "odometry", "slam_odom")]),
).global_config(n_workers=10, robot_model="m20", robot_ip="127.0.0.1", robot_width=1.1)
