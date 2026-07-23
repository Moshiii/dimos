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
    pcd_path: str = "/home/zengxianwei/Desktop/work_resource/project_resource/code_folder/cat_m20_WD/dimos-test/m20_global_map.pcd"
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
    """Minimal ASCII .pcd loader returning (N,3) float32 array."""
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"PCD map not found: {p}")
    lines = p.read_text().splitlines()
    data_start = 0
    count = 0
    for i, line in enumerate(lines):
        if line.startswith("POINTS"):
            count = int(line.split()[1])
        if line == "DATA ascii":
            data_start = i + 1
            break
    pts = []
    for line in lines[data_start : data_start + count]:
        parts = line.split()
        if len(parts) >= 3:
            pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(pts, dtype=np.float32)


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
).global_config(n_workers=10, robot_model="m20", robot_ip="127.0.0.1")
