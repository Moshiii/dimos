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

"""PCD-based offline simulation of the M20 3D MLS navigation stack.

Loads a static PCD global map and runs the MLS planner + path follower in a
closed simulation loop with FakeRobotSim.  Useful for tuning planner parameters
and validating the surface graph against a known map without a live robot.

Usage
-----
.. code-block:: bash

   cd dimos-test
   python -m dimos.run \\
       dimos.robot.deeprobotics.m20.3dnav.m20_mls_3Dnav_pcd_sim:m20_mls_3Dnav_pcd_sim
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.basic_path_follower.module import BasicPathFollower
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.deeprobotics.m20.nav.pcd_sim import FakeRobotSim
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

# ---------------------------------------------------------------------------
# PCD map loader
# ---------------------------------------------------------------------------

PCD_PATH = str(Path(__file__).resolve().parents[1] / "m20_global_map.pcd")


class PCDMapConfig(ModuleConfig):
    pcd_path: str = PCD_PATH
    publish_hz: float = 0.5  # Static map: publish infrequently


class PCDMap(Module):
    """Load a .pcd file and publish it as a PointCloud2 on /global_map."""

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


# ---------------------------------------------------------------------------
# Rerun visualization
# ---------------------------------------------------------------------------

def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun()


def _render_path(msg: Any) -> Any:
    if len(msg.poses) == 0:
        return None
    return msg.to_rerun(color=(0, 255, 128), z_offset=0.05, radii=0.06)


def _render_node_edges(msg: Any) -> Any:
    return msg.to_rerun(z_offset=0.0)


def _render_surface(msg: Any) -> Any:
    return msg.to_rerun(radii=0.03)


def _render_nodes(msg: Any) -> Any:
    return msg.to_rerun(radii=0.06)


def _static_scene(rr: Any) -> list[tuple[str, Any]]:
    return [
        ("world/tf/map", rr.TransformAxes3D(axis_length=1.0)),
        ("world/tf/base_link", rr.TransformAxes3D(axis_length=0.5)),
    ]


def _m20_sim_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="3D",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_sim = autoconnect(
    RerunBridgeModule.blueprint(
        blueprint=_m20_sim_rerun_blueprint,
        memory_limit="4GB",
        max_hz={
            "world/global_map": 2.0,
            "world/surface_map": 2.0,
            "world/nodes": 2.0,
            "world/node_edges": 2.0,
            "world/path": 0,
        },
        visual_override={
            "world/camera_info": None,
            "world/color_image": None,
            "world/global_map": _render_global_map,
            "world/surface_map": _render_surface,
            "world/nodes": _render_nodes,
            "world/node_edges": _render_node_edges,
            "world/path": _render_path,
        },
        static={"world/tf/base_link": _static_scene},
    ),
    RerunWebSocketServer.blueprint(),
    WebsocketVisModule.blueprint(),
)

# ---------------------------------------------------------------------------
# Planner parameters (tuned for M20)
# ---------------------------------------------------------------------------

VOXEL_SIZE = 0.08  # 8 cm voxels

# M20 robot geometry / capabilities
M20_ROBOT_HEIGHT = 0.6  # body + headroom above surface
M20_WALL_CLEARANCE = 0.25  # hard clearance from walls (wider than go2)
M20_WALL_BUFFER = 0.75  # soft standoff zone
M20_WALL_BUFFER_WEIGHT = 100.0  # strong wall repulsion
M20_STEP_THRESHOLD = 0.15  # max traversable step
M20_STEP_PENALTY = 4.0  # climb penalty (prefer flat routes)
M20_SURFACE_CLOSING = 0.3  # hole-filling radius
M20_NODE_SPACING = 1.0  # graph node spacing
M20_MAX_OVERHEAD = 2.0  # ignore surface above this height over robot

# Simulation
FAKE_ROBOT_START_X = -1.0
FAKE_ROBOT_START_Y = -3.5
FAKE_ROBOT_START_YAW = 0.0

# Path follower
FOLLOWER_SPEED = 0.5
FOLLOWER_HEADING_GAIN = 0.4
FOLLOWER_MAX_ANGULAR = 0.6

# ---------------------------------------------------------------------------
# Assemble
# ---------------------------------------------------------------------------

m20_mls_3Dnav_pcd_sim = autoconnect(
    # --- Visualization ---
    rerun_sim,

    # --- Static global map from PCD ---
    PCDMap.blueprint(pcd_path=PCD_PATH, publish_hz=0.5),

    # --- MLS 3D planner (full rebuild on each global_map message) ---
    # local_map / region_bounds are left unconnected; the planner runs
    # purely on the static global_map.
    MLSPlannerNative.blueprint(
        world_frame="map",
        voxel_size=VOXEL_SIZE,
        robot_height=M20_ROBOT_HEIGHT,
        max_overhead_m=M20_MAX_OVERHEAD,
        surface_closing_radius=M20_SURFACE_CLOSING,
        node_spacing_m=M20_NODE_SPACING,
        wall_clearance_m=M20_WALL_CLEARANCE,
        wall_buffer_m=M20_WALL_BUFFER,
        wall_buffer_weight=M20_WALL_BUFFER_WEIGHT,
        step_threshold_m=M20_STEP_THRESHOLD,
        step_penalty_weight=M20_STEP_PENALTY,
        viz_publish_hz=1.0,
    ).remappings([(MLSPlannerNative, "local_map", "local_map_unused")]),

    # --- Goal relay: odometry -> start_pose, clicked goal -> goal_pose ---
    GoalRelay.blueprint().remappings([(GoalRelay, "odometry", "slam_odom")]),

    # --- Path following ---
    BasicPathFollower.blueprint(
        speed=FOLLOWER_SPEED,
        heading_gain=FOLLOWER_HEADING_GAIN,
        max_angular=FOLLOWER_MAX_ANGULAR,
    ).remappings([(BasicPathFollower, "odometry", "slam_odom")]),

    # --- Movement manager (muxes nav_cmd_vel + clicked_point -> cmd_vel) ---
    MovementManager.blueprint(),

    # --- Fake robot simulation (closed-loop: cmd_vel -> odometry) ---
    FakeRobotSim.blueprint(
        initial_x=FAKE_ROBOT_START_X,
        initial_y=FAKE_ROBOT_START_Y,
        initial_yaw=FAKE_ROBOT_START_YAW,
    ).remappings([(FakeRobotSim, "slam_odom", "slam_odom")]),

    # --- TF tree: odometry -> map->base_link ---
    M20TF.blueprint().remappings([(M20TF, "odometry", "slam_odom")]),

).global_config(
    n_workers=10,
    robot_model="m20",
    robot_ip="127.0.0.1",
    robot_width=1.1,
)
