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

"""The dimos adapter: a stateful visual override that streams a city around GPS fixes.

Register :meth:`CityMeshLayer.on_fix` in a blueprint's rerun ``visual_override``
for a NavSatFix entity. Each fix still returns the ``GeoPoints`` the MapView
consumes; as a side effect the layer anchors an ENU frame at the first fix,
streams city tiles around the robot (and any configured anchors) under ``root``,
and logs a marker plus trail — everything a ``City`` 3D view needs.

Runs inside the RerunBridgeModule process, which is where the recording lives
and where internet access is expected. Nothing here crosses the transport.

The heavy geo stack (shapely, rasterio, trimesh…) comes from the ``citymesh``
extra and is imported on the first fix, not at module load: a blueprint can
declare the layer without the extra installed, and the layer degrades to plain
GeoPoints rendering with a warning.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass, field
import math
import threading
import time
from typing import TYPE_CHECKING, Any, Literal

from dimos.utils.logging_config import setup_logger
from dimos.visualization.citymesh.georegister import Registration, TrackAligner

if TYPE_CHECKING:
    from dimos.visualization.citymesh.frame import AltDatum, EnuFrame
    from dimos.visualization.citymesh.tiles import TileStreamer

logger = setup_logger()

Source = Literal["osm", "overture"]

# Auto-anchors snap to this grid, and the frame origin always sits at sea
# level: the origin is arbitrary, but block bboxes — and so the on-disk fetch
# cache — are pure functions of it. Fix-exact origins gave every session its
# own bbox floats and a cold cache; snapped ones make one session's Overpass
# answers the next session's cache hits. ~1.1 km grid, well inside the
# float32 precision budget.
SNAP_DEG = 0.01


def snap_origin(lat: float, lon: float) -> tuple[float, float]:
    """The deterministic frame origin for any fix in this ~1 km cell."""
    return (round(lat / SNAP_DEG) * SNAP_DEG, round(lon / SNAP_DEG) * SNAP_DEG)


@dataclass
class _Runtime:
    """Everything created on the first fix, in the bridge process only."""

    frame: EnuFrame
    streamer: TileStreamer
    anchors_enu: list[tuple[float, float]]
    trail: deque[tuple[float, float, float]] = field(default_factory=lambda: deque(maxlen=4096))
    # Georegistration: odom and GPS tracks paired by bridge arrival time (the
    # two stamps ride different clocks; arrival skew is bounded by GPS latency,
    # well under GPS noise at walking speed).
    aligner: TrackAligner = field(default_factory=TrackAligner)
    registration: Registration | None = None
    last_solve: float = 0.0
    last_odom_z: float = 0.0
    last_fix_enu_z: float = 0.0


class CityMeshLayer:
    """Renders an extruded city around coordinates we care about.

    ``anchors`` are static (lat, lon) points to keep loaded regardless of where
    the robot is; the fix stream itself is the moving focus. The first fix
    anchors the ENU frame, which then never moves.

    ``geoid_undulation=0.0`` (the default) keeps the whole scene consistently
    orthometric — GPS altitude, Copernicus DEM and building heights all already
    share the MSL datum, so no geoid model (and no pyproj network fetch) is
    needed. Pass ``None`` to look up EGM2008 instead.
    """

    def __init__(
        self,
        anchors: Sequence[tuple[float, float]] = (),
        theme: str = "blueprint",
        source: Source = "osm",
        root: str = "world/city",
        flat_ground: bool = False,
        load_radius_m: float = 400.0,
        unload_radius_m: float = 700.0,
        workers: int = 2,
        datum: AltDatum = "msl",
        geoid_undulation: float | None = 0.0,
        marker_radius_m: float = 0.5,
    ) -> None:
        self.anchors = list(anchors)
        self.theme = theme
        self.source: Source = source
        self.root = root
        self.flat_ground = flat_ground
        self.load_radius_m = load_radius_m
        self.unload_radius_m = unload_radius_m
        self.workers = workers
        self.datum: AltDatum = datum
        self.geoid_undulation = geoid_undulation
        self.marker_radius_m = marker_radius_m

        self._lock = threading.Lock()
        self._runtime: _Runtime | None = None
        self._disabled = False

    # The layer travels inside the bridge blueprint's config; only its
    # configuration should cross the process boundary, never live runtime state.
    def __getstate__(self) -> dict[str, Any]:
        state = self.__dict__.copy()
        state["_lock"] = None
        state["_runtime"] = None
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        self.__dict__.update(state)
        self._lock = threading.Lock()
        self._runtime = None

    def on_fix(self, msg: Any) -> Any:
        """Visual override for a NavSatFix topic.

        Returns GeoPoints for the MapView (None while searching — stale coords
        would pin the robot somewhere it isn't); city rendering is the side
        effect.
        """
        if not getattr(msg, "has_fix", False):
            return None
        geo = msg.to_rerun()
        if self._disabled:
            return geo
        with self._lock:
            try:
                self._render(msg)
            except Exception:
                # A broken city must never take the GPS stream down with it.
                logger.exception(
                    "citymesh rendering failed, disabling for this session "
                    "(is the `citymesh` extra installed?)"
                )
                self._disabled = True
        return geo

    def _render(self, msg: Any) -> None:
        runtime = self._runtime
        if runtime is None:
            runtime = self._start(msg)
        frame = runtime.frame
        # The Go2's rt/gnss carries no altitude, so NavSatFix arrives with the
        # standard "unknown" marker (NaN). The horizontal conversion barely
        # cares (cm-scale over a city), so any finite stand-in works; the
        # marker's z is then draped on the DEM instead of trusting the fix.
        alt = frame.origin_msl if math.isnan(msg.altitude) else msg.altitude
        enu = frame.geodetic_to_enu(msg.latitude, msg.longitude, alt, datum=self.datum)[0]
        e, n = float(enu[0]), float(enu[1])
        z = self._ground_z(runtime, e, n) if math.isnan(msg.altitude) else float(enu[2])
        self._log_robot(runtime, (e, n, z))
        runtime.streamer.update((e, n, z), extra_foci=runtime.anchors_enu)
        self._georegister(runtime, e, n, z)

    def _georegister(self, runtime: _Runtime, e: float, n: float, z: float) -> None:
        """Pair this fix with the odom track and refresh the city's placement."""
        now = time.monotonic()
        runtime.aligner.add_fix(now, e, n)
        runtime.last_fix_enu_z = z
        if now - runtime.last_solve < 2.0:
            return
        runtime.last_solve = now
        registration = runtime.aligner.solve()
        if registration is None:
            return
        first = runtime.registration is None
        runtime.registration = registration
        self._log_registration(runtime, registration)
        log_fn = logger.info if first else logger.debug
        log_fn(
            "citymesh georegistered",
            yaw_deg=round(math.degrees(registration.yaw), 1),
            rms_m=round(registration.rms_m, 2),
            pairs=registration.n_pairs,
        )

    def _log_registration(self, runtime: _Runtime, registration: Registration) -> None:
        """Place the ENU scene under the odom-frame world view.

        The transform on ``root`` maps its (ENU) content into the parent's
        odom coordinates; yaw about +z only — both frames are gravity-aligned.
        """
        import rerun as rr

        yaw, (te, tn) = registration.odom_from_enu()
        tz = runtime.last_odom_z - runtime.last_fix_enu_z
        rr.log(
            self.root,
            rr.Transform3D(
                translation=[te, tn, tz],
                rotation=rr.Quaternion(xyzw=[0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]),
            ),
        )

    def on_odom(self, msg: Any) -> Any:
        """Visual override for the odometry topic: feed the aligner, pass through.

        Registered alongside :meth:`on_fix` so both tracks meet in this one
        object; the message itself renders exactly as it would without us.
        """
        runtime = self._runtime
        if runtime is not None and not self._disabled:
            position = msg.pose.pose.position
            with self._lock:
                runtime.aligner.add_odom(time.monotonic(), float(position[0]), float(position[1]))
                runtime.last_odom_z = float(position[2])
        return msg

    def _ground_z(self, runtime: _Runtime, e: float, n: float) -> float:
        """Terrain height under the robot, once its DEM block has streamed in."""
        from dimos.visualization.citymesh.tiles import tile_of

        terrain = runtime.streamer.builder.peek_dem(tile_of(e, n))
        if terrain is None:
            return 0.0
        import numpy as np

        lat, lon, _ = runtime.frame.enu_to_geodetic(np.array([[e, n, 0.0]]))
        ground = float(terrain.sample_ground_msl(lon, lat)[0])
        return ground - runtime.frame.origin_msl + self.marker_radius_m

    def _start(self, msg: Any) -> _Runtime:
        """First fix: anchor the frame, declare the scene, start the streamer."""
        from dimos.visualization.citymesh.frame import EnuFrame
        from dimos.visualization.citymesh.themes import load_theme
        from dimos.visualization.citymesh.tiles import TileStreamer
        from dimos.visualization.citymesh.viz import RerunTileSink, init_world

        origin_lat, origin_lon = (
            self.anchors[0] if self.anchors else snap_origin(msg.latitude, msg.longitude)
        )
        # Sea-level origin, always: the scene is absolute-MSL (terrain and
        # marker at their true elevations) and the origin — hence the fetch
        # cache — never depends on what altitude this particular fix carried.
        frame = EnuFrame.at(
            origin_lat,
            origin_lon,
            0.0,
            datum=self.datum,
            undulation=self.geoid_undulation,
        )
        theme = load_theme(self.theme)
        sink = RerunTileSink(frame, theme=theme, root=self.root)
        streamer = TileStreamer(
            frame,
            sink,
            theme=theme,
            source=self.source,
            load_radius_m=self.load_radius_m,
            unload_radius_m=self.unload_radius_m,
            workers=self.workers,
            flat_ground=self.flat_ground,
            pacing="live",
        )
        init_world(frame, root=self.root)

        anchors_enu: list[tuple[float, float]] = []
        for lat, lon in self.anchors:
            e, n, _ = frame.geodetic_to_enu(lat, lon, frame.origin_msl, datum="msl")[0]
            anchors_enu.append((float(e), float(n)))
        if anchors_enu:
            self._log_anchors(anchors_enu)

        runtime = _Runtime(frame=frame, streamer=streamer, anchors_enu=anchors_enu)
        self._runtime = runtime
        logger.info(
            "citymesh anchored",
            lat=origin_lat,
            lon=origin_lon,
            theme=self.theme,
            source=self.source,
            anchors=len(anchors_enu),
        )
        return runtime

    def _log_anchors(self, anchors_enu: list[tuple[float, float]]) -> None:
        import rerun as rr

        rr.log(
            f"{self.root}/anchors",
            rr.Points3D(
                positions=[(e, n, 0.0) for e, n in anchors_enu],
                radii=[self.marker_radius_m * 2],
                colors=[(255, 200, 60)],
            ),
            static=True,
        )

    def _log_robot(self, runtime: _Runtime, enu: tuple[float, float, float]) -> None:
        import rerun as rr

        from dimos.visualization.citymesh.viz import log_trail

        rr.log(
            f"{self.root}/robot",
            rr.Points3D(positions=[enu], radii=[self.marker_radius_m], colors=[(255, 90, 60)]),
        )
        runtime.trail.append(enu)
        if len(runtime.trail) >= 2:
            import numpy as np

            log_trail(
                f"{self.root}/trail",
                np.asarray(runtime.trail),
                theme=runtime.streamer.theme,
            )
