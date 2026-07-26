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

"""Offline tests for *when* a tile is logged, as opposed to what is in it.

A recording generated from a route runs tens of times faster than the flight it
describes, so a tile that takes half a second to build finishes tens of
flight-seconds after the vehicle asked for it. Logged at the moment it finished,
it appears far behind the vehicle on replay — or not at all, if the vehicle has
by then moved out of range. The fix is to log it at the flight time it was
*enqueued*; these tests pin that down.

No viewer and no network: ``rr.log`` and ``rr.set_time`` are replaced by a
recorder, so every log carries the timeline position it would have been written
at. Patching them on the ``rerun`` module reaches every caller, since
:mod:`citymesh.tiles` and :mod:`citymesh.viz` both hold that same module object.

The slow build is simulated by harvesting finished tiles without logging them
(``_collect(budget=0)``) and letting a later fix flush them — which is exactly
what the real streamer does when a tile lands between two fixes.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest
import rerun as rr

from dimos.visualization.citymesh.extrude import Mesh
from dimos.visualization.citymesh.frame import EnuFrame
from dimos.visualization.citymesh.themes import THEMES
from dimos.visualization.citymesh.tiles import FLIGHT_TIMELINE, TileData, TileKey, TileStreamer
from dimos.visualization.citymesh.viz import RerunTileSink

TILES_ROOT = "world/tiles"


@pytest.fixture
def frame() -> EnuFrame:
    return EnuFrame.at(37.9938, 23.7253, 0.0, datum="ellipsoidal", undulation=0.0)


@dataclass
class _Event:
    path: str
    t: float | None
    kind: str


class _Recorder:
    """Stands in for rerun, remembering the timeline position of every log."""

    def __init__(self) -> None:
        self.now: float | None = None
        self.events: list[_Event] = []

    def set_time(self, timeline: str, duration=None, **kwargs) -> None:
        self.now = None if duration is None else float(duration)

    def log(self, path: str, *args, **kwargs) -> None:
        self.events.append(
            _Event(path=path, t=self.now, kind=type(args[0]).__name__ if args else "?")
        )

    def times(self, fragment: str, kind: str | None = None) -> list[float | None]:
        return [e.t for e in self.events if fragment in e.path and (kind is None or e.kind == kind)]


@pytest.fixture
def rec(monkeypatch) -> _Recorder:
    recorder = _Recorder()
    monkeypatch.setattr(rr, "set_time", recorder.set_time)
    monkeypatch.setattr(rr, "log", recorder.log)
    return recorder


class _StubBuilder:
    """Instant, offline, and just enough geometry to produce a log call."""

    flat_ground = True

    def __init__(self) -> None:
        self.built = 0

    def build(self, key: TileKey) -> TileData:
        self.built += 1
        return TileData(
            key=key,
            terrain=Mesh(
                vertices=np.zeros((3, 3), dtype=np.float32),
                triangles=np.array([[0, 1, 2]], dtype=np.uint32),
            ),
            n_buildings=0,
        )


def _streamer(frame: EnuFrame, **kwargs) -> TileStreamer:
    streamer = TileStreamer(
        frame,
        RerunTileSink(frame, theme=THEMES["blueprint"]),
        theme=THEMES["blueprint"],
        flat_ground=True,
        load_radius_m=300.0,
        unload_radius_m=450.0,
        max_logs_per_update=1,
        **kwargs,
    )
    streamer.builder = _StubBuilder()
    return streamer


def _fix(streamer: TileStreamer, e: float, t: float) -> None:
    """One fix, exactly as the fly loop issues it: set the time, then update."""
    rr.set_time(FLIGHT_TIMELINE, duration=t)
    streamer.update([e, 0.0, 60.0], t_s=t)


def _build_without_logging(streamer: TileStreamer) -> None:
    """Let the workers finish, but log nothing — the tile is now 'late'."""
    streamer._collect(budget=0, block_s=0.05)


def _hover(streamer: TileStreamer, e: float, first_t: float, steps: int = 30) -> None:
    for i in range(steps):
        _fix(streamer, e, first_t + i)


# -- backdating ------------------------------------------------------------


def test_offline_pacing_logs_a_tile_at_the_time_it_was_requested(frame, rec):
    """The bug: tiles requested at t=0 must not land tens of seconds later."""
    streamer = _streamer(frame, pacing="offline")
    _fix(streamer, 0.0, 0.0)  # every tile enqueued here
    _build_without_logging(streamer)
    _hover(streamer, 0.0, first_t=10.0)
    streamer.close(drain_timeout_s=5.0)

    logged = rec.times("world/tiles/", kind="Mesh3D")
    assert len(logged) > 1, "the stub tiles should have been logged"
    assert set(logged) == {0.0}, "every tile belongs at the fix that asked for it"


def test_live_pacing_logs_a_tile_when_it_arrives(frame, rec):
    """In the field the two clocks are the same one, and there is nothing to undo."""
    streamer = _streamer(frame, pacing="live")
    _fix(streamer, 0.0, 0.0)
    _build_without_logging(streamer)
    _hover(streamer, 0.0, first_t=10.0)
    streamer.close(drain_timeout_s=5.0)

    logged = [t for t in rec.times("world/tiles/", kind="Mesh3D") if t is not None]
    assert len(logged) > 1
    assert min(logged) >= 10.0, "live pacing must not rewind the timeline"


def test_the_timeline_is_put_back_after_a_backdated_tile(frame, rec):
    """Whatever the caller logs next must land at the present, not in the past."""
    streamer = _streamer(frame, pacing="offline")
    _fix(streamer, 0.0, 0.0)
    _build_without_logging(streamer)
    _fix(streamer, 0.0, 99.0)

    assert 0.0 in rec.times("world/tiles/"), "a tile was backdated"
    assert rec.now == 99.0, "and the timeline is back at the present"


# -- tiles that arrive after the vehicle has gone --------------------------


def test_a_late_tile_is_shown_for_the_interval_the_vehicle_was_there(frame, rec):
    """Backdated in at its enqueue time, cleared at the present: a finite window.

    Logging it plainly would leave it hanging in the world behind the vehicle
    forever, since nothing revisits a tile to unload it.
    """
    streamer = _streamer(frame, pacing="offline")
    _fix(streamer, 0.0, 0.0)  # enqueue around the origin
    _build_without_logging(streamer)  # they finish, unlogged
    _hover(streamer, 5000.0, first_t=50.0)  # the drone is long gone
    streamer.close(drain_timeout_s=5.0)

    assert streamer.stats.late > 0
    assert streamer.stats.dropped == 0

    entity = f"{TILES_ROOT}/{TileKey(0, 0)}"
    shown = [e.t for e in rec.events if e.path.startswith(entity) and e.kind != "Clear"]
    cleared = [e.t for e in rec.events if e.path == entity and e.kind == "Clear"]
    assert shown == [0.0], "the tile belongs at the moment it was asked for"
    assert cleared, "a late tile must not be left hanging in the world"
    assert min(cleared) > shown[0], "cleared after it was shown, not before"
    assert min(cleared) >= 50.0, "cleared once the drone had actually left"


def test_a_late_tile_is_not_marked_live(frame, rec):
    """It was logged and cleared; the policy must be free to request it again."""
    streamer = _streamer(frame, pacing="offline")
    _fix(streamer, 0.0, 0.0)
    _build_without_logging(streamer)
    _hover(streamer, 5000.0, first_t=50.0)

    assert TileKey(0, 0) not in streamer.live_tiles()
    streamer.close(drain_timeout_s=5.0)


def test_live_pacing_drops_a_late_tile_instead(frame, rec):
    """Its moment is genuinely past; there is nothing useful to write."""
    streamer = _streamer(frame, pacing="live")
    _fix(streamer, 0.0, 0.0)
    _build_without_logging(streamer)
    _hover(streamer, 5000.0, first_t=50.0)
    streamer.close(drain_timeout_s=5.0)

    assert streamer.stats.dropped > 0
    assert streamer.stats.late == 0
    entity = f"{TILES_ROOT}/{TileKey(0, 0)}"
    assert not [e for e in rec.events if e.path.startswith(entity)]


# -- what must NOT be backdated -------------------------------------------


def test_the_micro_carpet_stays_at_the_current_fix(frame, rec):
    """The carpet tracks the vehicle, not a fetch: backdating it would strand it."""
    streamer = _streamer(frame, pacing="offline")
    _fix(streamer, 0.0, 0.0)
    _build_without_logging(streamer)
    _fix(streamer, 5000.0, 50.0)
    streamer.close(drain_timeout_s=5.0)

    carpet_at = rec.times("world/grid/micro")
    assert carpet_at, "the blueprint theme has a micro layer"
    assert carpet_at[0] == 0.0
    assert 50.0 in carpet_at, "it moved with the drone, at the drone's own time"
    assert 0.0 in rec.times("world/tiles/"), "while a tile was backdated alongside"


def test_unloading_clears_at_the_present_not_at_the_build_time(frame, rec):
    """Leaving a tile happens now; only its arrival is rewound."""
    streamer = _streamer(frame, pacing="offline")
    for i, e in enumerate(range(0, 2000, 100)):
        _build_without_logging(streamer)
        _fix(streamer, float(e), float(i))
    streamer.close(drain_timeout_s=5.0)

    assert streamer.stats.unloaded > 0
    entity = f"{TILES_ROOT}/{TileKey(0, 0)}"
    cleared = [e.t for e in rec.events if e.path == entity and e.kind == "Clear"]
    assert cleared, "the traverse should have unloaded the tile it started on"
    assert min(cleared) > 0.0, "the drone was there at t=0; it left later"


def test_a_streamer_given_no_times_still_works(frame, rec):
    """``t_s`` is optional: with nothing to backdate to, tiles land where they land."""
    streamer = _streamer(frame, pacing="offline")
    for e in range(0, 800, 100):
        _build_without_logging(streamer)
        streamer.update([float(e), 0.0, 60.0])
    streamer.close(drain_timeout_s=5.0)

    assert streamer.stats.loaded > 0
    assert set(rec.times("world/tiles/")) == {None}, "the timeline was never touched"
