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

"""Entity paths the tf triads are drawn at. Rendering itself needs a human."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, cast
from unittest.mock import patch

import pytest
import rerun as rr

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.tf_tree import RerunTFTree, TFTreeVis

if TYPE_CHECKING:
    from collections.abc import Iterator

# tf_tree.py logs through this stdlib logger name (setup_logger() derives it
# from the module's file path).
_TF_TREE_LOGGER = "dimos/visualization/rerun/tf_tree.py"


def edge(parent: str, child: str, ts: float = 1.0) -> Transform:
    return Transform(frame_id=parent, child_frame_id=child, ts=ts)


@dataclass
class Topic:
    name: str


def _bridge_tf_paths(tf_axes: float) -> list[str]:
    """Entity paths a bridge logs for the same two tf messages."""
    bridge = RerunBridgeModule(tf_axes=tf_axes)
    bridge._min_intervals = {}
    try:
        with patch("rerun.log") as mock_log:
            bridge._on_message(TFMessage(edge("odom", "base_link")), Topic("/tf"))
            bridge._on_message(TFMessage(edge("odom", "base_link", ts=2.0)), Topic("/tf"))
    finally:
        bridge.stop()
    return [call.args[0] for call in mock_log.call_args_list]


@pytest.fixture
def recording() -> Iterator[None]:
    """Give ``rr.log`` somewhere to write, so nothing reaches a real viewer."""
    import rerun as rr

    with rr.RecordingStream("dimos_test_tf_tree"):
        yield


@pytest.fixture
def tf_warnings(caplog: pytest.LogCaptureFixture) -> Iterator[pytest.LogCaptureFixture]:
    """Capture tf_tree log lines via ``caplog``.

    The dimos logger is structlog over a stdlib logger with
    ``propagate=False``, so caplog's root-level handler never sees it.
    """
    lg = logging.getLogger(_TF_TREE_LOGGER)
    lg.addHandler(caplog.handler)
    caplog.set_level(logging.WARNING, logger=_TF_TREE_LOGGER)
    try:
        yield caplog
    finally:
        lg.removeHandler(caplog.handler)


def test_paths_mirror_the_tree() -> None:
    vis = TFTreeVis()
    vis.buffer.receive_tfmessage(
        TFMessage(edge("odom", "base_link"), edge("base_link", "mid360_link"))
    )

    assert vis.path("mid360_link") == "world/tf/odom/base_link/mid360_link"
    assert vis.path("base_link") == "world/tf/odom/base_link"


def test_root_frame_gets_a_path() -> None:
    vis = TFTreeVis()
    vis.buffer.receive_tfmessage(TFMessage(edge("odom", "base_link")))

    assert vis.path("odom") == "world/tf/odom"


def test_root_honors_the_configured_prefix() -> None:
    vis = TFTreeVis(root="scene/tf")
    vis.buffer.receive_tfmessage(TFMessage(edge("odom", "base_link")))

    assert vis.path("base_link") == "scene/tf/odom/base_link"


def test_frame_names_are_escaped() -> None:
    vis = TFTreeVis()
    vis.buffer.receive_tfmessage(TFMessage(edge("odom", "camera/optical")))

    assert vis.path("camera/optical") == "world/tf/odom/camera\\/optical"


def test_late_reroot_leaves_paths_alone() -> None:
    vis = TFTreeVis()
    vis.buffer.receive_tfmessage(TFMessage(edge("odom", "base_link")))
    assert vis.path("base_link") == "world/tf/odom/base_link"

    vis.buffer.receive_tfmessage(TFMessage(edge("map", "odom")))

    assert vis.path("odom") == "world/tf/odom"
    assert vis.path("base_link") == "world/tf/odom/base_link"
    assert vis.path("map") == "world/tf/map"


def test_settle_window_waits_for_the_whole_tree(recording: None) -> None:
    """A tf tree arrives edge by edge, in whatever order its publishers start."""
    vis = TFTreeVis(settle=1.0)
    vis.log(TFMessage(edge("base_link", "front_camera")))
    vis.log(TFMessage(edge("mid360_link", "base_link")))
    vis.log(TFMessage(edge("odom", "mid360_link")))
    assert vis.frame_paths() == {}

    vis.log(TFMessage(edge("odom", "mid360_link", ts=2.0)))

    assert vis.path("front_camera") == "world/tf/odom/mid360_link/base_link/front_camera"


def test_settle_window_does_not_swallow_a_one_shot_edge(recording: None) -> None:
    """``world -> map`` shows up twice at startup in a real recording, then never again."""
    vis = TFTreeVis(settle=1.0)
    vis.log(TFMessage(edge("world", "map")))
    vis.log(TFMessage(edge("map", "odom"), edge("odom", "base_link")))

    vis.log(TFMessage(edge("odom", "base_link", ts=2.0)))

    assert vis._axes_logged == {
        "world/tf/world",
        "world/tf/world/map",
        "world/tf/world/map/odom",
        "world/tf/world/map/odom/base_link",
    }


def test_slash_in_a_frame_name_is_not_a_reparent(
    recording: None, tf_warnings: pytest.LogCaptureFixture
) -> None:
    """The escaped name keeps its slash, so the path cannot be split to find the parent."""
    vis = TFTreeVis(settle=0.0)
    vis.log(TFMessage(edge("odom", "camera/optical")))

    assert [r for r in tf_warnings.records if "re-parented" in r.getMessage()] == []


def test_reparent_of_a_slashed_frame_still_warns(
    recording: None, tf_warnings: pytest.LogCaptureFixture
) -> None:
    vis = TFTreeVis(settle=0.0)
    vis.log(TFMessage(edge("odom", "camera/optical")))
    vis.log(TFMessage(edge("base_link", "camera/optical", ts=2.0)))

    assert len([r for r in tf_warnings.records if "re-parented" in r.getMessage()]) == 1


def test_reparented_frame_warns_once(
    recording: None, tf_warnings: pytest.LogCaptureFixture
) -> None:
    vis = TFTreeVis(settle=0.0)
    vis.log(TFMessage(edge("odom", "base_link")))
    vis.log(TFMessage(edge("chassis", "base_link", ts=2.0)))
    vis.log(TFMessage(edge("chassis", "base_link", ts=3.0)))

    warnings = [r for r in tf_warnings.records if "re-parented" in r.getMessage()]
    assert len(warnings) == 1
    assert vis.path("base_link") == "world/tf/odom/base_link"


def test_every_frame_gets_axes_once(recording: None) -> None:
    vis = TFTreeVis(settle=0.0)
    vis.log(TFMessage(edge("odom", "base_link"), edge("base_link", "mid360_link")))
    vis.log(TFMessage(edge("odom", "base_link", ts=2.0)))

    assert vis._axes_logged == {
        "world/tf/odom",
        "world/tf/odom/base_link",
        "world/tf/odom/base_link/mid360_link",
    }


def _arrow_length(arrows: rr.Arrows3D) -> float:
    assert arrows.vectors is not None
    return float(max(arrows.vectors.as_arrow_array().to_pylist()[0]))


def test_triads_shrink_with_depth() -> None:
    vis = TFTreeVis(axis_length=1.0, settle=0.0)
    with patch("rerun.log") as mock_log:
        vis.log(TFMessage(edge("odom", "base_link"), edge("base_link", "mid360_link")))

    lengths = {
        call.args[0]: _arrow_length(arrows)
        for call in mock_log.call_args_list
        for arrows in call.args[1:]
        if isinstance(arrows, rr.Arrows3D)
    }
    assert lengths == pytest.approx(
        {
            "world/tf/odom": 1.0,
            "world/tf/odom/base_link": 0.8,
            "world/tf/odom/base_link/mid360_link": 0.64,
        }
    )


class FakeStream:
    """Re-iterable stand-in for a memory2 stream of tf observations."""

    def __init__(self, *stamped: tuple[float, TFMessage]) -> None:
        self._stamped = stamped

    def __iter__(self) -> Iterator[SimpleNamespace]:
        return iter([SimpleNamespace(ts=ts, data=msg) for ts, msg in self._stamped])


def _drive(tf: FakeStream, stamps: list[float]) -> list[tuple[float, str]]:
    """Replay ``stamps`` through the transformer, returning (time, entity) in log order."""
    upstream = iter([SimpleNamespace(ts=ts) for ts in stamps])
    events: list[tuple[float, str]] = []
    now = [0.0]

    def set_time(_timeline: str, *, timestamp: float) -> None:
        now[0] = timestamp

    with (
        patch("rerun.set_time", side_effect=set_time),
        patch("rerun.log", side_effect=lambda path, *a, **k: events.append((now[0], path))),
    ):
        list(cast("Any", RerunTFTree(cast("Any", tf)))(cast("Any", upstream)))
    return events


def test_transformer_logs_tf_in_step_with_the_stream() -> None:
    tf = FakeStream(
        (1.0, TFMessage(edge("odom", "base_link"))),
        (2.0, TFMessage(edge("odom", "base_link", ts=2.0))),
        (3.0, TFMessage(edge("odom", "base_link", ts=3.0))),
    )

    stamps = [t for t, _ in _drive(tf, [1.5, 2.5, 3.5])]

    assert stamps == sorted(stamps)
    assert set(stamps) == {1.0, 2.0, 3.0}


def test_transformer_does_not_run_ahead_of_the_stream() -> None:
    tf = FakeStream(
        (1.0, TFMessage(edge("odom", "base_link"))),
        (9.0, TFMessage(edge("odom", "base_link", ts=9.0))),
    )

    # Upstream stops at 2.0, so the tf message at 9.0 is outside the replay.
    assert {t for t, _ in _drive(tf, [2.0])} == {1.0}


def test_transformer_nests_from_the_whole_stream() -> None:
    """Topology is read up front, so the first message already knows its parents."""
    tf = FakeStream(
        (1.0, TFMessage(edge("base_link", "front_camera"))),
        (1.0, TFMessage(edge("odom", "base_link", ts=1.0))),
    )

    paths = {path for _, path in _drive(tf, [5.0])}

    assert "world/tf/odom/base_link/front_camera" in paths


def test_bridge_nests_tf_when_axes_are_on() -> None:
    assert _bridge_tf_paths(tf_axes=0.4) == [
        "world/tf/odom",
        "world/tf/odom",
        "world/tf/odom/base_link",
        "world/tf/odom/base_link",
    ]


def test_bridge_leaves_tf_flat_when_axes_are_off() -> None:
    assert _bridge_tf_paths(tf_axes=0.0) == ["world/tf/base_link", "world/tf/base_link"]
