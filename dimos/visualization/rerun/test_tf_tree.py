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

"""Where the triads get drawn, not how they look."""

from __future__ import annotations

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.visualization.rerun.tf_tree import TFTreeVis


def edge(parent: str, child: str) -> Transform:
    return Transform(frame_id=parent, child_frame_id=child, ts=1.0)


def paths(vis: TFTreeVis) -> dict[str, str]:
    return {frame: spot.path for frame, spot in vis.placements().items()}


def feed(vis: TFTreeVis, *messages: TFMessage) -> None:
    """tf republishes, and the tree draws once a message adds nothing new."""
    for msg in (*messages, messages[-1]):
        vis.log(msg, [archetype for _, archetype in msg.to_rerun()])


def test_triads_nest_along_the_tree() -> None:
    vis = TFTreeVis()
    feed(vis, TFMessage(edge("odom", "base_link"), edge("base_link", "camera/optical")))

    assert paths(vis) == {
        "odom": "world/tf/odom",
        "base_link": "world/tf/odom/base_link",
        "camera/optical": "world/tf/odom/base_link/camera\\/optical",
    }


def test_a_late_root_re_parents_the_tree() -> None:
    """The mount tree is published seconds before the odometry that roots it."""
    vis = TFTreeVis()
    feed(vis, TFMessage(edge("mid360_link", "base_link")))
    assert paths(vis)["base_link"] == "world/tf/mid360_link/base_link"

    feed(vis, TFMessage(edge("odom", "mid360_link")))

    assert paths(vis) == {
        "odom": "world/tf/odom",
        "mid360_link": "world/tf/odom/mid360_link",
        "base_link": "world/tf/odom/mid360_link/base_link",
    }


def test_a_cycle_does_not_hang() -> None:
    vis = TFTreeVis()
    feed(vis, TFMessage(edge("a", "b"), edge("b", "a")))

    assert set(paths(vis)) == {"a", "b"}
