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

from __future__ import annotations

import time

import pytest

from dimos.core.stream import In
from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.tracer import Tracer


class OdomTracer(Tracer):
    odometry: In[Odometry]
    clicked: In[PointStamped]


@pytest.fixture
def tracer():  # type: ignore[no-untyped-def]
    tracer = OdomTracer()
    transports = []
    for name, port in tracer.inputs.items():
        transport = pLCMTransport(f"/test/tracer/{name}")
        transport.start()
        port.transport = transport
        transports.append(transport)
    tracer.start()
    yield tracer
    tracer.stop()
    for transport in transports:
        transport.stop()


def odom(x: float, y: float, ts: float = 1.0) -> Odometry:
    return Odometry(ts=ts, frame_id="odom", pose=Pose(x, y, 0.0))


def publish(tracer: OdomTracer, name: str, msg: object) -> None:
    tracer.inputs[name].transport.publish(msg)


def wait_for(paths: list[Path], count: int, timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while len(paths) < count and time.monotonic() < deadline:
        time.sleep(0.01)
    assert len(paths) >= count, f"expected {count} paths, got {len(paths)}"


def test_path_outputs_injected(tracer: OdomTracer) -> None:
    assert set(tracer.inputs) == {"odometry", "clicked"}
    assert set(tracer.outputs) == {"odometry_path", "clicked_path"}
    assert all(out.type is Path for out in tracer.outputs.values())


def test_unsupported_input_type_raises() -> None:
    with pytest.raises(TypeError, match="inputs must be"):

        class BadTracer(Tracer):
            image: In[Image]


def test_traces_at_resolution(tracer: OdomTracer) -> None:
    paths: list[Path] = []
    tracer.outputs["odometry_path"].subscribe(paths.append)

    publish(tracer, "odometry", odom(0.0, 0.0, ts=1.0))
    wait_for(paths, 1)
    publish(tracer, "odometry", odom(0.1, 0.0, ts=2.0))  # < 0.25 m, dropped
    publish(tracer, "odometry", odom(0.3, 0.0, ts=3.0))
    wait_for(paths, 2)
    publish(tracer, "odometry", odom(0.3, 0.24, ts=4.0))  # < 0.25 m, dropped
    publish(tracer, "odometry", odom(0.3, 0.26, ts=5.0))
    wait_for(paths, 3)

    assert [len(p) for p in paths] == [1, 2, 3]
    final = paths[-1]
    assert final.frame_id == "odom"
    assert final.ts == 5.0
    assert [(p.x, p.y) for p in final.poses] == [(0.0, 0.0), (0.3, 0.0), (0.3, 0.26)]


def test_traces_point_stamped(tracer: OdomTracer) -> None:
    paths: list[Path] = []
    tracer.outputs["clicked_path"].subscribe(paths.append)

    publish(tracer, "clicked", PointStamped(0.0, 0.0, 0.0, ts=1.0, frame_id="world"))
    wait_for(paths, 1)
    publish(tracer, "clicked", PointStamped(1.0, 1.0, 0.0, ts=2.0, frame_id="world"))
    wait_for(paths, 2)

    assert [len(p) for p in paths] == [1, 2]
    assert paths[-1].poses[-1].x == 1.0
    assert paths[-1].frame_id == "world"
