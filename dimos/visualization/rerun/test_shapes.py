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

import pickle
from unittest.mock import patch

import numpy as np
import rerun as rr

from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.shapes import quadcopter


def test_quadcopter_parts() -> None:
    parts = dict(quadcopter(arm_m=2.0)(rr))

    assert list(parts) == ["frame", "hub"]
    assert isinstance(parts["frame"], rr.LineStrips3D)
    assert isinstance(parts["hub"], rr.Boxes3D)

    strips = parts["frame"].strips.as_arrow_array()  # type: ignore[union-attr]
    assert len(strips) == 7  # 2 arms + 4 rotors + nose
    assert len(parts["frame"].colors.as_arrow_array()) == 7  # type: ignore[union-attr]


def test_quadcopter_scales_with_arm() -> None:
    def nose_tip(arm_m: float) -> float:
        strips = dict(quadcopter(arm_m=arm_m)(rr))["frame"].strips.as_arrow_array().to_pylist()
        return float(np.asarray(strips[-1])[1][0])

    assert nose_tip(4.0) == 2 * nose_tip(2.0)


def test_shape_pickles_to_the_bridge_worker() -> None:
    shape = pickle.loads(pickle.dumps(quadcopter(arm_m=2.0)))

    assert [subpath for subpath, _ in shape(rr)] == ["frame", "hub"]


def test_models_log_shape_parts_under_the_frame_attach() -> None:
    bridge = RerunBridgeModule(models={"drone/base_link": quadcopter()})

    try:
        with patch("dimos.visualization.rerun.bridge.rr.log") as mock_log:
            bridge._log_static()
    finally:
        bridge.stop()

    logged = {call.args[0]: call.args[1] for call in mock_log.call_args_list}
    assert set(logged) == {
        "world/models/drone/base_link",
        "world/models/drone/base_link/frame",
        "world/models/drone/base_link/hub",
    }
    assert all(call.kwargs["static"] for call in mock_log.call_args_list)

    attach = logged["world/models/drone/base_link"]
    descriptors = {str(b.component_descriptor()) for b in attach.as_component_batches()}
    assert any("Transform3D:parent_frame" in d for d in descriptors)
