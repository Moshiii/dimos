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

from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.core.module import ModuleBase
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.memory.prompted_object_localizer import (
    PromptedObjectLocalizationRuntime,
    PromptedObjectLocalizerConfig,
    PromptedObjectLocalizerModule,
    latest_recording_window,
    strongest_detection,
)
from dimos.perception.memory.tool_localize import (
    PromptedObjectLocalizationRuntime as ToolLocalizationRuntime,
)


def test_latest_recording_window_is_bounded_by_available_history(mocker: MockerFixture) -> None:
    stream = mocker.Mock()
    stream.get_time_range.side_effect = [(10.0, 100.0), (95.0, 110.0)]

    assert latest_recording_window(stream, 30.0) == (70.0, 100.0)
    assert latest_recording_window(stream, 30.0) == (95.0, 110.0)


def test_strongest_detection_returns_largest_reconstructed_cloud() -> None:
    small = SimpleNamespace(pointcloud=[1])
    large = SimpleNamespace(pointcloud=[1, 2, 3])
    observations = [
        SimpleNamespace(data=[small]),
        SimpleNamespace(data=[]),
        SimpleNamespace(data=[large]),
    ]

    assert strongest_detection(observations) is large
    assert strongest_detection([]) is None


@pytest.fixture
def localizer() -> PromptedObjectLocalizerModule:
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = PromptedObjectLocalizerModule()
    module.config = PromptedObjectLocalizerConfig(db_path="/tmp/prompted-localizer-test.db")
    return module


def _cloud() -> PointCloud2:
    return PointCloud2.from_numpy(
        np.asarray([[0.4, 0.0, 0.2]], dtype=np.float32),
        frame_id="world",
        timestamp=100.0,
    )


def test_localizer_uses_active_recording_and_latest_window(
    localizer: PromptedObjectLocalizerModule,
    mocker: MockerFixture,
) -> None:
    cloud = _cloud()
    localizer.recorder = mocker.Mock()
    localizer.recorder.recording_path.return_value = "/tmp/active.db"
    color_stream = mocker.Mock()
    color_stream.get_time_range.return_value = (10.0, 100.0)
    store = SimpleNamespace(streams=SimpleNamespace(color_image=color_stream))
    store_context = mocker.MagicMock()
    store_context.__enter__.return_value = store
    sqlite = mocker.patch(
        "dimos.perception.memory.prompted_object_localizer.SqliteStore",
        return_value=store_context,
    )
    runtime = mocker.MagicMock()
    runtime.__enter__.return_value = runtime
    runtime.localize.return_value = SimpleNamespace(pointcloud=cloud)
    runtime_factory = mocker.patch(
        "dimos.perception.memory.prompted_object_localizer.PromptedObjectLocalizationRuntime",
        return_value=runtime,
    )

    result = localizer.localize("white and red marker")

    assert result is cloud
    sqlite.assert_called_once_with(path="/tmp/active.db", must_exist=True)
    runtime_factory.assert_called_once_with(
        store,
        optical_frame="camera_color_optical_frame",
        report=mocker.ANY,
    )
    runtime.localize.assert_called_once_with("white and red marker", 70.0, 100.0)


def test_localizer_returns_none_without_running_models_for_empty_recording(
    localizer: PromptedObjectLocalizerModule,
    mocker: MockerFixture,
) -> None:
    localizer.recorder = mocker.Mock()
    localizer.recorder.recording_path.return_value = "/tmp/active.db"
    color_stream = mocker.Mock()
    color_stream.get_time_range.side_effect = LookupError
    store = SimpleNamespace(streams=SimpleNamespace(color_image=color_stream))
    store_context = mocker.MagicMock()
    store_context.__enter__.return_value = store
    mocker.patch(
        "dimos.perception.memory.prompted_object_localizer.SqliteStore",
        return_value=store_context,
    )
    runtime_factory = mocker.patch(
        "dimos.perception.memory.prompted_object_localizer.PromptedObjectLocalizationRuntime"
    )

    assert localizer.localize("marker") is None
    runtime_factory.assert_not_called()


def test_debug_tool_uses_the_same_localization_runtime() -> None:
    assert ToolLocalizationRuntime is PromptedObjectLocalizationRuntime
