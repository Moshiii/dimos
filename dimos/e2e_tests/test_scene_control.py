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

from typing import Any

import pytest

from dimos.e2e_tests.scene_contract import PlanarBounds
import dimos.e2e_tests.scene_control as scene_controls


class _Client:
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def set_agent_position(self, x: float, y: float, z: float = 0.4) -> None: ...
    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None: ...
    def publish_goal(self, x: float, y: float) -> None: ...
    def semantic_object_bounds(self, query: str) -> PlanarBounds:
        return PlanarBounds(min_x=0, min_y=0, max_x=1, max_y=1)


class _EntryPoint:
    name = "test"

    def __init__(self, value: Any) -> None:
        self.value = value

    def load(self) -> Any:
        return self.value


def _install(monkeypatch: pytest.MonkeyPatch, entries: list[_EntryPoint]) -> None:
    def entry_points(*, group: str) -> list[_EntryPoint]:
        assert group == scene_controls.ENTRY_POINT_GROUP
        return entries

    monkeypatch.setattr(scene_controls, "entry_points", entry_points)


def test_load_valid_scene_control(monkeypatch: pytest.MonkeyPatch) -> None:
    _install(monkeypatch, [_EntryPoint(_Client)])
    assert isinstance(scene_controls.load_scene_control("test"), _Client)


@pytest.mark.parametrize("entries", [[], [_EntryPoint(_Client), _EntryPoint(_Client)]])
def test_missing_or_duplicate_scene_control_is_rejected(
    monkeypatch: pytest.MonkeyPatch, entries: list[_EntryPoint]
) -> None:
    _install(monkeypatch, entries)
    with pytest.raises(ValueError, match="expected one scene-control provider"):
        scene_controls.load_scene_control("test")


@pytest.mark.parametrize("value", [object(), lambda: object()])
def test_incompatible_scene_control_is_rejected(
    monkeypatch: pytest.MonkeyPatch, value: Any
) -> None:
    _install(monkeypatch, [_EntryPoint(value)])
    with pytest.raises(TypeError, match="Scene-control provider"):
        scene_controls.load_scene_control("test")
