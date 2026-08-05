# Copyright 2025-2026 Dimensional Inc.
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

from dimos.simulation import providers
from dimos.simulation.providers import SimulationBinding, SimulationRequest


class _Provider:
    def build(self, request: SimulationRequest) -> SimulationBinding:
        raise NotImplementedError


class _EntryPoint:
    name = "test"

    def __init__(self, value: Any | None = None) -> None:
        self.value = _Provider() if value is None else value

    def load(self) -> Any:
        return self.value


def test_load_external_simulation_provider(monkeypatch: pytest.MonkeyPatch) -> None:
    def entry_points(*, group: str, name: str | None = None) -> list[_EntryPoint]:
        assert group == providers.ENTRY_POINT_GROUP
        return [_EntryPoint()] if name in (None, "test") else []

    monkeypatch.setattr(providers.importlib_metadata, "entry_points", entry_points)

    assert isinstance(providers.load_simulation_provider("test"), _Provider)


def test_missing_provider_lists_available_names(monkeypatch: pytest.MonkeyPatch) -> None:
    available = _EntryPoint()
    available.name = "other"
    monkeypatch.setattr(
        providers.importlib_metadata,
        "entry_points",
        lambda **kwargs: [] if "name" in kwargs else [available],
    )

    with pytest.raises(ValueError, match="not installed.*other"):
        providers.load_simulation_provider("test")


def test_duplicate_provider_is_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        providers.importlib_metadata,
        "entry_points",
        lambda **kwargs: [_EntryPoint(), _EntryPoint()],
    )

    with pytest.raises(ValueError, match="registered more than once"):
        providers.load_simulation_provider("test")


def test_incompatible_provider_is_rejected(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        providers.importlib_metadata,
        "entry_points",
        lambda **kwargs: [_EntryPoint(value=object())],
    )

    with pytest.raises(TypeError, match="must implement SimulationProvider"):
        providers.load_simulation_provider("test")
