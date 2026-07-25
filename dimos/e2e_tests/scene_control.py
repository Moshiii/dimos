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

from __future__ import annotations

from importlib.metadata import entry_points
from typing import Protocol

from dimos.e2e_tests.dim_sim_client import DimSimClient


class SceneControl(Protocol):
    def start(self) -> None: ...

    def stop(self) -> None: ...

    def set_agent_position(self, x: float, y: float, z: float = 0.4) -> None: ...

    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None: ...

    def publish_goal(self, x: float, y: float) -> None: ...


def load_scene_control(simulator: str) -> SceneControl:
    if simulator == "dimsim":
        return DimSimClient()
    matches = [
        entry
        for entry in entry_points(group="dimos.simulation.scene_controls")
        if entry.name == simulator
    ]
    if len(matches) != 1:
        raise ValueError(
            f"expected one scene-control provider for {simulator!r}, found {len(matches)}"
        )
    client = matches[0].load()()
    return client


__all__ = ["SceneControl", "load_scene_control"]
