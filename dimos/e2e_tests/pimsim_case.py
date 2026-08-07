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

from collections.abc import Mapping
from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any

import pytest

from dimos.e2e_tests.scene_control import EpisodeSceneControl


@dataclass(frozen=True)
class PimSimTabletopCase:
    """One explicit row in the DimOS native tabletop evaluation suite."""

    case_id: str
    family_id: str
    scene_seed: int
    variation_seed: int
    semantic_roles: Mapping[str, str]
    object_count: int = 2

    def to_scenario_request(self, authoring: Any) -> Any:
        return authoring.ScenarioRequest(
            case_id=self.case_id,
            family_id=self.family_id,
            scene_seed=self.scene_seed,
            variation_seed=self.variation_seed,
            role_constraints={
                role_id: authoring.AssetQuery(semantic_classes=(semantic_class,))
                for role_id, semantic_class in self.semantic_roles.items()
            },
        )


@dataclass(frozen=True)
class PimSimCaseRun:
    """Materialized case and live DimOS interfaces supplied to one test."""

    case: PimSimTabletopCase
    scene_package: Path
    scenario: Path
    reset: Mapping[str, Any]
    pick_and_place: Any
    scene_control: EpisodeSceneControl
    role_names: Mapping[str, str]

    def role(self, role_id: str) -> str:
        try:
            return self.role_names[role_id]
        except KeyError as error:
            available = ", ".join(sorted(self.role_names))
            raise KeyError(f"unknown role {role_id!r}; available: {available}") from error

    def wait_for_role(self, role_id: str, timeout: float = 20.0) -> Any:
        name = self.role(role_id)
        deadline = time.monotonic() + timeout
        last_observation: Any = None
        while time.monotonic() < deadline:
            last_observation = self.pick_and_place.look()
            if name in last_observation.message:
                return last_observation
            time.sleep(0.25)
        pytest.fail(f"role {role_id!r} ({name!r}) was not observed: {last_observation}")

    def evaluate_goal(self) -> dict[str, Any]:
        return dict(self.scene_control.evaluate_goal())

    def wait_for_goal(self, timeout: float = 5.0) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        evaluation: dict[str, Any] = {}
        while time.monotonic() < deadline:
            evaluation = self.evaluate_goal()
            if evaluation["passed"]:
                return evaluation
            time.sleep(0.1)
        return evaluation


def semantic_role_names(scene_package: Path, scenario: Path) -> dict[str, str]:
    scenario_raw = json.loads(scenario.read_text(encoding="utf-8"))
    package_raw = json.loads(scene_package.read_text(encoding="utf-8"))
    semantic_classes = {
        str(entity["entity_id"]): str(entity["semantic_class"])
        for entity in package_raw["entities"]
    }
    names: dict[str, str] = {}
    for role_id, entity_id in scenario_raw["role_bindings"].items():
        try:
            names[str(role_id)] = semantic_classes[str(entity_id)]
        except KeyError as error:
            raise ValueError(
                f"scenario role {role_id!r} references unknown entity {entity_id!r}"
            ) from error
    return names


__all__ = ["PimSimCaseRun", "PimSimTabletopCase", "semantic_role_names"]
