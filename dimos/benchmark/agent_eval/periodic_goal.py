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

"""Private periodic-goal loading and semantic proximity evaluation."""

from __future__ import annotations

import hashlib
from pathlib import Path

from pydantic import Field

from dimos.benchmark.agent_eval.base import BaseEvalModel
from dimos.benchmark.agent_eval.case import (
    EvalCase,
    PeriodicGoalValidatorRef,
    SemanticObjectProximityGoal,
)
from dimos.e2e_tests.scene_contract import PlanarBounds


class SemanticGoalObservation(BaseEvalModel):
    """Private point-in-time observation retained only in private evidence."""

    observed_at_monotonic: float = Field(ge=0.0, allow_inf_nan=False)
    robot_x: float = Field(allow_inf_nan=False)
    robot_y: float = Field(allow_inf_nan=False)
    target_bounds: PlanarBounds
    distance_metres: float = Field(ge=0.0, allow_inf_nan=False)
    maximum_distance_metres: float = Field(gt=0.0, allow_inf_nan=False)
    passed: bool


def load_periodic_goal(case: EvalCase, case_root: Path) -> SemanticObjectProximityGoal:
    """Load and digest-check a private goal without escaping its case package."""

    reference = case.validator
    if not isinstance(reference, PeriodicGoalValidatorRef):
        raise TypeError("case does not use a periodic-goal validator")
    root = case_root.expanduser().resolve()
    path = (root / reference.private_path).resolve()
    if not path.is_relative_to(root):
        raise ValueError("validator private_path escapes the case package")
    try:
        payload = path.read_bytes()
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"private goal not found: {reference.private_path}") from exc
    digest = hashlib.sha256(payload).hexdigest()
    if digest != reference.private_sha256:
        raise ValueError("private goal digest does not match case reference")
    return SemanticObjectProximityGoal.model_validate_json(payload)


def observe_semantic_proximity(
    *,
    goal: SemanticObjectProximityGoal,
    bounds: PlanarBounds,
    robot_x: float,
    robot_y: float,
    observed_at_monotonic: float,
) -> SemanticGoalObservation:
    distance = bounds.distance_to(robot_x, robot_y)
    return SemanticGoalObservation(
        observed_at_monotonic=observed_at_monotonic,
        robot_x=robot_x,
        robot_y=robot_y,
        target_bounds=bounds,
        distance_metres=distance,
        maximum_distance_metres=goal.maximum_distance_metres,
        passed=distance <= goal.maximum_distance_metres,
    )
