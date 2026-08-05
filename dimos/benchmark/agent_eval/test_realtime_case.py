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

import hashlib
import json
from pathlib import Path

from pydantic import ValidationError
import pytest

from dimos.benchmark.agent_eval.case import (
    EmbodiedInstructionTask,
    EvalCase,
    LiveCodePolicyInteraction,
    PeriodicGoalValidatorRef,
    SimulatorSceneSource,
    SourcePreparationRef,
)
from dimos.benchmark.agent_eval.periodic_goal import load_periodic_goal


def _source(**updates: object) -> SimulatorSceneSource:
    values = {
        "scene": "dimsim-apartment",
        "simulation_provider": "pimsim",
        "robot": "unitree-go2",
        "dimos_blueprint": "unitree-go2",
    }
    values.update(updates)
    return SimulatorSceneSource.model_validate(values)


def _case(digest: str = "a" * 64) -> EvalCase:
    return EvalCase.compile(
        case_id="go-to-bed",
        source=_source(),
        task=EmbodiedInstructionTask(prompt="Go to the bed"),
        interaction=LiveCodePolicyInteraction(driver_revision="v1", timeout_seconds=10.0),
        validator=PeriodicGoalValidatorRef(
            revision="v1",
            private_path="private/goal.json",
            private_sha256=digest,
        ),
    )


def test_checked_in_case_and_private_goal_are_valid_and_private() -> None:
    root = Path(__file__).parents[1] / "realtime_sim" / "cases" / "go2-apartment-go-to-bed"
    case = EvalCase.model_validate_json((root / "case.json").read_bytes())
    goal = load_periodic_goal(case, root)

    assert case.source.kind == "simulator_scene"
    assert case.source.dimos_blueprint == "unitree-go2"
    assert case.task.prompt == "Go to the bed"
    assert goal.semantic_query == "queen size bed"
    assert goal.maximum_distance_metres == 2.0
    public = case.public_projection().model_dump_json()
    assert "queen size bed" not in public
    assert "2.0" not in public
    assert "private/goal.json" not in public


def test_simulator_source_and_preparation_are_fingerprint_bound() -> None:
    first = _case()
    second = EvalCase.compile(
        case_id=first.case_id,
        source=_source(scene="another-scene"),
        task=first.task,
        interaction=first.interaction,
        validator=first.validator,
    )
    assert first.fingerprint != second.fingerprint

    preparation = SourcePreparationRef(
        revision="v1",
        exploration_route=((1.0, 2.0),),
        final_start_pose=(3.0, 0.0, 0.52),
        step_timeout_seconds=1.0,
        odometry_timeout_seconds=1.0,
        start_tolerance_metres=0.25,
    )
    prepared = EvalCase.compile(
        case_id=first.case_id,
        source=_source(preparation=preparation),
        task=first.task,
        interaction=first.interaction,
        validator=first.validator,
    )
    assert first.fingerprint != prepared.fingerprint


@pytest.mark.parametrize("timeout", [0.0, -1.0, float("inf"), float("nan")])
def test_live_interaction_rejects_invalid_deadlines(timeout: float) -> None:
    with pytest.raises(ValidationError):
        LiveCodePolicyInteraction(driver_revision="v1", timeout_seconds=timeout)


@pytest.mark.parametrize("field", ["scene", "simulation_provider", "robot", "dimos_blueprint"])
def test_simulator_source_rejects_missing_required_field(field: str) -> None:
    values = _source().model_dump()
    del values[field]
    with pytest.raises(ValidationError):
        SimulatorSceneSource.model_validate(values)


@pytest.mark.parametrize("path", ["/tmp/goal.json", "../goal.json", "private/../goal.json"])
def test_periodic_goal_rejects_unsafe_private_path(path: str) -> None:
    with pytest.raises(ValidationError, match="safe relative path"):
        PeriodicGoalValidatorRef(revision="v1", private_path=path, private_sha256="a" * 64)


def test_private_goal_rejects_malformed_or_changed_content(tmp_path: Path) -> None:
    private = tmp_path / "private"
    private.mkdir()
    goal = private / "goal.json"
    goal.write_text("{}")
    digest = hashlib.sha256(goal.read_bytes()).hexdigest()
    with pytest.raises(ValidationError):
        load_periodic_goal(_case(digest), tmp_path)

    goal.write_text(json.dumps({"changed": True}))
    with pytest.raises(ValueError, match="digest"):
        load_periodic_goal(_case(digest), tmp_path)
