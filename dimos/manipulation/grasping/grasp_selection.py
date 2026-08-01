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

"""Shared contracts for score-ordered, motion-feasible grasp selection."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.spec.utils import Spec


class GraspCandidateRejection(str, Enum):
    """Reason a model-ranked grasp was rejected before physical execution."""

    INVALID = "invalid"
    PRE_GRASP_IK_INFEASIBLE = "pre_grasp_ik_infeasible"
    GRASP_IK_INFEASIBLE = "grasp_ik_infeasible"
    RETREAT_IK_INFEASIBLE = "retreat_ik_infeasible"
    PRE_GRASP_PLANNING_INFEASIBLE = "pre_grasp_planning_infeasible"
    GRASP_PLANNING_INFEASIBLE = "grasp_planning_infeasible"
    RETREAT_PLANNING_INFEASIBLE = "retreat_planning_infeasible"


class GraspEvaluationState(str, Enum):
    """Progress state emitted while evaluating score-ranked proposals."""

    CURRENT = "current"
    REJECTED = "rejected"
    SELECTED = "selected"


@dataclass(frozen=True)
class FeasibleGrasp:
    """First model-ranked grasp with connected IK and motion plans."""

    candidate: GraspCandidate
    rank: int
    pre_grasp_pose: Pose
    retreat_pose: Pose


@dataclass(frozen=True)
class GraspSelectionResult:
    """Motion-free result of evaluating ordered grasp candidates."""

    selected: FeasibleGrasp | None
    rejections: dict[str, int]
    evaluated_count: int


class GraspSelectionSpec(Spec, Protocol):
    """RPC contract for shared manipulation-backed grasp feasibility filtering."""

    def select_first_feasible_grasp(
        self,
        candidates: GraspCandidateArray,
        robot_name: str,
        pre_grasp_offset: float,
        retreat_offset: float | None = None,
        approach_vector: tuple[float, float, float] = (0.0, 0.0, -1.0),
        sequence_start: JointState | None = None,
    ) -> GraspSelectionResult: ...
