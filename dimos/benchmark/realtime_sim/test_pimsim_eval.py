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

from pathlib import Path

import pytest

from dimos.benchmark.agent_eval.single_case import EvalRunConfig, execute_single_case


@pytest.mark.self_hosted_large
def test_pimsim_go2_bed_evaluation_owns_full_lifecycle(tmp_path: Path) -> None:
    """Credentialed acceptance: startup, preparation, Pi, polling, and cleanup."""

    case = Path(__file__).parent / "cases" / "go2-apartment-go-to-bed" / "case.json"
    result = execute_single_case(case, config=EvalRunConfig(), output_root=tmp_path)

    assert result.attempt_status == "completed"
    assert result.task_result in {"passed", "failed"}
    assert result.reason in {"goal_reached", "episode_timeout"}
    required = {
        "runtime-startup.v1.json",
        "source-preparation.v1.json",
        "agent-outcome.v1.json",
        "goal-observations.private.v1.json",
        "score.private.v1.json",
        "attempt-manifest.v1.json",
        "outcome.v1.json",
    }
    assert required <= {path.name for path in result.artifact_path.iterdir()}
