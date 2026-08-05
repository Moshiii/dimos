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

import numpy as np
import open3d as o3d
import pytest

from dimos.benchmark.agent_eval.case import (
    EvalCase,
    ExactIntegerValidatorRef,
    FrozenCodePolicyInteraction,
    FrozenRecordingSource,
    IntegerQuestionTask,
)
from dimos.benchmark.agent_eval.pi import PiTurn
from dimos.benchmark.agent_eval.pi_adapter import PythonExecBroker
from dimos.benchmark.short_horizon_qa.eval import (
    parse_integer_prediction,
    run_frozen_case,
)
from dimos.benchmark.short_horizon_qa.models import MapperSettings
from dimos.benchmark.short_horizon_qa.prepare import prepare_bundle
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _cloud(x: float, ts: float) -> PointCloud2:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.asarray([[x, 0.0, 0.5]]))
    return PointCloud2(cloud, frame_id="world", ts=ts)


class ScriptedPiSession:
    def __init__(self, broker: PythonExecBroker, final_text: str) -> None:
        self.session_id = "pi_session_scripted"
        self.broker = broker
        self.final_text = final_text

    def prompt(self, prompt: str, timeout_s: float) -> PiTurn:
        del prompt, timeout_s
        self.broker.request(
            "python_exec",
            {
                "code": "(memory.streams.lidar.count(), "
                "memory.streams.global_map.last().tags['frame_count'], "
                "'app' in globals())"
            },
        )
        return PiTurn(final_text=self.final_text, policy_call_count=1)

    def abort(self, timeout_s: float) -> None:
        del timeout_s

    def dispose(self) -> None:
        return None

    def artifact_references(self):
        return ()


class ScriptedPiFactory:
    def __init__(self, final_text: str) -> None:
        self.final_text = final_text

    def create(
        self,
        *,
        attempt_path: Path,
        public_prompt: str,
        code_policy_session_id: str,
        call_log,
        mcp,
    ) -> ScriptedPiSession:
        del public_prompt
        return ScriptedPiSession(
            PythonExecBroker(
                attempt_id=attempt_path.name,
                pi_session_id="pi_session_scripted",
                code_policy_session_id=code_policy_session_id,
                mcp=mcp,
                call_log=call_log,
            ),
            self.final_text,
        )


@pytest.mark.parametrize(
    ("text", "status", "answer"),
    [
        ("I counted them.\nANSWER: 4", "parsed", 4),
        ("ANSWER: -2", "parsed", -2),
        ("The answer is 4", "invalid", None),
        ("ANSWER: 3\nthen maybe\nANSWER: 4", "invalid", None),
        ("ANSWER: 4\nextra", "invalid", None),
    ],
)
def test_marked_integer_parser(text: str, status: str, answer: int | None) -> None:
    prediction = parse_integer_prediction(
        case_id="case",
        attempt_id="attempt",
        agent_session_id="pi",
        interaction_session_id="policy",
        final_text=text,
    )
    assert prediction.status == status
    assert prediction.integer_answer == answer


@pytest.mark.parametrize(
    ("final_text", "task_result"),
    [("Used memory.\nANSWER: 2", "passed"), ("ANSWER: 3", "failed"), ("2", "failed")],
)
def test_real_standalone_frozen_attempt_scores_scripted_pi(
    tmp_path: Path, final_text: str, task_result: str
) -> None:
    recording = tmp_path / "recording.db"
    with SqliteStore(path=str(recording)) as store:
        lidar = store.stream("lidar", PointCloud2)
        for index in range(5):
            ts = 100.0 + index
            lidar.append(_cloud(float(index), ts), ts=ts)
    bundle = tmp_path / "bundle"
    prepare_bundle(
        recording,
        [],
        bundle,
        progress=[1.0],
        mapper=MapperSettings(device="CPU:0"),
    )
    private_root = tmp_path / "validators"
    oracle_path = private_root / "private" / "oracle.json"
    oracle_path.parent.mkdir(parents=True)
    oracle = {
        "schema_version": "1.0",
        "expected_count": 2,
        "counting_policy": "Test rooms.",
        "rooms": [
            {"schema_version": "1.0", "label": "one", "evidence": ["test"]},
            {"schema_version": "1.0", "label": "two", "evidence": ["test"]},
        ],
        "reviewed_by": ["test-reviewer"],
    }
    oracle_path.write_text(json.dumps(oracle))
    digest = hashlib.sha256(oracle_path.read_bytes()).hexdigest()
    case = EvalCase.compile(
        case_id="recording-room-count",
        source=FrozenRecordingSource(recording="recording", progress=1.0),
        task=IntegerQuestionTask(prompt="How many rooms in total?"),
        interaction=FrozenCodePolicyInteraction(driver_revision="v1"),
        validator=ExactIntegerValidatorRef(
            revision="exact-v1",
            private_path="private/oracle.json",
            private_sha256=digest,
        ),
    )

    result = run_frozen_case(
        case=case,
        bundle=bundle,
        private_root=private_root,
        output_root=tmp_path / "attempts",
        pi_factory=ScriptedPiFactory(final_text),
    )

    assert result.outcome.attempt_status == "completed"
    assert result.outcome.task_result == task_result
    assert (result.attempt_path / "prediction.v1.json").is_file()
    assert (result.attempt_path / "score.private.v1.json").is_file()
    assert (result.attempt_path / "attempt-manifest.v1.json").is_file()
    calls = (result.attempt_path / "code-policy-calls.jsonl").read_text().splitlines()
    assert len(calls) == 1
    records = json.loads((result.attempt_path / "code-policy-records.v1.json").read_text())
    assert "False" in records[0]["output"]
