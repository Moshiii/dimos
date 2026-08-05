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

"""Self-hosted mechanics gate over the real Hong Kong office recording.

The zero-valued oracle in this test is deliberately synthetic and must never be
used as the north-star room-count oracle. It validates plumbing only.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

from dimos.benchmark.agent_eval.case import (
    EvalCase,
    ExactIntegerValidatorRef,
    FrozenCodePolicyInteraction,
    FrozenRecordingSource,
    IntegerQuestionTask,
)
from dimos.benchmark.short_horizon_qa.eval import run_frozen_case
from dimos.benchmark.short_horizon_qa.models import MapperSettings
from dimos.benchmark.short_horizon_qa.prepare import prepare_bundle
from dimos.benchmark.short_horizon_qa.test_eval import ScriptedPiFactory
from dimos.utils.data import get_data


@pytest.mark.self_hosted
def test_real_hongkong_recording_standalone_scripted_mechanics(tmp_path: Path) -> None:
    recording = get_data("go2_hongkong_office.db")
    bundle = tmp_path / "bundle"
    manifest = prepare_bundle(
        recording,
        [],
        bundle,
        progress=[1.0],
        mapper=MapperSettings(device="CPU:0"),
    )
    private_root = tmp_path / "validators"
    oracle_path = private_root / "private" / "mechanics-only.json"
    oracle_path.parent.mkdir(parents=True)
    oracle_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "expected_count": 0,
                "counting_policy": "Synthetic mechanics value; not a room oracle.",
                "rooms": [],
                "reviewed_by": ["self-hosted-mechanics-test"],
            }
        )
    )
    case = EvalCase.compile(
        case_id="hongkong-office-mechanics-only",
        source=FrozenRecordingSource(
            recording="go2_hongkong_office",
            progress=1.0,
            bundle_manifest_sha256=hashlib.sha256(
                (bundle / "manifest.v1.json").read_bytes()
            ).hexdigest(),
        ),
        task=IntegerQuestionTask(prompt="How many rooms in total?"),
        interaction=FrozenCodePolicyInteraction(driver_revision="v1"),
        validator=ExactIntegerValidatorRef(
            revision="mechanics-only-v1",
            private_path="private/mechanics-only.json",
            private_sha256=hashlib.sha256(oracle_path.read_bytes()).hexdigest(),
        ),
    )

    result = run_frozen_case(
        case=case,
        bundle=bundle,
        private_root=private_root,
        output_root=tmp_path / "attempts",
        pi_factory=ScriptedPiFactory("ANSWER: 0"),
    )

    assert result.outcome.task_result == "passed"
    assert manifest.cutoffs[0].normalized_progress == 1.0
    assert manifest.cutoffs[0].map_frame_count == 4235
    records = json.loads((result.attempt_path / "code-policy-records.v1.json").read_text())
    assert "False" in records[0]["output"]
