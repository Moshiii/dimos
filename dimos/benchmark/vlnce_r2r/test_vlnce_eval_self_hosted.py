# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Credentialed acceptance of the exact one-command VLN-CE user contract."""

import json
from pathlib import Path
import subprocess

import pytest

from dimos.constants import DIMOS_PROJECT_ROOT


@pytest.mark.self_hosted_large
def test_exact_cli_owns_full_external_benchmark_lifecycle(tmp_path: Path) -> None:
    case = DIMOS_PROJECT_ROOT / "dimos/benchmark/vlnce_r2r/cases/mp3d-example-episode-515/case.json"
    output = tmp_path / "vlnce-eval"
    command = [
        "uv",
        "run",
        "dimos",
        "eval",
        "run",
        str(case),
        f"--output={output}",
        "--quiet",
    ]

    completed = subprocess.run(
        command,
        cwd=DIMOS_PROJECT_ROOT,
        check=False,
        capture_output=True,
        text=True,
        timeout=600,
    )

    assert completed.returncode in {0, 1}, completed.stderr
    attempts = list(output.glob("attempt_*"))
    assert len(attempts) == 1
    outcome = json.loads((attempts[0] / "outcome.v1.json").read_text())
    assert outcome["attempt_status"] == "completed"
    assert outcome["task_result"] in {"passed", "failed"}
    assert (attempts[0] / "terminal-private/vlnce-result.v1.json").is_file()
    assert (attempts[0] / "runtime-public-terminal.v1.json").is_file()
    cleanup = json.loads((attempts[0] / "cleanup.v1.json").read_text())
    assert cleanup["diagnostics"] == []
