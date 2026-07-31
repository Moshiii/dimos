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
import subprocess

import pytest
from pytest_mock import MockerFixture

from dimos.simulation.dimsim.dimsim_process import (
    _launch_hidden_browser,
    _validate_repo,
)
from dimos.simulation.dimsim.revision import DIMSIM_REPO_COMMIT


def test_validate_repo_accepts_clean_pinned_checkout(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    run = mocker.patch(
        "dimos.simulation.dimsim.dimsim_process.subprocess.run",
        side_effect=(
            subprocess.CompletedProcess([], 0, stdout=f"{DIMSIM_REPO_COMMIT}\n"),
            subprocess.CompletedProcess([], 0, stdout=""),
        ),
    )

    _validate_repo(tmp_path)

    assert run.call_count == 2


def test_validate_repo_rejects_unpinned_checkout(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    mocker.patch(
        "dimos.simulation.dimsim.dimsim_process.subprocess.run",
        return_value=subprocess.CompletedProcess([], 0, stdout=f"{'0' * 40}\n"),
    )

    with pytest.raises(RuntimeError, match="expected pinned revision"):
        _validate_repo(tmp_path)


def test_validate_repo_rejects_tracked_modifications(
    mocker: MockerFixture,
    tmp_path: Path,
) -> None:
    mocker.patch(
        "dimos.simulation.dimsim.dimsim_process.subprocess.run",
        side_effect=(
            subprocess.CompletedProcess([], 0, stdout=f"{DIMSIM_REPO_COMMIT}\n"),
            subprocess.CompletedProcess([], 0, stdout=" M src/engine.js\n"),
        ),
    )

    with pytest.raises(RuntimeError, match="modified tracked files"):
        _validate_repo(tmp_path)


@pytest.mark.parametrize("value", ["0", "false", "FALSE", "no", " No "])
def test_explicit_false_disables_hidden_dimsim_browser(value: str) -> None:
    assert not _launch_hidden_browser(value)


@pytest.mark.parametrize("value", ["", "1", "true", "yes", "unexpected"])
def test_hidden_dimsim_browser_remains_default(value: str) -> None:
    assert _launch_hidden_browser(value)
