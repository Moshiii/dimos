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

import numpy as np
import pytest

import dimos.robot.model_validation as validation


class _FakeWorld:
    added: list[str] = []

    def __init__(self, enable_viz: bool) -> None:
        assert enable_viz is False

    def add_robot(self, config) -> None:
        self.added.append(config.name)

    def finalize(self) -> None:
        return


class _FakeSolver:
    def __init__(self, nq: int) -> None:
        self.nq = nq

    def forward_kinematics(self, positions: np.ndarray) -> object:
        assert len(positions) == self.nq
        return type("_Pose", (), {"translation": np.zeros(3)})()


@pytest.mark.self_hosted
def test_xarm_blueprint_validation_selects_both_dof_variants(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _FakeWorld.added = []
    fk_cases: list[str] = []

    def fake_from_model_path(path: Path, ee_joint_id: int) -> _FakeSolver:
        case = next(case for case in validation._FK_CASES if case.path == path)
        assert ee_joint_id == case.ee_joint_id
        fk_cases.append(case.name)
        return _FakeSolver(case.dof)

    monkeypatch.setattr(validation, "DrakeWorld", _FakeWorld)
    monkeypatch.setattr(validation.PinocchioIK, "from_model_path", fake_from_model_path)
    monkeypatch.setattr(
        validation,
        "_validate_g1_assets",
        lambda: pytest.fail("G1 must not be validated for the xArm blueprint"),
    )

    result = validation.validate_robot_models("xarm")

    assert _FakeWorld.added == ["xarm6", "xarm7"]
    assert fk_cases == ["xarm6", "xarm7"]
    assert result == (
        "Validated 4 robot model behaviors: xarm6:planning, xarm7:planning, xarm6:fk, xarm7:fk"
    )


@pytest.mark.self_hosted
def test_g1_blueprint_validation_skips_arm_models(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    g1_calls: list[None] = []
    monkeypatch.setattr(
        validation,
        "DrakeWorld",
        lambda **_kwargs: pytest.fail("Arm planning must not run for G1"),
    )
    monkeypatch.setattr(
        validation,
        "_validate_g1_assets",
        lambda: g1_calls.append(None),
    )

    result = validation.validate_robot_models("g1")

    assert g1_calls == [None]
    assert result == "Validated 1 robot model behaviors: g1:urdf+meshes"
