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

import pytest

from dimos.hardware.manipulators.galaxea_a1z.config import (
    A1ZConfig,
    A1ZGripperConfig,
    A1ZTeachingConfig,
)
from dimos.utils.data import LfsPath


@pytest.mark.parametrize("gravity_comp_factor", [-0.1, 1.1])
def test_gravity_compensation_factor_must_be_normalized(
    gravity_comp_factor: float,
) -> None:
    with pytest.raises(ValueError):
        A1ZConfig(gravity_comp_factor=gravity_comp_factor)


def test_gripper_free_drive_requires_gripper() -> None:
    with pytest.raises(ValueError, match="requires a configured gripper"):
        A1ZConfig(teaching=A1ZTeachingConfig(gripper_free_drive=True))


def test_teaching_with_gripper_free_drive_is_valid() -> None:
    config = A1ZConfig(
        gripper=A1ZGripperConfig(),
        teaching=A1ZTeachingConfig(gripper_free_drive=True),
    )

    assert config.teaching is not None
    assert config.teaching.gripper_free_drive


def test_lazy_urdf_path_is_validated_without_resolving() -> None:
    lazy_path = LfsPath("a1z_description/A1Z_G1Z.urdf")

    config = A1ZConfig(urdf_path=lazy_path)

    assert config.urdf_path is lazy_path
    assert object.__getattribute__(lazy_path, "_lfs_resolved_cache") is None


def test_urdf_path_rejects_other_types() -> None:
    with pytest.raises(TypeError, match="urdf_path.*str, Path, or None"):
        A1ZConfig(urdf_path=42)  # type: ignore[arg-type]
