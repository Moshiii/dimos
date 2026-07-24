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

"""Runtime validation blueprint for externally sourced robot models."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.world.drake_world import DrakeWorld
from dimos.robot.manipulators.a1z.config import (
    A1Z_FK_MODEL,
    make_a1z_model_config,
)
from dimos.robot.manipulators.a750.config import (
    A750_FK_MODEL,
    make_a750_model_config,
)
from dimos.robot.manipulators.piper.config import (
    PIPER_FK_MODEL,
    make_piper_model_config,
)
from dimos.robot.manipulators.xarm.config import (
    XARM6_FK_MODEL,
    XARM7_FK_MODEL,
    make_xarm6_model_config,
    make_xarm7_model_config,
)
from dimos.robot.unitree.g1.g1_rerun import (
    G1_MESH_DIR,
    G1_RERUN_URDF,
    g1_urdf_static_robot,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class _FkCase:
    name: str
    path: Path
    dof: int
    ee_joint_id: int


_PLANNING_CASES: tuple[tuple[str, RobotModelConfig], ...] = (
    ("piper", make_piper_model_config(name="piper")),
    ("xarm6", make_xarm6_model_config(name="xarm6", add_gripper=False)),
    ("xarm7", make_xarm7_model_config(name="xarm7", add_gripper=False)),
    ("a750", make_a750_model_config(name="a750")),
    ("a1z", make_a1z_model_config(name="a1z")),
)
_FK_CASES = (
    _FkCase("piper", PIPER_FK_MODEL, 6, 6),
    _FkCase("xarm6", XARM6_FK_MODEL, 6, 6),
    _FkCase("xarm7", XARM7_FK_MODEL, 7, 7),
    _FkCase("a750", A750_FK_MODEL, 6, 6),
    _FkCase("a1z", A1Z_FK_MODEL, 6, 6),
)


class RobotModelValidation(Module):
    """Resolve and exercise robot models migrated away from Git LFS."""

    @rpc
    def start(self) -> None:
        super().start()
        logger.info(self.validate())

    @rpc
    def validate(self) -> str:
        """Resolve every source and validate planning, FK, and G1 visualization assets."""
        return validate_robot_models()


def validate_robot_models() -> str:
    """Run the model checks used by the validation blueprint."""
    validated: list[str] = []
    for name, config in _PLANNING_CASES:
        world = DrakeWorld(enable_viz=False)
        world.add_robot(config)
        world.finalize()
        validated.append(f"{name}:planning")

    for case in _FK_CASES:
        solver = PinocchioIK.from_model_path(case.path, ee_joint_id=case.ee_joint_id)
        if solver.nq != case.dof:
            raise ValueError(f"{case.name} FK model has {solver.nq} DOF; expected {case.dof}")
        pose = solver.forward_kinematics(np.zeros(case.dof))
        if not np.isfinite(pose.translation).all():
            raise ValueError(f"{case.name} FK produced a non-finite pose")
        validated.append(f"{case.name}:fk")

    _validate_g1_assets()
    validated.append("g1:urdf+meshes")
    return f"Validated {len(validated)} robot model behaviors: {', '.join(validated)}"


def _validate_g1_assets() -> None:
    urdf_path = Path(str(G1_RERUN_URDF))
    mesh_root = Path(str(G1_MESH_DIR))
    root = ET.parse(urdf_path).getroot()
    mesh_paths = [
        filename
        for mesh in root.findall(".//mesh")
        if (filename := mesh.get("filename")) is not None
    ]
    if not mesh_paths:
        raise ValueError(f"G1 URDF contains no visual meshes: {urdf_path}")
    missing = [
        filename
        for filename in mesh_paths
        if not (urdf_path.parent / filename).exists()
        and not (mesh_root / Path(filename).name).exists()
    ]
    if missing:
        raise FileNotFoundError(f"G1 URDF references missing meshes: {missing[:5]}")
    g1_urdf_static_robot()._load_robot()


robot_model_validation = RobotModelValidation.blueprint()
