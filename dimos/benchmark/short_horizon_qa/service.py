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

"""Offline MCP service exposing CodePolicy over one frozen memory cutoff."""

from __future__ import annotations

import math
from pathlib import Path

from dimos.agents.code_policy import CodePolicyModule
from dimos.agents.mcp.mcp_server import McpServer
from dimos.benchmark.short_horizon_qa.models import CutoffRecord, FrozenMemoryManifest
from dimos.benchmark.short_horizon_qa.prepare import (
    DERIVED_NAME,
    MANIFEST_NAME,
    file_sha256,
)
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator


def load_bundle(
    bundle: Path,
    cutoff_seconds: float,
    *,
    verify_integrity: bool = True,
) -> tuple[FrozenMemoryManifest, CutoffRecord, Path, Path]:
    """Validate a prepared bundle and resolve one exact configured cutoff."""
    bundle = bundle.resolve()
    manifest_path = bundle / MANIFEST_NAME
    if not manifest_path.is_file():
        raise FileNotFoundError(manifest_path)
    manifest = FrozenMemoryManifest.model_validate_json(manifest_path.read_text(encoding="utf-8"))
    source_path = Path(manifest.source_path)
    derived_path = bundle / DERIVED_NAME
    if not source_path.is_file():
        raise FileNotFoundError(source_path)
    if not derived_path.is_file():
        raise FileNotFoundError(derived_path)

    matches = [
        cutoff
        for cutoff in manifest.cutoffs
        if math.isclose(cutoff.cutoff_seconds, cutoff_seconds, rel_tol=0.0, abs_tol=1e-9)
    ]
    if len(matches) != 1:
        available = ", ".join(str(item.cutoff_seconds) for item in manifest.cutoffs)
        raise ValueError(
            f"Cutoff {cutoff_seconds}s is not in the prepared bundle. Available: {available}"
        )

    if verify_integrity:
        if source_path.stat().st_size != manifest.source_size_bytes:
            raise ValueError(f"Source recording size changed: {source_path}")
        if file_sha256(source_path) != manifest.source_sha256:
            raise ValueError(f"Source recording hash changed: {source_path}")
        if file_sha256(derived_path) != manifest.derived_sha256:
            raise ValueError(f"Derived recording hash changed: {derived_path}")
    return manifest, matches[0], source_path, derived_path


def frozen_qa_blueprint(
    source_path: Path,
    derived_path: Path,
    cutoff: CutoffRecord,
    *,
    mcp_port: int = 9990,
) -> Blueprint:
    """Compose the offline one-tool policy service for a selected cutoff."""
    return autoconnect(
        CodePolicyModule.blueprint(
            recording_path=str(source_path),
            derived_recording_path=str(derived_path),
            memory_cutoff_timestamp=cutoff.cutoff_timestamp,
            connect_app=False,
        ),
        McpServer.blueprint(),
    ).global_config(viewer="none", n_workers=2, mcp_port=mcp_port)


def serve_bundle(bundle: Path, cutoff_seconds: float, *, mcp_port: int = 9990) -> None:
    """Run the frozen QA MCP endpoint until interrupted."""
    _, cutoff, source_path, derived_path = load_bundle(bundle, cutoff_seconds)
    blueprint = frozen_qa_blueprint(
        source_path,
        derived_path,
        cutoff,
        mcp_port=mcp_port,
    )
    coordinator = ModuleCoordinator.build(blueprint)
    coordinator.loop()
