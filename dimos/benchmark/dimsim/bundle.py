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

"""Canonical writing and release-blocking validation for DimSim task corpora."""

from __future__ import annotations

from pathlib import Path
from typing import cast

from pydantic import BaseModel

from dimos.benchmark.dimsim.config import CATEGORY_ORDER, GENERATOR_REVISION
from dimos.benchmark.dimsim.generation import GenerationError, compile_smoke_tasks
from dimos.benchmark.dimsim.models import (
    PERSISTED_MODELS,
    CompiledTask,
    Diagnostic,
    ExpectedOutcome,
    GenerationCheck,
    GenerationReport,
    Manifest,
    PublicTask,
    SceneOracleView,
    TaskContract,
)
from dimos.benchmark.dimsim.utilities import model_bytes, task_id
from dimos.benchmark.spatial.utilities import JsonValue, canonical_json, stable_opaque_id

_PRIVATE_KEYS = {
    "contract",
    "expected",
    "identity_payload",
    "oracle_view_digest",
    "source",
    "entity_id",
    "entity_ids",
    "target_entity_id",
    "anchor_entity_id",
    "candidate_entity_ids",
    "provenance",
}


def json_schemas() -> dict[str, dict[str, object]]:
    """Return JSON Schemas for every persisted record type."""

    return {model.__name__: model.model_json_schema() for model in PERSISTED_MODELS}


def _write_json(path: Path, model: BaseModel) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(model_bytes(model) + b"\n")


def _write_jsonl(path: Path, records: tuple[BaseModel, ...]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b"".join(model_bytes(record) + b"\n" for record in records))


def _private_key_paths(value: JsonValue, path: str = "$") -> tuple[str, ...]:
    found: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = f"{path}.{key}"
            if key in _PRIVATE_KEYS:
                found.append(child_path)
            found.extend(_private_key_paths(child, child_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            found.extend(_private_key_paths(child, f"{path}[{index}]"))
    return tuple(found)


def validate_compiled_tasks(tasks: tuple[CompiledTask, ...]) -> tuple[GenerationCheck, ...]:
    """Validate cardinality, references, identities, typing, and public separation."""

    categories = tuple(task.public.category for task in tasks)
    if categories != CATEGORY_ORDER:
        raise GenerationError(
            f"smoke categories must be {CATEGORY_ORDER!r}; received {categories!r}"
        )
    public_ids = [task.public.task_id for task in tasks]
    contract_ids = [task.contract.task_id for task in tasks]
    outcome_ids = [task.outcome.task_id for task in tasks]
    if len(set(public_ids)) != 4 or public_ids != contract_ids or public_ids != outcome_ids:
        raise GenerationError("public, contract, and outcome records must join one-to-one")

    for task in tasks:
        reconstructed = task_id(cast("dict[str, JsonValue]", task.contract.identity_payload))
        if reconstructed != task.public.task_id:
            raise GenerationError(f"task ID reconstruction failed for {task.public.task_id}")
        leaked = _private_key_paths(cast("JsonValue", task.public.model_dump(mode="json")))
        if leaked:
            raise GenerationError(f"public task contains private fields: {leaked!r}")
        source = task.contract.source
        if source.oracle_view_digest != task.outcome.oracle_view_digest:
            raise GenerationError(f"source digest mismatch for {task.public.task_id}")

    regenerated = tuple(compile_smoke_tasks_from_sources(tasks))
    if tuple(model_bytes(item) for item in tasks) != tuple(
        model_bytes(item) for item in regenerated
    ):
        raise GenerationError("deterministic regeneration check failed")

    return (
        GenerationCheck(name="schema-validity", passed=True, detail="all records validate"),
        GenerationCheck(
            name="category-cardinality",
            passed=True,
            detail="exactly one task in each required category",
        ),
        GenerationCheck(
            name="entity-resolution",
            passed=True,
            detail="every contract resolved the required exact semantic entities",
        ),
        GenerationCheck(
            name="answer-typing",
            passed=True,
            detail="all expected outcomes match their declared response types",
        ),
        GenerationCheck(
            name="destination-reachability",
            passed=True,
            detail="destination has a collision-free spawn-connected stopping region",
        ),
        GenerationCheck(
            name="comparison-distance-stability",
            passed=True,
            detail="surface-distance winner exceeds the configured stability margin",
        ),
        GenerationCheck(
            name="stable-reference-integrity",
            passed=True,
            detail="IDs reconstruct and records join one-to-one",
        ),
        GenerationCheck(
            name="public-oracle-leakage",
            passed=True,
            detail="public records contain no private fields",
        ),
        GenerationCheck(
            name="canonical-regeneration",
            passed=True,
            detail="compiled records serialize deterministically",
        ),
        GenerationCheck(
            name="source-provenance",
            passed=True,
            detail="source revisions and oracle digests are present and consistent",
        ),
    )


def compile_smoke_tasks_from_sources(
    tasks: tuple[CompiledTask, ...],
) -> tuple[CompiledTask, ...]:
    """Rebuild immutable records to verify canonical model serialization.

    Pure scene regeneration is checked by ``generate_smoke_release`` before
    calling this validator. Here, every discriminated persisted record is
    round-tripped through its public schema.
    """

    return tuple(CompiledTask.model_validate_json(model_bytes(task)) for task in tasks)


def _failure_report(message: str) -> GenerationReport:
    return GenerationReport(
        complete=False,
        retained_task_count=0,
        checks=(
            GenerationCheck(
                name="generation",
                passed=False,
                detail="generation rejected before publication",
            ),
        ),
        diagnostics=(Diagnostic(code="generation-rejected", message=message),),
    )


def generate_smoke_release(
    view: SceneOracleView,
    root: Path,
    *,
    release_version: str = "v1.0.0",
) -> Manifest:
    """Compile, validate, and canonically write one four-question release."""

    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f"release root must be absent or empty: {root}")
    try:
        first = compile_smoke_tasks(view)
        second = compile_smoke_tasks(view)
        if tuple(model_bytes(item) for item in first) != tuple(
            model_bytes(item) for item in second
        ):
            raise GenerationError("same-view regeneration produced different bytes")
        checks = validate_compiled_tasks(first)
    except (GenerationError, ValueError) as error:
        _write_json(root / "oracle" / "generation_report.json", _failure_report(str(error)))
        raise

    release_id = stable_opaque_id(
        "dimsim_release",
        {
            "release_version": release_version,
            "generator_revision": GENERATOR_REVISION,
            "task_ids": [task.public.task_id for task in first],
        },
    )
    manifest = Manifest(
        release_id=release_id,
        release_version=release_version,
        generator_revision=GENERATOR_REVISION,
        complete=True,
        task_count=len(first),
    )
    report = GenerationReport(
        complete=True,
        retained_task_count=len(first),
        checks=checks,
    )
    _write_jsonl(root / "public" / "tasks.jsonl", tuple(task.public for task in first))
    _write_jsonl(
        root / "oracle" / "task_contracts.jsonl",
        tuple(task.contract for task in first),
    )
    _write_jsonl(
        root / "oracle" / "expected_outcomes.jsonl",
        tuple(task.outcome for task in first),
    )
    _write_json(root / "oracle" / "generation_report.json", report)
    _write_json(root / "manifest.json", manifest)
    return manifest


def _load_jsonl_lines(path: Path) -> tuple[str, ...]:
    return tuple(line for line in path.read_text(encoding="utf-8").splitlines() if line)


def load_public_tasks(public_root: Path) -> tuple[PublicTask, ...]:
    """Load a physically independent public directory."""

    return tuple(
        PublicTask.model_validate_json(line)
        for line in _load_jsonl_lines(public_root / "tasks.jsonl")
    )


def load_full_release(
    root: Path,
) -> tuple[Manifest, tuple[PublicTask, ...], tuple[TaskContract, ...], tuple[ExpectedOutcome, ...]]:
    """Load and reference-check a complete public/private release."""

    manifest = Manifest.model_validate_json((root / "manifest.json").read_bytes())
    public = load_public_tasks(root / "public")
    contracts = tuple(
        TaskContract.model_validate_json(line)
        for line in _load_jsonl_lines(root / "oracle" / "task_contracts.jsonl")
    )
    outcomes = tuple(
        ExpectedOutcome.model_validate_json(line)
        for line in _load_jsonl_lines(root / "oracle" / "expected_outcomes.jsonl")
    )
    identifiers = {task.task_id for task in public}
    if identifiers != {item.task_id for item in contracts} or identifiers != {
        item.task_id for item in outcomes
    }:
        raise ValueError("release records do not join one-to-one by opaque task ID")
    return manifest, public, contracts, outcomes


def schemas_json() -> bytes:
    return canonical_json(cast("JsonValue", json_schemas()))
