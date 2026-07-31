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

import json
from pathlib import Path

import pytest

from dimos.benchmark.dimsim.bundle import (
    generate_smoke_release,
    load_full_release,
    load_public_tasks,
)
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture
from dimos.benchmark.dimsim.generation import GenerationError


def test_release_writes_canonical_public_and_private_layout(tmp_path: Path) -> None:
    root = tmp_path / "release"

    manifest = generate_smoke_release(apartment_oracle_fixture(), root)

    assert manifest.complete is True
    assert manifest.task_count == 4
    assert {path.relative_to(root).as_posix() for path in root.rglob("*") if path.is_file()} == {
        "manifest.json",
        "public/tasks.jsonl",
        "oracle/task_contracts.jsonl",
        "oracle/expected_outcomes.jsonl",
        "oracle/generation_report.json",
    }
    report = json.loads((root / "oracle" / "generation_report.json").read_text())
    assert report["complete"] is True
    assert all(check["passed"] for check in report["checks"])
    assert {check["name"] for check in report["checks"]} == {
        "schema-validity",
        "category-cardinality",
        "entity-resolution",
        "answer-typing",
        "destination-reachability",
        "comparison-distance-stability",
        "stable-reference-integrity",
        "public-oracle-leakage",
        "canonical-regeneration",
        "source-provenance",
    }


def test_same_fixture_regenerates_byte_equivalent_release(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"

    generate_smoke_release(apartment_oracle_fixture(), first)
    generate_smoke_release(apartment_oracle_fixture(), second)

    assert {
        path.relative_to(first): path.read_bytes() for path in first.rglob("*") if path.is_file()
    } == {
        path.relative_to(second): path.read_bytes() for path in second.rglob("*") if path.is_file()
    }


def test_public_root_loads_without_oracle_and_contains_no_private_fields(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)

    tasks = load_public_tasks(root / "public")
    serialized = (root / "public" / "tasks.jsonl").read_text()

    assert len(tasks) == 4
    assert "oracle_view_digest" not in serialized
    assert "entity_id" not in serialized
    assert '"expected"' not in serialized


def test_full_release_joins_only_by_opaque_task_id(tmp_path: Path) -> None:
    root = tmp_path / "release"
    generate_smoke_release(apartment_oracle_fixture(), root)

    manifest, public, contracts, outcomes = load_full_release(root)

    assert manifest.task_count == 4
    assert {item.task_id for item in public} == {item.task_id for item in contracts}
    assert {item.task_id for item in public} == {item.task_id for item in outcomes}


def test_failed_generation_writes_only_private_diagnostics(tmp_path: Path) -> None:
    root = tmp_path / "failed"
    view = apartment_oracle_fixture()
    changed = view.model_copy(
        update={
            "entities": tuple(
                entity for entity in view.entities if entity.semantic_class != "television"
            )
        }
    )

    with pytest.raises(GenerationError, match="television"):
        generate_smoke_release(changed, root)

    assert not (root / "manifest.json").exists()
    assert not (root / "public").exists()
    report = json.loads((root / "oracle" / "generation_report.json").read_text())
    assert report["complete"] is False
