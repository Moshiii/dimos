# Copyright 2026 Dimensional Inc.
"""Atomic, public-only persistence for standalone VQA evaluation runs."""

from __future__ import annotations

from collections.abc import Iterable
from datetime import UTC, datetime
import hashlib
import json
import os
from pathlib import Path
from typing import Any
from uuid import uuid4

from filelock import FileLock

from dimos.benchmark.vqa.evaluation.models import (
    EvaluationCheckpoint,
    EvaluationDatasetIdentity,
    EvaluationProgressEvent,
    EvaluationRunConfig,
    EvaluationRunManifest,
    RunCompletedEvent,
    RunFailedEvent,
    RunStartedEvent,
)


def dataset_identity(dataset: Path) -> EvaluationDatasetIdentity:
    """Hash only public case records and the optional public dataset manifest."""
    root = dataset.expanduser().resolve()
    case_records = sorted(root.glob("frame-*/cases/*/case.json"))
    digest = hashlib.sha256()
    for case_path in case_records:
        digest.update(case_path.relative_to(root).as_posix().encode())
        digest.update(b"\0")
        digest.update(case_path.read_bytes())
        digest.update(b"\0")
    manifest_path = root / "manifest.json"
    manifest_digest = (
        hashlib.sha256(manifest_path.read_bytes()).hexdigest() if manifest_path.is_file() else None
    )
    return EvaluationDatasetIdentity(
        resolved_path=str(root),
        public_digest=digest.hexdigest(),
        case_count=len(case_records),
        public_manifest_digest=manifest_digest,
    )


class EvaluationRunStore:
    """Exclusive owner of a run directory and its atomic public artifacts."""

    def __init__(self, root: Path, lock: FileLock, manifest: EvaluationRunManifest) -> None:
        self.root = root
        self._lock = lock
        self.manifest = manifest

    @classmethod
    def create(cls, root: Path, dataset: Path, config: EvaluationRunConfig) -> EvaluationRunStore:
        root = root.expanduser().resolve()
        try:
            root.mkdir(parents=True, exist_ok=False)
        except FileExistsError as error:
            raise ValueError(f"run output already exists: {root}") from error
        lock = FileLock(str(root / ".run.lock"))
        lock.acquire(timeout=0)
        now = _timestamp()
        manifest = EvaluationRunManifest(
            run_id=uuid4().hex,
            status="running",
            created_at=now,
            updated_at=now,
            dataset=dataset_identity(dataset),
            config=config,
        )
        store = cls(root, lock, manifest)
        store._write_json(store.manifest_path, manifest.model_dump(mode="json"))
        store.record_event(RunStartedEvent(run_id=manifest.run_id, timestamp=now))
        return store

    @classmethod
    def resume(cls, root: Path, dataset: Path, config: EvaluationRunConfig) -> EvaluationRunStore:
        root = root.expanduser().resolve()
        manifest_path = root / "run-manifest.json"
        if not manifest_path.is_file():
            raise ValueError(f"no resumable run manifest: {root}")
        lock = FileLock(str(root / ".run.lock"))
        lock.acquire(timeout=0)
        manifest = EvaluationRunManifest.model_validate_json(manifest_path.read_bytes())
        if manifest.dataset != dataset_identity(dataset) or manifest.config != config:
            lock.release()
            raise ValueError("run output is incompatible with this dataset or configuration")
        if manifest.status == "completed":
            lock.release()
            raise ValueError("run output is already completed")
        return cls(root, lock, manifest)

    @property
    def manifest_path(self) -> Path:
        return self.root / "run-manifest.json"

    def close(self) -> None:
        self._lock.release()

    def record_event(self, event: EvaluationProgressEvent) -> None:
        if event.run_id != self.manifest.run_id:
            raise ValueError("event run id does not match the active run")
        with (self.root / "events.jsonl").open("a", encoding="utf-8") as handle:
            handle.write(event.model_dump_json() + "\n")
            handle.flush()
            os.fsync(handle.fileno())

    def write_checkpoint(self, checkpoint: EvaluationCheckpoint) -> None:
        self._write_json(
            self.root / "checkpoints" / f"{checkpoint.index:08d}.json",
            checkpoint.model_dump(mode="json"),
        )

    def load_checkpoints(self) -> dict[int, EvaluationCheckpoint]:
        directory = self.root / "checkpoints"
        if not directory.is_dir():
            return {}
        return {
            checkpoint.index: checkpoint
            for path in sorted(directory.glob("*.json"))
            if (checkpoint := EvaluationCheckpoint.model_validate_json(path.read_bytes()))
        }

    def publish_final(
        self, results: Iterable[EvaluationCheckpoint], summary: dict[str, int | float | None]
    ) -> RunCompletedEvent:
        ordered = sorted(results, key=lambda item: item.index)
        self._write_json(
            self.root / "results.json", [item.result.model_dump(mode="json") for item in ordered]
        )
        self._write_json(self.root / "summary.json", summary)
        now = _timestamp()
        self.manifest = self.manifest.model_copy(
            update={"status": "completed", "updated_at": now, "completed_at": now}
        )
        self._write_json(self.manifest_path, self.manifest.model_dump(mode="json"))
        event = RunCompletedEvent(run_id=self.manifest.run_id, timestamp=now, summary=summary)
        self.record_event(event)
        return event

    def fail(self, error: str) -> RunFailedEvent:
        now = _timestamp()
        self.manifest = self.manifest.model_copy(
            update={"status": "failed", "updated_at": now, "completed_at": now}
        )
        self._write_json(self.manifest_path, self.manifest.model_dump(mode="json"))
        event = RunFailedEvent(run_id=self.manifest.run_id, timestamp=now, error=error)
        self.record_event(event)
        return event

    def _write_json(self, path: Path, payload: Any) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.{uuid4().hex}.tmp")
        try:
            with temporary.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
            directory_fd = os.open(path.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary.exists():
                temporary.unlink()


def _timestamp() -> str:
    return datetime.now(UTC).isoformat()
