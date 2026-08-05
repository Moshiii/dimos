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

"""Shared single-attempt engine for canonical agent-evaluation cases."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from pydantic import BaseModel, ConfigDict, JsonValue

from dimos.benchmark.agent_eval.artifacts import ArtifactReference
from dimos.benchmark.agent_eval.case import AttemptRequest, EvalOutcome, PrivateScore
from dimos.benchmark.agent_eval.interfaces import (
    AgentAdapter,
    AttemptContext,
    EvidenceSink,
    InteractionDriver,
    SourceDriver,
    ValidatorDriver,
    ValidatorSession,
)
from dimos.benchmark.agent_eval.store import AttemptStore


class EngineResult(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)

    attempt_path: Path
    outcome: EvalOutcome
    artifacts: tuple[ArtifactReference, ...]


class AttemptEvidence(EvidenceSink):
    def __init__(self, store: AttemptStore) -> None:
        self.store = store
        self.artifacts: list[ArtifactReference] = []

    def event(self, kind: str, payload: dict[str, JsonValue] | None = None) -> None:
        self.store.append_event(kind, payload=payload)

    def artifact(self, relative_path: str, value: BaseModel | JsonValue | bytes | str) -> None:
        self.artifacts.append(self.store.write_artifact(relative_path, value))

    def reference(self, relative_path: str) -> None:
        self.artifacts.append(_reference(self.store.path, relative_path))


class AttemptEngine:
    """Coordinate source, private validator, interaction, evidence, and cleanup."""

    def __init__(
        self,
        *,
        request: AttemptRequest,
        output_root: Path,
        source: SourceDriver,
        interaction: InteractionDriver,
        validator: ValidatorDriver,
        agent: AgentAdapter,
    ) -> None:
        self.request = request
        self.output_root = output_root
        self.source = source
        self.interaction = interaction
        self.validator = validator
        self.agent = agent

    def run(self) -> EngineResult:
        store = AttemptStore(self.output_root)
        result: EngineResult | None = None
        close_error: Exception | None = None
        try:
            result = self._run_reserved(store)
        finally:
            try:
                store.close()
            except Exception as exc:
                close_error = exc
        if close_error is not None:
            return EngineResult(
                attempt_path=store.path,
                outcome=EvalOutcome(
                    attempt_id=store.attempt_id,
                    attempt_status="failed",
                    task_result="not_evaluated",
                    reason=f"attempt store cleanup failed: {type(close_error).__name__}: {close_error}"[
                        :1024
                    ],
                ),
                artifacts=result.artifacts if result is not None else (),
            )
        assert result is not None
        return result

    def _run_reserved(self, store: AttemptStore) -> EngineResult:
        evidence = AttemptEvidence(store)
        context = AttemptContext(
            attempt_id=store.attempt_id,
            path=store.path,
            request=self.request,
        )
        validator_session: ValidatorSession | None = None
        agent_outcome = None
        outcome: EvalOutcome
        completed = False
        passed = False
        reason = "infrastructure failure"
        try:
            evidence.event("attempt-created")
            evidence.artifact("case.private.v1.json", self.request.case)
            evidence.artifact("case.public.v1.json", self.request.case.public_projection())
            prepared = self.source.prepare(
                source=self.request.case.source,
                context=context,
                evidence=evidence,
            )
            evidence.event("source-prepared")
            validator_session = self.validator.prepare(
                case=self.request.case,
                prepared_source=prepared,
                context=context,
                evidence=evidence,
            )
            evidence.event("validator-prepared")
            agent_outcome = self.interaction.run(
                case=self.request.case,
                prepared_source=prepared,
                agent=self.agent,
                context=context,
                evidence=evidence,
            )
            evidence.artifact("agent-outcome.v1.json", agent_outcome)
            evidence.event("interaction-completed")
            score = validator_session.evaluate(agent_outcome)
            _validate_score(score, store.attempt_id, self.request.case.case_id)
            evidence.artifact("score.private.v1.json", score)
            evidence.event("validation-completed", {"passed": score.passed})
            completed = True
            passed = score.passed
            reason = "validator passed" if passed else "validator failed"
        except KeyboardInterrupt:
            reason = "user interrupted"
            _safe_event(evidence, "attempt-interrupted")
        except Exception as exc:
            reason = f"{type(exc).__name__}: {exc}"[:1024]
            _safe_event(evidence, "infrastructure-failure", {"diagnostic": reason})
        cleanup_errors = self._cleanup(validator_session)
        if cleanup_errors:
            _safe_event(evidence, "cleanup-failure", {"diagnostic": "; ".join(cleanup_errors)})
            completed = False
            passed = False
            reason = "; ".join(cleanup_errors)
        try:
            evidence.reference("events.jsonl")
            evidence.artifact(
                "attempt-manifest.v1.json",
                {
                    "schema_version": "1.0",
                    "attempt_id": store.attempt_id,
                    "case_id": self.request.case.case_id,
                    "case_fingerprint": self.request.case.fingerprint,
                    "agent": self.request.agent.model_dump(mode="json"),
                    "runtime": self.request.runtime.model_dump(mode="json"),
                    "agent_session_id": (
                        agent_outcome.agent_session_id if agent_outcome is not None else None
                    ),
                    "interaction_session_id": (
                        agent_outcome.interaction_session_id if agent_outcome is not None else None
                    ),
                    "artifacts": [
                        artifact.model_dump(mode="json") for artifact in evidence.artifacts
                    ],
                },
            )
        except Exception as exc:
            completed = False
            passed = False
            reason = f"attempt finalization failed: {type(exc).__name__}: {exc}"[:1024]
            _safe_event(evidence, "finalization-failure", {"diagnostic": reason})
        outcome = EvalOutcome(
            attempt_id=store.attempt_id,
            attempt_status="completed" if completed else "failed",
            task_result=("passed" if passed else "failed") if completed else "not_evaluated",
            reason=reason,
        )
        try:
            evidence.artifacts.append(store.write_eval_outcome(outcome))
        except Exception as exc:
            outcome = EvalOutcome(
                attempt_id=store.attempt_id,
                attempt_status="failed",
                task_result="not_evaluated",
                reason=f"terminal publication failed: {type(exc).__name__}: {exc}"[:1024],
            )
            _safe_event(
                evidence,
                "terminal-publication-failure",
                {"diagnostic": outcome.reason},
            )
        return EngineResult(
            attempt_path=store.path,
            outcome=outcome,
            artifacts=tuple(evidence.artifacts),
        )

    def _cleanup(self, validator_session: ValidatorSession | None) -> list[str]:
        errors: list[str] = []
        resources: tuple[tuple[str, Any], ...] = (
            ("agent", self.agent),
            ("interaction", self.interaction),
            ("validator", validator_session),
            ("source", self.source),
        )
        for name, resource in resources:
            if resource is None:
                continue
            try:
                resource.close()
            except Exception as exc:
                errors.append(f"{name}: {type(exc).__name__}: {exc}"[:1024])
        return errors


def _validate_score(score: PrivateScore, attempt_id: str, case_id: str) -> None:
    if score.attempt_id != attempt_id or score.case_id != case_id:
        raise ValueError("validator score identity mismatch")


def _reference(root: Path, relative_path: str) -> ArtifactReference:
    import hashlib

    data = (root / relative_path).read_bytes()
    return ArtifactReference(
        path=relative_path,
        sha256=hashlib.sha256(data).hexdigest(),
        size_bytes=len(data),
    )


def _safe_event(
    evidence: AttemptEvidence,
    kind: str,
    payload: dict[str, JsonValue] | None = None,
) -> None:
    try:
        evidence.event(kind, payload)
    except Exception:
        # Evidence storage is already failing; cleanup and lock release remain mandatory.
        return
