## 1. Canonical Evaluation Contracts

- [x] 1.1 Add strict immutable models for `SourceSpec`, `TaskSpec`, `InteractionSpec`, private `ValidatorRef`, and compiled `EvalCase` with canonical fingerprints.
- [x] 1.2 Add public/private case projections and tests proving private validator material cannot enter the agent-visible case or task.
- [x] 1.3 Add `AttemptRequest`, agent condition, runtime binding, typed `AgentOutcome`, `Prediction`, private `Score`, and normalized infrastructure/task outcome models.
- [x] 1.4 Define source preparation, interaction driver, validator lifecycle, agent adapter, and evidence sink protocols without simulator- or frozen-QA-specific fields.
- [x] 1.5 Add model and protocol tests for missing components, invalid discriminators, stable fingerprints, source reuse across independent tasks, and operational-versus-task outcome invariants.

## 2. Normalized Recording Progress

- [x] 2.1 Extend frozen source and manifest models with finite normalized progress constrained to `[0, 1]` plus the resolved relative and absolute cutoff.
- [x] 2.2 Implement deterministic resolution over the sealed nonempty-stream recording range, including exact `0` start and exact `1` end behavior.
- [x] 2.3 Update preparation to accept one or more normalized progress selections, select runtime-cadence maps, and retain authored/resolved provenance without recomputing it at serve time.
- [x] 2.4 Preserve a documented relative-seconds compatibility path for existing callers while requiring new authored eval cases to use normalized progress.
- [x] 2.5 Add tests for start, end, interior, duplicate, out-of-range, non-finite, pre-first-map, per-stream-boundary, and manifest round-trip behavior.
- [x] 2.6 Update the preparation and serving CLI help and frozen-QA documentation with normalized-progress examples.

## 3. Plain CodePolicy Core

- [x] 3.1 Extract configuration, session/evidence records, kernel lifecycle, execution locking, bounded output, timeout recovery, reset, and observer state into a plain `CodePolicySession` with no DimOS module imports.
- [x] 3.2 Add strict frozen and live environment configurations and move namespace bootstrap generation behind the environment union.
- [x] 3.3 Bootstrap frozen sessions with read-only `FrozenMemoryStore` as `memory` and no `app`, and live sessions with read-only live Memory2 `memory` plus porcelain-backed `app`.
- [x] 3.4 Make `CodePolicyModule` a temporary compatibility wrapper delegating to the extracted session core while preserving its current RPC, skill schema, receipt, observer, and timeout behavior.
- [x] 3.5 Move and expand tests to exercise the plain core directly and run the existing module tests against the wrapper.
- [x] 3.6 Add a live-memory consistency test showing a long-lived read-only session observes newly committed system-writer data according to the supported Memory2 contract.

## 4. Standalone CodePolicy MCP Service

- [x] 4.1 Implement a standalone host that owns `CodePolicySession`, serves MCP directly, and does not construct a `Module`, `Blueprint`, worker, RPC client, or `ModuleCoordinator`.
- [x] 4.2 Expose exactly the compatible trusted-unsandboxed `python_exec` schema and keep readiness, reset, interrupt, observer, receipt, and record operations on a distinct host-only control surface.
- [x] 4.3 Add a process launcher/control client that selects an available endpoint, detects readiness, returns the fresh session identity, collects evidence, and performs bounded shutdown.
- [x] 4.4 Add standalone frozen and live CLI entry points suitable for developer use without making fixed ports or local paths part of case identity.
- [x] 4.5 Replace the blueprint-backed frozen service implementation with the standalone host and retain source/derived hash verification at startup.
- [x] 4.6 Add service tests for exact inventory, direct invocation without RPC, persistent namespace, reset isolation, frozen namespace contents, live namespace contents, startup failure, timeout recovery, interruption, and teardown after failure.

## 5. Shared Agent-Evaluation Engine Cleanup

- [x] 5.1 Move `PiTurn` and Pi session/factory protocols out of the live runner and remove the Pi process adapter's import dependency on DimSim orchestration.
- [x] 5.2 Generalize Pi authentication binding, `PythonExecBroker`, call logging, native Pi session evidence, artifact references, and attempt storage for use by any canonical case interaction.
- [x] 5.3 Implement the shared attempt engine that verifies a compiled case, reserves evidence, prepares the source and private validator, runs the interaction, finalizes validation, cleans up unconditionally, and writes exactly one normalized outcome.
- [x] 5.4 Split generic attempt artifacts from backend-specific sidecars so frozen attempts do not carry nullable DimSim release, reset, or native-result fields.
- [x] 5.5 Add engine tests covering pass, wrong answer, parser failure, source failure, interaction failure, validator failure, partial evidence, cleanup failure, interruption, and exactly-once terminal outcome.
- [x] 5.6 Document the canonical extension points and require future generic evaluations to add typed source/interaction/validator drivers instead of new top-level runner architectures.

## 6. Frozen QA Interaction and Validation

- [x] 6.1 Implement the frozen Memory2 source compiler/driver that resolves one prepared normalized progress and emits verified public binding plus private provenance receipts.
- [x] 6.2 Implement the runner-owned frozen CodePolicy interaction that starts a fresh standalone service, validates the one-tool inventory, creates a fresh Pi session, delivers the public task, captures at least one turn, and tears down both processes.
- [x] 6.3 Add the public integer terminal protocol requiring exactly one terminal `ANSWER: <integer>` marker while preserving explanatory text.
- [x] 6.4 Implement deterministic marked-integer parsing and immutable prediction records bound to case, attempt, Pi session, CodePolicy session, and parser revision.
- [x] 6.5 Implement the private exact-integer validator and score record, including correct, incorrect, and malformed-prediction outcomes without leaking the oracle.
- [x] 6.6 Add frozen-attempt evidence finalization for case, source manifest, MCP inventory, CodePolicy receipt/calls, Pi session/prompts, final response, prediction, score, events, manifest, and outcome.
- [x] 6.7 Add fake-agent tests and a real standalone-service/scripted-agent integration test over a small prepared recording.

## 7. Live DimSim Migration

- [x] 7.1 Represent the existing DimSim smoke as a canonical source, task, live CodePolicy interaction, and private native validator without changing its selected public instruction or contract.
- [x] 7.2 Move authoritative reset/readiness and native evaluation start/wait/cancel behavior into live source and validator drivers.
- [x] 7.3 Move Pi continuation, realtime completion, motion cancellation, and porcelain attachment into the live interaction driver backed by runner-owned standalone CodePolicy.
- [x] 7.4 Remove `CodePolicyModule` and its agent-facing `McpServer` from the external-Pi DimSim evaluation blueprint while retaining the normal robot stack discoverable through porcelain.
- [x] 7.5 Port the existing live runner, backend, CLI, evidence, and cleanup tests to the shared engine and prove unchanged pass, timeout, native failure, continuation, interruption, and artifact behavior.
- [ ] 7.6 Run and document the existing manual DimSim Pi smoke through the migrated shape before deleting the superseded top-level orchestration path.

## 8. Hong Kong Office North-Star

- [ ] 8.1 Have a human author enumerate the rooms represented by the complete Hong Kong office recording, state hallway/open-plan/visible-only counting policy, cite recording evidence, and record the proposed private integer oracle.
- [ ] 8.2 Obtain explicit independent review of the room inventory and expected count; do not derive or revise the oracle from the evaluated Pi answer.
- [x] 8.3 Add the public authored case selecting `go2_hongkong_office` at normalized progress `1.0`, exact question `How many rooms in total?`, integer response, frozen CodePolicy interaction, and pinned existing Pi condition.
- [x] 8.4 Prepare and verify the progress-`1.0` bundle from the LFS recording, retaining exact source/derived hashes, resolved end timestamp, stream boundaries, and final runtime-cadence map selection.
- [x] 8.5 Add a deterministic real-recording integration that launches standalone CodePolicy, verifies `memory` and absence of `app`, executes at least one memory query through `python_exec`, parses a scripted answer, scores it, and checks all required evidence.
- [x] 8.6 Add a documented one-command credentialed Pi invocation and preflight checks for recording, bundle, private oracle, built Pi adapter, credentials, and available output location.
- [ ] 8.7 Run the real Pi north-star, require at least one successful `python_exec`, a parsed prediction equal to the approved oracle, complete evidence, passed task outcome, and confirmed standalone-service teardown.
- [ ] 8.8 If the real evidence shows that text-only CodePolicy output cannot support the task, retain the failed attempt and create a separate explicit proposal for multimodal CodePolicy results rather than weakening the oracle or injecting reconstructed answers.

## 9. Verification and Documentation

- [x] 9.1 Run focused CodePolicy, frozen preparation, Pi adapter, shared engine, frozen runner, and migrated live runner test suites.
- [x] 9.2 Run formatting, lint, type checking, blueprint-registry generation checks where affected, and the repository's fast test command.
- [x] 9.3 Update architecture and benchmark documentation with the canonical case shape, standalone frozen/live namespace contract, normalized progress semantics, lifecycle ownership, privacy boundary, and evidence layout.
- [x] 9.4 Verify no canonical frozen or live evaluation constructs `CodePolicyModule`, `McpServer`, `Blueprint`, or `ModuleCoordinator` for the agent CodePolicy path.
- [ ] 9.5 Run `openspec verify` for this change and reconcile proposal, design, specs, tasks, implementation, and retained north-star evidence before archive.

## 10. Canonical Single-Case CLI

- [x] 10.1 Consolidate evaluation serialization on `BaseEvalModel` and add strict Pydantic agent-auth, agent-backend, run-config, and compact-result models.
- [x] 10.2 Add `dimos eval run <case>` with dotted `--agent.*` options, Pi/Codex-OAuth defaults, optional `--output`, and `--json`, without semantic case overrides or blueprint construction.
- [x] 10.3 Resolve private validator references relative to the immutable case document, verify them before dispatch, and keep frozen Memory2 storage bindings behind the source driver.
- [x] 10.4 Add pretty terminal result rendering plus compact Pydantic JSON output over the shared attempt engine.
- [x] 10.5 Add focused model and CLI tests for defaults, dotted overrides, static-case enforcement, auth binding safety, relative validator resolution, pretty output, and JSON output.
- [x] 10.6 Update the frozen-QA runbook to make the single-case CLI canonical and document that dataset orchestration, containers, parallelism, and scheduling are deferred.

## 11. Live Pi Evaluation Trace

- [x] 11.1 Add a typed best-effort progress interface and extend the Pi adapter protocol to forward lifecycle and visible assistant text while discarding thinking deltas.
- [x] 11.2 Emit bounded `python_exec` call/result progress and single-case preparation lifecycle without changing evidence or evaluation outcomes.
- [x] 11.3 Render progress to standard error by default, add `--quiet`, and preserve pretty/JSON standard-output contracts.
- [x] 11.4 Add TypeScript and Python tests for event filtering, ordering, bounds, observer isolation, quiet mode, and JSON separation.
- [x] 11.5 Document live progress and run the credentialed Hong Kong smoke case from the named environment binding, iterating until operational completion with at least one successful `python_exec` call.

## 12. Automatic API-Key Authentication

- [x] 12.1 Infer API-key authentication from a nonempty `OPENAI_API_KEY` when the single-case CLI receives no authentication options, while preserving explicit auth choices and custom environment bindings.
- [x] 12.2 Add hermetic CLI tests and update the runbook and authentication contract for automatic environment binding.

## 13. Evaluation Session Header

- [x] 13.1 Display the public case, source selection, question, and pending-answer marker at the beginning of the live single-case trace without exposing the private oracle.
- [x] 13.2 Add typed progress and renderer tests, document the header, and preserve `--quiet` and JSON-output behavior.
