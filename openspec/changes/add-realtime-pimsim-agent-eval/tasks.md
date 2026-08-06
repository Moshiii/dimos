## 1. Integrate the PiMSim provider branch

- [x] 1.1 Merge `cc/chore/pimsim-integration`, resolve conflicts against the current branch without discarding unrelated work, and audit the resulting diff for provider, blueprint, transport, and E2E changes.
- [x] 1.2 Verify and, where needed, adapt the merged `dimos.simulation.providers` and `dimos.simulation.scene_controls` entry-point interfaces for current DimOS APIs without changing the external PiMSim repository.
- [x] 1.3 Add focused provider-discovery and scene-control compatibility tests covering missing, duplicate, incompatible, and valid PiMSim entry points.
- [x] 1.4 Run the merged branch's focused simulation/provider tests and regenerate `dimos/robot/all_blueprints.py` if the merge changes built-in blueprints.

## 2. Extend canonical case contracts

- [x] 2.1 Add strict `SimulatorSceneSource` and source-preparation reference models with scene, simulation provider, robot, DimOS blueprint, and optional preparation fields.
- [x] 2.2 Extend the live CodePolicy interaction model with a finite positive case-bound timeout while preserving one-attempt session lifetime.
- [x] 2.3 Add `PeriodicGoalValidatorRef` plus typed private goal documents for semantic-object proximity and positive polling intervals.
- [x] 2.4 Extend case fingerprinting and public projection tests to cover every new source, interaction, and validator field and prove that private goal contents do not enter the public projection.
- [x] 2.5 Add strict validation tests for incomplete simulator sources, invalid deadlines, unsafe private paths, malformed goals, and private digest mismatches.

## 3. Build real-time source lifecycle and preparation

- [x] 3.1 Implement case-driven provider loading and simulation binding creation for the selected scene and robot.
- [x] 3.2 Implement evaluator-owned DimOS blueprint startup, readiness checks for required simulator/sensor/odometry/Memory2/Porcelain/motion capabilities, and startup evidence receipts.
- [x] 3.3 Implement the apartment spatial-memory preparation recipe using the provider-neutral exploration route, declared final start pose, per-step limits, and odometry-convergence verification.
- [x] 3.4 Add deterministic fake-provider tests for successful startup/preparation and each pre-agent infrastructure failure path.
- [x] 3.5 Implement bounded cleanup for evaluator-owned DimOS and simulator resources and test partial cleanup failure retention.

## 4. Implement live agent execution and goal control

- [x] 4.1 Adapt standalone CodePolicy to a live environment exposing updating read-only `memory` and the ready DimOS Porcelain `app` without adding an internal blueprint agent.
- [x] 4.2 Implement one-attempt external Pi session execution with retained policy calls, neutral continuation support, evaluator-triggered abort, and no private goal feedback.
- [x] 4.3 Implement the semantic-object-proximity goal checker using the existing scene-control semantic bounds and current robot position, returning typed private observations and scores.
- [x] 4.4 Refactor the shared attempt control so the evaluator concurrently supervises Pi activity, runtime health, periodic goal checks, and the monotonic deadline instead of delegating termination to the interaction implementation.
- [x] 4.5 Implement terminal arbitration for goal success, final deadline check, timeout, interruption, agent/runtime failure, motion cancellation, and cleanup with normalized completed/passed, completed/failed, and failed/not-evaluated outcomes.
- [x] 4.6 Add concurrency and lifecycle tests for success during an active Pi turn, early Pi return, timeout, deadline-edge success, unavailable goal state, interruption, and cancellation failures.

## 5. Unify the `dimos eval run` command

- [x] 5.1 Refactor single-case preflight and dispatch to select frozen or real-time execution from the validated case contracts while resolving private files relative to the case package.
- [x] 5.2 Preserve existing dotted agent/authentication, `--output`, `--json`, and `--quiet` options and reject any source/runtime override for simulator-scene cases.
- [x] 5.3 Extend compact human and JSON results for embodied tasks and simulator scenes without exposing private goal parameters or measurements.
- [x] 5.4 Extend live progress rendering with public runtime, preparation, agent, monitoring, terminal, and cleanup events while keeping private validator data off public output.
- [x] 5.5 Add CLI tests proving static QA remains unchanged and real-time cases dispatch without a separate runtime option or parallel live command.

## 6. Add the Go2 apartment bed-navigation case

- [x] 6.1 Add the checked-in `go2-apartment-go-to-bed` case with PiMSim, apartment scene, ordinary Go2 blueprint, spatial-memory preparation, exact public instruction, bounded live interaction, private goal reference, and valid fingerprint.
- [x] 6.2 Add the private `queen size bed` proximity goal with a two-metre threshold, case-bound polling interval, digest, and privacy tests.
- [x] 6.3 Add a fake-backed end-to-end test that loads the checked-in case through `dimos eval run`, exercises preparation and polling, and verifies completed/passed artifacts.
- [x] 6.4 Add a self-hosted PiMSim integration test covering provider startup, preparation, external agent control, semantic goal observation, timeout behavior, and evaluator-owned cleanup.

## 7. Verify and document acceptance

- [x] 7.1 Update `docs/benchmark/agent-evaluation-stack-overview.md` and the evaluation CLI documentation with the static-versus-real-time case comparison, ownership model, final command, result meanings, prerequisites, and troubleshooting.
- [x] 7.2 Run the focused case-model, attempt-engine, CLI, provider, scene-control, live CodePolicy, and Go2 evaluation test suites plus mypy and pre-commit checks for changed files.
- [x] 7.3 Execute the documented credentialed command `uv run dimos eval run <go-to-bed-case> --output=<path>` against PiMSim and retain the finalized attempt path and result in implementation notes.
- [x] 7.4 Verify the real attempt contains all required evidence, exposes no private goal material publicly, and classifies a clean goal timeout as completed/failed rather than infrastructure failure.

## 8. Enable invocation-level Rerun Web visualization

- [x] 8.1 Preserve resolved DimOS viewer and Rerun presentation settings when starting the case-bound realtime runtime, including explicit headless mode.
- [x] 8.2 Add hermetic tests proving realtime runtime configuration does not override viewer selection or change case identity.
- [x] 8.3 Document the exact Rerun Web and headless commands, then run focused tests and changed-file quality checks.

## 9. Restore original physical source preparation

- [x] 9.1 Replace per-waypoint scene-control teleportation with the original `/cmd_vel` waypoint traversal, retaining only the original final start-pose teleport and odometry check.
- [x] 9.2 Remove the teleport-only observation dwell contract, update the checked-in case fingerprint, and add a hermetic preparation sequence test.
- [x] 9.3 Run focused and real PiMSim acceptance, record any expected traversal flakiness, and update the architecture documentation.
