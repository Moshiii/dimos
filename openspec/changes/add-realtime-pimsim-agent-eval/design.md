## Context

The canonical evaluation model separates `source`, `task`, `interaction`, and private `validator` contracts, and `dimos eval run <case>` already executes frozen-memory QA cases with an external Pi session, standalone CodePolicy, retained evidence, and normalized outcomes. Real-time simulation is described by the same conceptual stack but is not available through that CLI.

The `cc/chore/pimsim-integration` branch adds DimOS-side simulation-provider and scene-control entry points. Its Go2 `gotobed` system test starts an agentic blueprint, populates apartment spatial memory through a fixed exploration route, returns the robot to `(3.0, 0.0, 0.52)`, sends `go to the bed`, waits for `/goal_reached`, and then checks that odometry is within two metres of the private semantic bounds for `queen size bed`.

This change turns that behavior into a canonical evaluation case. PiMSim remains an unchanged external dependency. The evaluator uses the existing provider and scene-control interfaces, starts an ordinary non-agentic DimOS blueprint, owns external Pi and CodePolicy, and privately checks goal satisfaction while the system runs.

## Goals / Non-Goals

**Goals:**

- Run the PiMSim-backed Go2 `gotobed` evaluation with `dimos eval run <case>`.
- Keep one case format and result/evidence model for static QA and real-time simulation.
- Bind the simulator provider, scene, robot, DimOS blueprint, preparation recipe, interaction deadline, and validator identity into the case fingerprint.
- Let the evaluator own startup, readiness, source preparation, goal polling, timeout, cancellation, evidence, and cleanup.
- Keep Pi and standalone CodePolicy outside the DimOS blueprint and expose updating `memory` plus Porcelain `app` during the live attempt.
- Reuse the existing PiMSim integration without adding PiMSim APIs or stronger scene-attestation machinery.

**Non-Goals:**

- Running one authored case against interchangeable simulator providers.
- Changing PiMSim scene composition, reset semantics, or semantic-query behavior.
- Building a multi-case scheduler, distributed evaluator, leaderboard, or security sandbox.
- Replacing the existing frozen-QA execution path or its case packages.
- Requiring `/goal_reached` as evaluation truth.
- Generalizing every historical DimSim-native evaluator in the first implementation.

## Decisions

### The case binds the complete simulated environment

Add a `SimulatorSceneSource` containing `scene`, `simulation_provider`, `robot`, `dimos_blueprint`, and an optional versioned `preparation` recipe. These fields are part of `EvalCase.fingerprint`. The CLI does not offer source or runtime overrides.

This treats the scene as the evaluation source while declaring the concrete backend that materializes it. Changing PiMSim, the robot, or the blueprint creates a different case, which matches the intended use: this case will not be compared across runtime substitutions.

Alternative considered: select `--runtime=unitree-go2-pimsim` on the CLI. This was rejected because it permits the same case identity to execute with materially different environment behavior.

### Live timing and private goal identity are case contracts

Add a bounded `LiveCodePolicyInteraction` with a positive `timeout_seconds` and one-attempt session lifetime. Add a `PeriodicGoalValidatorRef` with a revision, safe relative private path, and content digest. The private document selects a typed predicate and its poll interval.

For the first case, the predicate is semantic-object proximity: query `queen size bed`, read current robot position, compute distance to its planar bounds, and succeed at or below two metres. The private target query and measurements never enter the agent prompt, CodePolicy results, live Memory2 namespace, or public outcome.

Alternative considered: ask Pi or DimOS to report completion through `/goal_reached`. This was rejected because an evaluated system's self-report is not private ground truth.

### One CLI dispatches by case contracts

The supported command is:

```bash
uv run dimos eval run \
  dimos/benchmark/realtime_sim/cases/go2-apartment-go-to-bed/case.json \
  --output=/tmp/dimos-eval-go-to-bed
```

The case determines static versus live execution and all environment semantics. Existing agent/authentication, output, `--json`, and `--quiet` options remain operational configuration because they select the evaluated agent or presentation rather than redefine the case.

Alternative considered: retain the separate `python -m dimos.benchmark.agent_eval run --config ...` live smoke command. This was rejected because it creates a parallel runner and makes authored live cases behave differently from static cases.

### The evaluator owns real-time attempt control

The real-time evaluator is a deep module with one primary interface: execute a validated case with the selected agent condition and return an attempt result. Its implementation performs:

1. Load and verify the case and private validator before starting Pi.
2. Load the case-bound simulation provider and materialize the requested scene and robot binding.
3. Build and start the case-bound DimOS blueprint, wait for required sensor, odometry, Memory2, and Porcelain readiness, and retain startup evidence.
4. Execute any case-bound source preparation before the task deadline.
5. Start one fresh standalone live CodePolicy workspace and one fresh external Pi session.
6. Run Pi's observe/reason/act turns while the evaluation control thread periodically checks the private goal and watches the monotonic deadline and runtime health.
7. On goal satisfaction, timeout, interruption, or infrastructure loss, stop new agent work, abort the current Pi turn, cancel robot motion, capture final evidence, and clean up evaluator-owned processes.

The interaction implementation manages the agent-facing session only. It does not decide task success or own the episode deadline. The goal checker is read-only and returns a typed observation; the evaluator decides terminal state.

Alternative considered: let each simulator backend expose `start_evaluation()` and `wait_result()`. This was rejected for this path because it would require PiMSim-specific evaluation changes and couple agent-session control to simulator-native result machinery.

### The original source preparation remains explicit

The `gotobed` source selects an apartment spatial-memory preparation recipe. Before the agent and task clock start, the evaluator follows the existing provider-neutral apartment exploration route, waits for the resulting odometry/observations, respawns the robot at `(3.0, 0.0, 0.52)`, and waits for odometry convergence. This preserves the useful starting context of the system test without embedding target-specific private data in public memory.

Preparation failure is an infrastructure failure and Pi is not started. The route and start pose are source configuration, not validator material.

Alternative considered: start with empty live memory and require the evaluated agent to explore the entire apartment. This would test a different task and would not reproduce the existing `gotobed` behavior.

### Goal polling uses existing DimOS-side interfaces

The PiMSim adapter uses its registered scene-control interface to obtain semantic object bounds and the existing DimOS observation/control path to obtain current robot position. The implementation does not add a benchmark endpoint to PiMSim. The evaluator samples after source preparation and periodically during the live attempt; the first satisfied observation terminates the attempt.

At the deadline, the evaluator performs a final goal check before recording timeout so a goal reached at the boundary is not lost solely to polling cadence. Timeout is a completed task failure. Loss of required simulator, DimOS, agent, or validation state is an infrastructure failure with task result `not_evaluated`.

### Merge only the required PiMSim integration behavior

Merge `cc/chore/pimsim-integration` before implementing the evaluator, resolve conflicts against the current branch, and retain its provider discovery, scene-control loading, Go2 binding, provider-neutral scenario data, and relevant tests. Do not copy the pytest harness into the evaluator and do not modify the external PiMSim repository. Validate the installed PiMSim integration package against the merged DimOS provider interface during preflight.

### Evidence follows the shared attempt model

The attempt retains the private and public case projections, source/startup/preparation receipts, Pi and CodePolicy evidence, agent actions, periodic private goal observations, final private score, lifecycle events, cleanup diagnostics, attempt manifest, and normalized outcome. Public progress may say that goal monitoring is active, but it must not print the target query, distance, bounds, or predicate progress.

### Visualization remains invocation-level presentation

The real-time runtime honors the process-global DimOS `viewer`, `rerun_open`,
and `rerun_web` settings instead of forcing visualization off. These flags are
resolved before the `eval` subcommand and affect only observer presentation;
they do not enter the case fingerprint, agent prompt, validator, or outcome.
This permits Rerun Web for an otherwise identical attempt while retaining
`--viewer none` for unattended execution.

## Risks / Trade-offs

- **[PiMSim and DimOS provider versions drift]** → Validate the provider and scene-control entry points before reserving the live attempt and fail with an actionable preflight diagnostic.
- **[The fixed exploration route is slow or flaky]** → Keep it as a named source-preparation recipe with readiness and per-step timeouts, retain partial evidence, and test the recipe independently from Pi.
- **[Agent cancellation cannot immediately stop an in-flight action]** → Stop new Pi work first, invoke DimOS motion cancellation, use bounded cleanup waits, and retain cleanup failures without hiding the primary terminal reason.
- **[Polling misses a short-lived predicate]** → Use a case-bound poll interval and a final deadline check; the first predicate is stable spatial proximity rather than a transient event.
- **[Private simulator facts leak through diagnostics]** → Separate public progress from private artifacts and test that target query, bounds, and distance are absent from public case, agent input, CodePolicy results, and compact CLI output.
- **[Merging the provider branch introduces unrelated changes]** → Audit the merge diff, preserve current APIs where conflicts occur, and verify the focused provider, scene-control, blueprint, and existing evaluation test suites before live acceptance.

## Migration Plan

1. Merge `cc/chore/pimsim-integration` and establish passing provider/scene-control contract tests.
2. Add the new case and private-goal models without changing existing frozen variants.
3. Add real-time source preparation, goal checking, and attempt control behind the generic evaluator.
4. Extend `dimos eval run` dispatch while preserving the frozen path and its output.
5. Add the checked-in `gotobed` case and focused unit/integration tests.
6. Run the credentialed PiMSim CLI acceptance attempt and retain its artifact path in implementation notes.

Rollback consists of removing the new discriminated case variants and live CLI dispatch while leaving the merged simulator provider usable by ordinary DimOS commands. Existing frozen cases require no migration.

## Open Questions

No product-level questions remain. Implementation must confirm the exact compatible PiMSim integration revision and the narrow DimOS readiness signals used before source preparation.
