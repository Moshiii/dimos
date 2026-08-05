## ADDED Requirements

### Requirement: Canonical real-time single-case command
`dimos eval run <case>` SHALL dispatch a simulator-scene, embodied-instruction, live-CodePolicy, periodic-goal case as one synchronous local attempt. The command SHALL use the same agent/authentication, output, JSON, quiet, case privacy, evidence, and normalized outcome conventions as frozen QA and SHALL NOT require a separate live-evaluation command or runtime-selection option.

#### Scenario: Run one simulator case
- **WHEN** a developer invokes `dimos eval run <case> --output=<path>` with a valid real-time case and installed case-bound provider
- **THEN** the runner executes exactly one attempt and reports its case, scene, task, result, duration, attempt identity, and artifact path without printing private validator material

#### Scenario: Preserve frozen QA dispatch
- **WHEN** a developer invokes the existing command with a frozen-memory case
- **THEN** the existing frozen source, interaction, validation, progress, and result behavior remains available without simulator dependencies

### Requirement: Evaluator-owned simulated runtime lifecycle
The real-time evaluator SHALL load the case-bound simulation provider, materialize the requested scene and robot binding, build and start the case-bound DimOS blueprint, and wait for required simulator, sensor, odometry, live Memory2, Porcelain, and motion-control readiness before starting the evaluated agent. It SHALL stop all simulator and DimOS resources that it started after every terminal path.

#### Scenario: Runtime becomes ready
- **WHEN** the provider and DimOS blueprint start and every required readiness condition is met
- **THEN** the evaluator records readiness evidence and proceeds to source preparation

#### Scenario: Runtime startup fails
- **WHEN** provider discovery, scene materialization, blueprint construction, process startup, or readiness fails
- **THEN** Pi is not started, partial evidence is retained, owned resources are cleaned up, and the outcome is failed/not-evaluated

### Requirement: Case-bound source preparation
When the source names a preparation recipe, the evaluator SHALL complete that recipe after live runtime readiness and before starting the Pi session or interaction deadline. Preparation SHALL use public source behavior only, retain progress and completion evidence, and fail the attempt as infrastructure failure if it cannot establish the declared start state.

#### Scenario: Prepare spatial memory
- **WHEN** a live source selects the apartment spatial-memory preparation recipe
- **THEN** the evaluator executes its provider-neutral exploration route, restores the declared task start pose, verifies odometry convergence, and only then starts the evaluated agent

#### Scenario: Preparation fails before dispatch
- **WHEN** the route, respawn, observation, or final-pose readiness cannot complete within its infrastructure limits
- **THEN** the task prompt is not dispatched and the attempt reports failed/not-evaluated

### Requirement: External live Pi and CodePolicy interaction
Each real-time attempt SHALL create one fresh standalone CodePolicy workspace and one fresh external Pi session. CodePolicy SHALL expose updating read-only `memory` and the ready DimOS Porcelain `app`; the DimOS blueprint MUST NOT contain an internal evaluated LLM agent. One Pi session SHALL remain active for the episode and MAY receive neutral continuations while the private goal remains pending and time remains.

#### Scenario: Agent observes and acts
- **WHEN** live source preparation succeeds
- **THEN** Pi receives the exact public task, can inspect updating Memory2 and act through supported DimOS capabilities, and its policy calls and actions are retained as attempt evidence

#### Scenario: Early Pi turn does not claim success
- **WHEN** a Pi turn returns text before the private goal is satisfied
- **THEN** the evaluator does not pass the task and may continue the same session without providing private progress or directional hints

### Requirement: Evaluator-owned periodic goal control
The evaluation control loop SHALL invoke the private goal predicate periodically while the simulator, DimOS, and agent remain active. The first satisfied observation SHALL terminate the attempt as completed/passed; expiration of the case deadline after a final goal check SHALL terminate it as completed/failed; loss of required runtime or validation health SHALL terminate it as failed/not-evaluated. Agent text and `/goal_reached` MUST NOT determine task success.

#### Scenario: Private goal becomes satisfied
- **WHEN** a periodic observation reports that the case predicate is satisfied before or at the deadline
- **THEN** the evaluator records the passing observation, aborts remaining agent work, cancels active motion, and finalizes completed/passed

#### Scenario: Episode reaches its deadline
- **WHEN** the final deadline observation remains unsatisfied and infrastructure is healthy
- **THEN** the evaluator stops the episode and finalizes completed/failed with terminal reason `episode_timeout`

#### Scenario: Goal observation becomes unavailable
- **WHEN** the evaluator cannot obtain required private world or robot state while the attempt is active
- **THEN** it stops the episode and finalizes failed/not-evaluated with retained diagnostics

### Requirement: Real-time evidence and privacy
Every real-time attempt SHALL retain public and private case snapshots, runtime and source-preparation receipts, Pi and CodePolicy records, agent actions, ordered lifecycle events, private periodic and final goal observations, a private score when validation completes, cleanup diagnostics, one attempt manifest, and one normalized terminal outcome. Private target identity, bounds, distances, and predicate progress MUST NOT appear in the public case, agent input, CodePolicy results, progress renderer, or compact public result.

#### Scenario: Inspect a completed real-time attempt
- **WHEN** an authorized reviewer opens a finalized attempt
- **THEN** the evidence reconstructs source preparation, agent activity, goal observations, terminal arbitration, score, and cleanup on one correlated timeline

#### Scenario: Render public progress
- **WHEN** a real-time attempt runs without `--quiet`
- **THEN** standard error reports public lifecycle and agent activity while withholding private goal parameters and measurements

### Requirement: Bounded real-time cleanup
On success, timeout, infrastructure failure, interruption, or presentation failure, the evaluator SHALL best-effort stop new agent actions, abort the active Pi turn, interrupt active CodePolicy execution, cancel robot motion, close the Pi and CodePolicy sessions, stop the DimOS runtime, and stop the simulator resources it owns. Cleanup failures SHALL be retained and MUST NOT erase the primary terminal evidence.

#### Scenario: User interrupts an attempt
- **WHEN** the user interrupts while Pi or robot motion is active
- **THEN** the evaluator performs bounded cleanup, preserves the valid evidence prefix, and writes a best-effort interrupted infrastructure outcome

#### Scenario: Cleanup partially fails
- **WHEN** one owned resource cannot be stopped within its cleanup limit
- **THEN** the evaluator continues cleaning the remaining resources and records the cleanup diagnostic with the outcome
