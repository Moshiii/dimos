## MODIFIED Requirements

### Requirement: Canonical real-time single-case command
`dimos eval run <case>` SHALL dispatch either a simulator-scene case with live CodePolicy and periodic private goal validation or a one-shot external-benchmark episode with live CodePolicy, explicit terminal submission, and benchmark-native result validation as one synchronous local attempt. The command SHALL use the same agent/authentication, output, JSON, quiet, case privacy, evidence, normalized outcome, and presentation conventions as frozen QA and SHALL NOT require a separate live-evaluation, simulator, container, blueprint, preparation, or runtime-selection command.

#### Scenario: Run one simulator case
- **WHEN** a developer invokes `dimos eval run <case> --output=<path>` with a valid real-time simulator-scene case and installed case-bound provider
- **THEN** the runner executes exactly one attempt and reports its case, scene, task, result, duration, attempt identity, and artifact path without printing private validator material

#### Scenario: Run one external benchmark case
- **WHEN** a developer invokes `dimos eval run <case> --output=<path>` with a valid external-benchmark episode case
- **THEN** the runner prepares case-bound dependencies, starts the benchmark container and DimOS stack, runs exactly one episode, collects the native result, cleans up, and reports the normalized outcome without another startup command

#### Scenario: Preserve frozen QA dispatch
- **WHEN** a developer invokes the existing command with a frozen-memory case
- **THEN** the existing frozen source, interaction, validation, progress, and result behavior remains available without simulator or container dependencies

### Requirement: Evaluator-owned simulated runtime lifecycle
For a simulator-scene case, the real-time evaluator SHALL load the case-bound simulation provider, materialize the requested scene and robot binding, build and start the case-bound DimOS blueprint, and wait for required simulator, sensor, odometry, live Memory2, Porcelain, and motion-control readiness. For an external-benchmark case, it SHALL prepare and start the case-bound one-shot OCI runtime, wait for its public gateway and reset readiness, build and start the case-bound DimOS blueprint, establish the public protocol handshake, and wait for the case-declared public observations, map, live Memory2, Porcelain, submission, and motion-control readiness. It SHALL stop every simulator, container, and DimOS resource that it started after every terminal path.

#### Scenario: Simulator-scene runtime becomes ready
- **WHEN** the case-bound provider and DimOS blueprint start and every required readiness condition is met
- **THEN** the evaluator records readiness evidence and proceeds to source preparation

#### Scenario: External benchmark runtime becomes ready
- **WHEN** the one-shot container verifies and resets its episode, its public gateway handshakes with the DimOS blueprint, and every declared public stream and control becomes ready
- **THEN** the evaluator records correlated readiness evidence and starts the evaluated agent and episode deadline

#### Scenario: Runtime startup fails
- **WHEN** provider discovery, dependency preparation, OCI startup, scene materialization, blueprint construction, protocol handshake, process startup, or readiness fails
- **THEN** Pi is not started, partial evidence is retained, owned resources are cleaned up, and the outcome is failed/not-evaluated

## ADDED Requirements

### Requirement: External benchmark terminal control
For an external-benchmark case, the evaluation control loop SHALL keep one Pi session active until the first accepted agent submission, benchmark-owned deadline, user interruption, or unrecoverable infrastructure failure. Agent text and ordinary DimOS navigation completion MUST NOT determine task success. After accepted submission or healthy deadline, the evaluator SHALL wait boundedly for the benchmark-native terminal artifact before normalizing the outcome.

#### Scenario: Agent submits terminal action
- **WHEN** the case-bound submission skill reports an accepted terminal submission
- **THEN** the evaluator stops new agent work and motion, waits for the correlated native result, and derives task success only from its authoritative native success metric

#### Scenario: External benchmark reaches its deadline
- **WHEN** the benchmark-owned deadline expires without agent submission and runtime health remains valid
- **THEN** the benchmark finalizes its native timeout result and the evaluator reports a completed task failure when the authoritative success metric is false

#### Scenario: Native result is unavailable
- **WHEN** the benchmark runtime cannot finalize or deliver a required correlated terminal artifact
- **THEN** the evaluator stops the episode and reports failed/not-evaluated with retained diagnostics

### Requirement: External benchmark evidence and privacy
Every external-benchmark attempt SHALL retain the compiled public and private case snapshots, preparation receipt, image and mounted-asset digests, OCI invocation receipt, public handshake, ordered lifecycle events, public observation metadata, agent and CodePolicy records, accepted controls, terminal submission if any, benchmark-native result, cleanup diagnostics, attempt manifest, and normalized terminal outcome. Private live benchmark state MUST NOT appear in public evidence or agent-visible interfaces.

#### Scenario: Inspect a completed external attempt
- **WHEN** an authorized reviewer opens a finalized external-benchmark attempt
- **THEN** the evidence reconstructs preparation, runtime identity, public observations, agent activity, controls, submission or timeout, native scoring, normalization, and cleanup on one correlated timeline

#### Scenario: Render public external progress
- **WHEN** an external-benchmark attempt runs without `--quiet`
- **THEN** standard error reports public preparation and lifecycle activity while withholding private goal, route, progress, and metric values until terminal result reporting

### Requirement: Bounded external runtime cleanup
On external-benchmark success, task failure, timeout, infrastructure failure, interruption, or presentation failure, the evaluator SHALL best-effort stop new agent actions, abort the active Pi turn, interrupt CodePolicy execution, cancel motion, close the public bridge, close Pi and CodePolicy sessions, stop the DimOS runtime, terminate the owned OCI container, release attempt sockets and locks, and preserve all valid evidence. Cleanup failures SHALL NOT erase the primary terminal evidence.

#### Scenario: Interrupt an external attempt
- **WHEN** the user interrupts while the agent, robot motion, or benchmark container is active
- **THEN** the evaluator performs bounded cleanup, preserves the valid evidence prefix, and writes a best-effort interrupted infrastructure outcome

#### Scenario: Container resists termination
- **WHEN** the owned container does not stop within its graceful cleanup limit
- **THEN** the evaluator applies its bounded forced-stop policy, continues cleaning other owned resources, and records the cleanup diagnostic
