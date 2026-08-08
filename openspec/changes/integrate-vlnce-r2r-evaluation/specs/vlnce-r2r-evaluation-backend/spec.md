## ADDED Requirements

### Requirement: One-command VLN-CE R2R attempt
The system SHALL provide a checked-in VLN-CE R2R case that `dimos eval run <case>` can prepare and execute as exactly one synchronous attempt. The command SHALL start the case-bound benchmark container, DimOS blueprint, public bridge, standalone CodePolicy workspace, and evaluated Pi session; collect the benchmark-native result; stop every resource it started; and report the normalized outcome and artifact path without requiring a separate simulator, container, or blueprint command.

#### Scenario: Run the development case from one command
- **WHEN** a developer invokes `dimos eval run` with the checked-in public-scene R2R case and a fresh output path
- **THEN** the command prepares any absent public cached dependencies, runs exactly one episode, finalizes an attempt outcome, cleans up its processes, and prints the result and artifact location

#### Scenario: Preparation fails before agent startup
- **WHEN** a pinned public dependency cannot be fetched, built, verified, or loaded and no valid cached copy exists
- **THEN** the attempt fails as infrastructure failure before starting Pi and reports a specific preparation diagnostic without starting a partially configured evaluation

### Requirement: Pinned non-gated development episode
The system SHALL include one development case that binds the public Habitat MP3D example scene `17DRP5sb8fy` to one matching official R2R/VLN-CE training episode. The case SHALL pin the scene asset, episode dataset, upstream code, image recipe, split, episode ID, scene ID, task instruction, and content digests. The runtime SHALL verify those identities and SHALL reject any instruction that differs from the selected official episode.

#### Scenario: Prepare the official public-scene case
- **WHEN** the case preparation recipe resolves the public `17DRP5sb8fy` asset and matching official episode
- **THEN** it produces content-addressed assets whose identities and digests match the compiled case without requiring Matterport credentials

#### Scenario: Reject substituted episode material
- **WHEN** the mounted scene, episode dataset, episode identity, scene identity, or copied task instruction does not match the compiled case
- **THEN** the container exits with an infrastructure result before the evaluated agent starts

### Requirement: Reproducible one-shot OCI runtime
Each attempt SHALL start a fresh OCI container from the case-pinned image digest and execute exactly one episode. The image SHALL pin its base image, VLN-CE and Habitat revisions, Python dependencies, gateway, and generated protocol code. Scene and episode data SHALL be mounted read-only and SHALL NOT be embedded in or redistributed with the image.

#### Scenario: Start a pinned episode runtime
- **WHEN** case preflight succeeds
- **THEN** the supervisor starts one container with the exact image digest, one private episode binding, one attempt-private public socket mount, and one supervisor-only terminal-result mount

#### Scenario: Container exits before a valid result
- **WHEN** the container crashes, violates its one-episode contract, or exits on a scoring path without writing a valid atomic result
- **THEN** the supervisor reports infrastructure failure, retains diagnostics, and does not fabricate a task score

### Requirement: Benchmark-owned private state and scoring
The container SHALL exclusively own episode reset, goal position, semantic annotations, reference path, trajectory scoring state, success/progress measures, episode timeout, and official VLN-CE measure implementations. None of those values SHALL appear in the DimOS process environment, filesystem view, public bridge schema or frames, agent prompt, CodePolicy results, Rerun output, or public progress display.

#### Scenario: Inspect the public runtime boundary
- **WHEN** an evaluator audits the DimOS runtime and recorded public protocol frames
- **THEN** the audit finds no private goal, reference-path, semantic-oracle, progress, success, or score data

#### Scenario: Preserve private terminal evidence
- **WHEN** an episode terminates normally
- **THEN** the native result is written only to the supervisor-owned result mount and becomes available to DimOS evaluation code only after episode termination

### Requirement: Versioned public UDS bridge
The benchmark container and `VlnceConnection` SHALL communicate through one benchmark-specific Protobuf/gRPC bidirectional stream over an attempt-private Unix-domain socket. Before actions are accepted, a compatibility handshake SHALL bind attempt, case, episode, protocol revision, coordinate frames, observation encodings, control limits, and capabilities. Every observation and command SHALL carry monotonic correlation data, and the implementation SHALL reject incompatible, stale, duplicate, malformed, or uncorrelated traffic.

#### Scenario: Establish a compatible connection
- **WHEN** both endpoints present matching attempt and protocol metadata
- **THEN** the connection records the handshake and begins publishing coherent public observation epochs

#### Scenario: Reject an incompatible endpoint
- **WHEN** endpoint identity, protocol revision, frames, capabilities, or control limits do not match the case
- **THEN** the attempt fails before Pi starts and no motion command is accepted

### Requirement: Geometry-only public observation contract
For every coherent observation epoch, `VlnceConnection` SHALL publish the official RGB and depth observations with camera calibration, benchmark-agent pose as odometry and TF, and depth projected into a camera-frustum `PointCloud2`. It SHALL also publish one complete, static, inspectable `OccupancyGrid` derived from Habitat navigability. The occupancy grid SHALL contain geometry and traversability only and MUST NOT encode semantics, the goal, reference route, or benchmark progress.

#### Scenario: Initialize the DimOS navigation stack
- **WHEN** the connection receives the initial benchmark observation and public map
- **THEN** native DimOS image, depth, camera-info, point-cloud, odometry, TF, and occupancy-grid consumers become ready without importing VLN-CE types

#### Scenario: Reject private map decoration
- **WHEN** a generated occupancy grid contains a semantic label, goal marker, reference-route marker, or progress-dependent cell
- **THEN** public-contract validation fails and the agent is not started

### Requirement: Collision-aware planar DimOS control
The bridge SHALL accept bounded planar DimOS `Twist` commands for `linear.x`, `linear.y`, and `angular.z`, integrate accepted commands at the case-declared fixed control period, and apply collision-aware Habitat motion to the VLN-CE 1.5 m-high, 0.1 m-radius cylinder with 1.25 m camera height. The agent interface MUST NOT expose teleport, reset-pose, or direct pose-setting operations. Every accepted motion update SHALL contribute to the trajectory used by the official measures.

#### Scenario: Execute a velocity command
- **WHEN** DimOS sends an in-range correlated planar velocity command during an active episode
- **THEN** Habitat advances collision-aware motion, publishes the next coherent observation, and appends the resulting pose to the benchmark trajectory

#### Scenario: Reject privileged or invalid control
- **WHEN** a caller requests teleportation, reset, direct pose mutation, an out-of-range velocity, or an action outside the active episode
- **THEN** the bridge rejects the request without changing the agent pose and records the protocol violation

### Requirement: Explicit route submission
The case-bound blueprint SHALL expose a documented `submit_route()` skill/RPC whose only benchmark effect is to submit VLN-CE `STOP` once and end the episode. The task prompt SHALL explain that meaning before the agent acts. Navigation-goal completion, zero velocity, ordinary motion cancellation, Pi text, and timeout MUST NOT be translated into agent-submitted STOP. Submission SHALL reveal no score, success, distance, or progress feedback.

#### Scenario: Agent submits its route
- **WHEN** the agent calls `submit_route()` during the active episode
- **THEN** the bridge sends one terminal submission, acknowledges only that submission was accepted, rejects later actions, and lets the benchmark finalize official measures

#### Scenario: Navigation goal finishes without submission
- **WHEN** a DimOS point-to-point navigation goal completes and the agent has not called `submit_route()`
- **THEN** the R2R episode remains active until a later submission, benchmark timeout, interruption, or infrastructure failure

### Requirement: Official VLN-CE result authority
The container SHALL finalize its configured official VLN-CE measures and atomically write one native terminal result bound to the attempt, case, episode, scene, runtime, protocol, terminal reason, and trajectory. DimOS SHALL retain every native metric unchanged, SHALL NOT recompute or augment benchmark scoring, and SHALL map only official `SUCCESS` to normalized task pass or fail.

#### Scenario: Score an unsuccessful healthy episode
- **WHEN** the episode is submitted or reaches its healthy benchmark timeout and official `SUCCESS` equals zero
- **THEN** the attempt is `completed`, the task result is `failed`, all official metrics are retained unchanged, and the command exits successfully at the infrastructure level

#### Scenario: Score a successful episode
- **WHEN** official measure finalization returns `SUCCESS` equal to one in a valid correlated native result
- **THEN** the attempt is `completed`, the task result is `passed`, and all accompanying native metrics are retained unchanged

### Requirement: Declared non-leaderboard condition
The public case, attempt manifest, native-result reference, and human-readable report SHALL identify the run as a single-scene R2R training/development evaluation under the DimOS geometry condition. They SHALL state that the complete public navmesh map and planar velocity interface differ from standard VLN-CE and SHALL NOT label the score as validation, test, or leaderboard-comparable.

#### Scenario: Report the development result
- **WHEN** the public `17DRP5sb8fy` case completes
- **THEN** its output identifies the official episode and metrics while clearly labeling the modified condition and non-comparable training-scene status

### Requirement: Independent harness and real-agent verification
The implementation SHALL verify the real public-scene runtime with a deterministic test-only controller that demonstrates official accepting and rejecting score paths, and with a normal DimOS-agent attempt that uses only the public contract. The deterministic controller SHALL remain outside the evaluated DimOS runtime. Real-agent task success SHALL be recorded but MUST NOT be an integration acceptance requirement.

#### Scenario: Verify official score behavior
- **WHEN** the deterministic verification paths run against the public-scene container
- **THEN** one controlled trajectory produces the expected official success result and one controlled trajectory or STOP produces the expected official failure result

#### Scenario: Verify the evaluated-agent path
- **WHEN** the normal `dimos eval run` development case executes with a configured agent
- **THEN** it reaches a valid completed native result, retains inspectable public evidence, and cleans up all owned resources whether the task passes or fails
