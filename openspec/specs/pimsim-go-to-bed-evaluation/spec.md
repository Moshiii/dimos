# pimsim-go-to-bed-evaluation Specification

## Purpose

Define the canonical PiMSim-backed Go2 apartment bed-navigation evaluation and
its private acceptance behavior.

## Requirements

### Requirement: Authored PiMSim Go2 bed-navigation case
The repository SHALL contain one authored real-time case whose source selects the apartment scene, PiMSim simulation provider, Unitree Go2 robot, ordinary non-agentic Go2 DimOS blueprint, and apartment spatial-memory preparation; whose public task is `Go to the bed`; whose interaction is bounded live CodePolicy; and whose validator is a private periodic semantic-object-proximity goal.

#### Scenario: Load the checked-in case
- **WHEN** the canonical CLI loads the Go2 apartment bed-navigation case package
- **THEN** strict validation binds its source, task, interaction, private validator reference, and fingerprint without requiring source or runtime CLI options

#### Scenario: Keep the evaluated agent external
- **WHEN** the case-bound blueprint is built
- **THEN** it contains the ordinary Go2 spatial runtime and no internal LLM agent competing with the external Pi session

### Requirement: Reproduce the apartment task starting context
The case's source preparation SHALL populate DimOS spatial memory by physically driving the existing provider-neutral apartment exploration route through `/cmd_vel`, without teleporting between its waypoints. It SHALL then perform one scene-control placement of the simulated robot at `(3.0, 0.0, 0.52)` in the canonical DimOS world frame and wait until odometry is within the configured start tolerance before task dispatch.

#### Scenario: Preparation completes
- **WHEN** PiMSim and the Go2 DimOS runtime are healthy
- **THEN** the moving-camera exploration observations are available in live Memory2 and the task begins from the declared start pose

#### Scenario: Task start cannot be established
- **WHEN** exploration or start-pose convergence fails
- **THEN** Pi does not receive the task and the attempt reports failed/not-evaluated

### Requirement: Private bed-proximity success predicate
The private validator SHALL resolve semantic bounds for `queen size bed` through the existing PiMSim scene-control integration and report success when the current robot position is at most `2.0` metres from those bounds. The evaluator SHALL check this predicate periodically until success or the live interaction deadline without requiring a PiMSim modification or `/goal_reached` message.

#### Scenario: Robot reaches the bed
- **WHEN** the robot position enters or reaches the two-metre distance threshold before the deadline
- **THEN** the next goal observation passes the case and terminates the active agent attempt

#### Scenario: Robot never reaches the bed
- **WHEN** the deadline expires and the final robot-to-bed distance remains above two metres
- **THEN** the attempt completes with a failed task and retains the final private distance evidence

### Requirement: PiMSim provider compatibility preflight
The case runner SHALL use the DimOS simulation-provider and scene-control entry points supplied by the installed PiMSim integration package. It SHALL verify both entry points and their required interfaces before reserving live agent work and SHALL NOT require changes to the PiMSim repository.

#### Scenario: Compatible PiMSim integration is installed
- **WHEN** exactly one `pimsim` provider and scene-control adapter satisfy the required interfaces
- **THEN** the evaluator may start the case-bound simulation and DimOS runtime

#### Scenario: PiMSim integration is absent or incompatible
- **WHEN** either entry point is missing, duplicated, fails to load, or does not satisfy the expected interface
- **THEN** preflight fails with an actionable diagnostic before Pi starts

### Requirement: Credentialed CLI acceptance
The documented acceptance procedure SHALL run the checked-in case with `uv run dimos eval run <case> --output=<path>` and valid Pi credentials. Acceptance SHALL require a fully finalized attempt with complete infrastructure evidence; the task result SHALL independently report whether Pi reached the bed.

#### Scenario: End-to-end task passes
- **WHEN** the credentialed Pi attempt reaches the private bed-proximity predicate and all required artifacts finalize
- **THEN** the CLI reports completed/passed and the artifact directory contains the correlated real-time evidence

#### Scenario: Agent times out cleanly
- **WHEN** Pi does not reach the bed but the simulator, DimOS, validator, and evidence pipeline remain healthy through the deadline
- **THEN** the CLI reports completed/failed rather than an infrastructure failure
