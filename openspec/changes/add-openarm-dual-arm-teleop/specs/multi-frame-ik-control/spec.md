## ADDED Requirements

### Requirement: Frame-based pose-target task configuration
The system SHALL configure streaming pose-target IK tasks with one `RobotModelConfig`, an explicit controlled-joint set, and one or more named target frames. It SHALL validate controlled joints, model joint mappings, and target frames before the task begins producing commands.

#### Scenario: Valid single-frame configuration
- **WHEN** a task is configured with a known controlled-joint set and one frame present in the robot model
- **THEN** the task is constructed with that frame as its pose target

#### Scenario: Valid multi-frame configuration
- **WHEN** a task is configured with multiple unique frames present in one robot model
- **THEN** the task is constructed with all frames in one kinematic context

#### Scenario: Invalid model binding
- **WHEN** a configured controlled joint, joint mapping, or target frame cannot be resolved in the robot model
- **THEN** task construction fails before the control loop starts

### Requirement: Coherent multi-frame Pink step
The system SHALL pass all active frame targets for a robot to one bounded Pink differential-IK step and SHALL produce at most one ordered joint-position result for the configured controlled joints per control tick.

#### Scenario: Two active frame targets
- **WHEN** a control tick contains fresh targets for two frames in one robot model
- **THEN** Pink solves both targets together in one differential-IK problem and returns one joint result

#### Scenario: One active frame target
- **WHEN** a control tick contains one fresh frame target
- **THEN** the same Pink step machinery solves the single target without requiring a different task class or backend

### Requirement: Bounded control-tick execution
The control path SHALL perform one Pink differential-IK update per active coordinator tick. It MUST NOT run a planning convergence loop, random restart loop, world collision query, or unbounded solve inside that tick.

#### Scenario: Active streaming target
- **WHEN** an active pose-target task is computed for one coordinator tick
- **THEN** it invokes one bounded Pink frame-target update and returns control to the coordinator

### Requirement: Shared Pink solving machinery
The system SHALL use the same internal Pink model, task construction, joint mapping, and differential-step machinery for control and planning. Planning SHALL retain its planning-group-based public API and SHALL privately translate planning-group tip links into frame targets before iterating the shared step to convergence.

#### Scenario: Planning multi-target solve
- **WHEN** planning receives pose targets scoped by planning groups
- **THEN** it translates each target to the group's tip-link frame and iterates the shared Pink step under the existing planning result contract

#### Scenario: Control frame-target solve
- **WHEN** control receives named frame targets
- **THEN** it calls the shared Pink step directly without constructing or resolving planning groups

### Requirement: Concrete Pink tuning
The pose-target control core SHALL accept `PinkKinematicsConfig` and SHALL apply its solver, timing, task-cost, damping, gain, and safety settings to the Pink tasks used by the control step. The change SHALL NOT introduce a generic control-solver interface.

#### Scenario: Customized Pink settings
- **WHEN** a task is configured with non-default Pink tuning values
- **THEN** the frame tasks and differential-IK solve use those values

#### Scenario: Joint-limit posture margin
- **WHEN** Pink is configured with a positive joint-limit posture margin and a seed coordinate is within that margin of a finite limit
- **THEN** the posture task targets that coordinate inward while the solve remains seeded from measured state

#### Scenario: One selected planning group
- **WHEN** planning targets one group of a robot that has other unselected joint groups
- **THEN** Pink updates the selected joints while preserving every unselected joint at its seed position through its supported configuration-update API

### Requirement: Joint command safety and arbitration
The pose-target control core SHALL warm-start from coordinator joint state, accept bounded-joint feedback only within the configured tolerance around nominal model limits, normalize accepted feedback to the configured inward command margin, saturate generated bounded-joint commands to that inward margin, reject non-finite or over-delta joint updates, preserve configured joint ordering, and emit `JointCommandOutput` through normal resource arbitration. This feedback policy SHALL apply only to bounded Pink-controlled joints in streaming control and SHALL NOT relax iterative planning limits or alter additional outputs such as grippers.

#### Scenario: Feedback just outside a bounded joint limit
- **WHEN** measured feedback is outside a nominal bounded-joint limit by no more than the configured feedback tolerance
- **THEN** the streaming Pink seed is normalized directly to the inward command margin and solving continues with Pink's safety break enabled

#### Scenario: Feedback beyond tolerance
- **WHEN** measured feedback exceeds a nominal bounded-joint limit by more than the configured feedback tolerance
- **THEN** the task emits no command for that tick, remains engaged, and rate-limits repeated warnings for the same joint boundary

#### Scenario: Generated command at a bounded joint limit
- **WHEN** a streaming Pink step produces a finite bounded-joint position outside the configured inward command interval
- **THEN** the position is saturated into that interval before the existing per-tick joint-delta check

#### Scenario: Unbounded or additional joint
- **WHEN** a streaming task controls an unbounded IK joint or emits an additional non-IK joint such as a gripper
- **THEN** the bounded-joint feedback tolerance and command margin do not alter that joint

#### Scenario: Valid IK update
- **WHEN** Pink returns a finite update within the configured joint-delta limit
- **THEN** the task emits one servo-position command ordered by configured coordinator joint names

#### Scenario: Unsafe IK update
- **WHEN** Pink returns a non-finite value or a joint update beyond the configured limit
- **THEN** the task emits no joint command for that tick

#### Scenario: Missing seed state
- **WHEN** any required controlled-joint position is unavailable and no valid warm start exists
- **THEN** the task emits no joint command for that tick

### Requirement: Coordinator remains kinematics-neutral
The control coordinator SHALL route pose inputs to registered task callbacks and arbitrate their declared joint claims without interpreting robot frames, kinematic models, Pink tasks, or planning groups.

#### Scenario: Multi-frame task arbitration
- **WHEN** a pose-target task claims the joints used for multiple target frames
- **THEN** the coordinator arbitrates that claim exactly like any other joint-position control task
