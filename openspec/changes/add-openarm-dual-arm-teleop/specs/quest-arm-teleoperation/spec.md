## ADDED Requirements

### Requirement: Collection-based Quest hand bindings
The system SHALL represent Quest arm teleoperation with one task class configured by exactly one or two unique operator-hand bindings. Each binding SHALL identify a unique left or right operator hand, a unique target frame, and optional gripper configuration.

#### Scenario: Single-arm Quest task
- **WHEN** a Quest task is configured with one valid hand binding
- **THEN** that task controls the bound frame as a single-arm teleoperation task

#### Scenario: Bimanual Quest task
- **WHEN** a Quest task is configured with valid left and right bindings for one robot model
- **THEN** that same task class controls both bound frames as one bimanual task

#### Scenario: Invalid binding collection
- **WHEN** the configuration has zero bindings, more than two bindings, duplicate hands, duplicate frames, or unresolved frames or gripper joints
- **THEN** task construction fails before teleoperation starts

### Requirement: Quest-relative target mapping
The Quest task SHALL own controller samples, engagement state, controller reference poses, robot-frame reference poses, and relative-to-absolute frame-target mapping. Quest-specific mapping state SHALL NOT be required by the shared pose-target IK base.

#### Scenario: Engaged controller motion
- **WHEN** an engaged operator hand moves relative to its captured controller reference
- **THEN** the task applies that relative transform to the corresponding captured robot-frame reference and supplies the resulting absolute frame target to the shared IK core

### Requirement: Atomic bimanual engagement
A two-binding Quest task SHALL emit commands only while both bound operator hands are engaged. The transition into engagement SHALL capture both hands' controller and robot-frame references atomically, and release of either hand SHALL disengage the complete task and clear both sides' pending target state.

#### Scenario: Only one hand engaged
- **WHEN** exactly one bound operator hand is engaged
- **THEN** the task remains inactive and emits no arm or gripper command

#### Scenario: Both hands engage
- **WHEN** both bound operator hands become engaged and fresh samples and joint state are available
- **THEN** the task captures both reference pairs as one transition and becomes eligible to emit one combined command

#### Scenario: Either hand releases
- **WHEN** an active bimanual task observes either operator hand disengage
- **THEN** it clears both references and targets and emits no further command until both hands engage again

### Requirement: Complete and fresh bimanual snapshots
The Quest task SHALL solve a bimanual control tick only when it has a fresh controller pose for every binding. Pose callbacks SHALL update samples without independently activating one side.

#### Scenario: One pose stream becomes stale
- **WHEN** either bound controller pose exceeds the configured timeout
- **THEN** the complete bimanual task becomes inactive and emits no joint command

#### Scenario: Both pose streams are fresh
- **WHEN** both bound controller poses are within the configured timeout while both hands are engaged
- **THEN** the task snapshots both samples and supplies both frame targets to one Pink step

### Requirement: Per-hand gripper mapping
Each Quest hand binding with a gripper SHALL map that hand's analog trigger to its configured open and closed positions. A bimanual task SHALL include both gripper positions in the same ordered joint command as the arm result only while the full task is active.

#### Scenario: Both triggers change while engaged
- **WHEN** an active bimanual task receives different analog trigger values for the left and right hands
- **THEN** the emitted joint command contains independently mapped left and right gripper positions

#### Scenario: Trigger changes while disengaged
- **WHEN** trigger values change while the bimanual engagement condition is false
- **THEN** the task emits no gripper command

### Requirement: Distinct controller stream routing
The coordinator SHALL expose distinct left and right Cartesian pose inputs that retain operator-hand identity and route by task name to corresponding Quest task callbacks. The button stream SHALL remain broadcast to interested Quest tasks.

#### Scenario: Both controllers target one task
- **WHEN** the Quest module stamps its left and right pose messages with the same bimanual task name
- **THEN** the coordinator delivers each message to the matching left or right callback on that one task

#### Scenario: Controllers target separate tasks
- **WHEN** a mixed-arm Quest module stamps its controller messages with different task names
- **THEN** the coordinator routes each hand to its named single-binding task

### Requirement: Mock-default OpenArm bimanual blueprint
The system SHALL provide a runnable `teleop-quest-openarm` blueprint containing one two-binding Quest task, `openarm_bimanual_model_config()`, both OpenArm grasp-frame targets, both arm joint sets, both grippers, a Viser-backed manipulation module, and a coordinator-side joint-trajectory task over both arm joint sets. This blueprint SHALL construct an explicit in-memory whole-body adapter by default and MUST NOT select physical OpenArm hardware implicitly.

#### Scenario: Default blueprint construction
- **WHEN** `teleop-quest-openarm` is resolved without a physical-hardware-specific selection
- **THEN** its coordinator contains one bimanual Quest task and one higher-priority arm trajectory task, its manipulation module uses the same bimanual model with Viser, and its OpenArm hardware component uses the `mock_whole_body` adapter initialized at the canonical all-zero pose

#### Scenario: Bimanual command in the mock blueprint
- **WHEN** both Quest hands are engaged and provide fresh pose and trigger inputs to the running blueprint
- **THEN** one task emits a combined command for both OpenArm arms and grippers to the in-memory whole-body hardware

#### Scenario: One-arm Viser IK on the bimanual model
- **WHEN** Viser targets one OpenArm planning group while the other arm is unselected
- **THEN** Pink solves the selected target without mutating its read-only configuration directly and holds the unselected arm at its seed

#### Scenario: Canonical zero-state solve
- **WHEN** OpenArm control or planning receives a reachable frame target from the canonical all-zero state
- **THEN** the shared limit-aware posture tuning supplies a deterministic inward direction without changing the mock start or adding a control-loop random restart

### Requirement: Existing Quest arm cases migrate to the unified task
All in-repository single-arm and mixed-arm Quest blueprints SHALL use the new hand-binding task configuration and distinct controller routing without retaining the parser-specific legacy task configuration.

#### Scenario: Existing single-arm blueprint
- **WHEN** an existing one-controller manipulator blueprint is constructed
- **THEN** it contains one Quest task with one hand binding and the appropriate robot target frame

#### Scenario: Existing mixed-arm blueprint
- **WHEN** the existing two-manipulator Quest blueprint is constructed
- **THEN** it contains two independent one-binding Quest tasks rather than one bimanual task
