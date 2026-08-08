## ADDED Requirements

### Requirement: Named Pink task composition
`PinkIK` SHALL expose a protected task-construction hook that returns an ordered mapping from stable string names to native Pink task instances.

#### Scenario: Subclass changes one inherited objective
- **WHEN** a `PinkIK` subclass calls the parent task-construction hook and replaces or tunes one named auxiliary entry
- **THEN** all other inherited entries SHALL remain in their original order and participate in the solve unchanged

#### Scenario: Subclass adds an unknown Pink task type
- **WHEN** a subclass adds a valid native or custom Pink task under a new auxiliary name
- **THEN** the common backend SHALL pass that task to `pink.solve_ik` without requiring a generic configuration field for its parameters

### Requirement: Mandatory frame objectives
The final task mapping SHALL contain one reserved `frame/<frame_name>` objective compatible with each commanded target frame, and the backend SHALL validate this invariant after the most-derived construction hook returns.

#### Scenario: Multi-frame objective is complete
- **WHEN** a caller supplies two frame targets and a subclass returns compatible reserved tasks for both frames
- **THEN** the backend SHALL update both task targets and include both tasks in the same Pink solve

#### Scenario: Subclass omits a commanded frame
- **WHEN** a subclass removes or misidentifies a reserved frame task
- **THEN** the backend SHALL reject the task stack before invoking `pink.solve_ik`

### Requirement: Persistent exclusively owned control task stack
Each pose-target control-task instance SHALL exclusively own its injected `PinkIK` instance, and that backend SHALL retain one task stack for each cached control context rather than reconstructing its task objects every tick.

#### Scenario: Consecutive control ticks reuse tasks
- **WHEN** two control ticks use the same robot model, controlled-joint order, and ordered target-frame selection
- **THEN** the backend SHALL reuse the same named task instances while refreshing the configuration and frame targets from the new tick

#### Scenario: Distinct control tasks use the same subclass
- **WHEN** two control-task instances are constructed with separate instances of the same `PinkIK` subclass
- **THEN** neither task's mutable Pink task state SHALL be visible to the other

### Requirement: Fixed structure with mutable task state
After a control task stack is created and validated, its names, ordering, and task identities SHALL remain fixed while per-tick hooks MAY mutate fields on the contained task objects.

#### Scenario: Frame target changes
- **WHEN** a new pose arrives for an existing commanded frame
- **THEN** the backend SHALL mutate the existing frame task's target rather than reconstructing or replacing that task

#### Scenario: Lifecycle hook attempts structural mutation
- **WHEN** a per-tick lifecycle hook receives the task stack
- **THEN** it SHALL receive a read-only mapping that prevents adding, removing, or replacing entries

### Requirement: Auxiliary task lifecycle hooks
`PinkIK` SHALL provide protected before-solve and after-solve hooks for updating dynamic auxiliary-task inputs and recording successful solver output.

#### Scenario: Temporal task records velocity
- **WHEN** a subclass includes a temporal Pink task and a solve succeeds
- **THEN** its after-solve hook SHALL receive the solved velocity and timestep before configuration integration

#### Scenario: Failed solve has no successful output callback
- **WHEN** `pink.solve_ik` raises or fails before returning a velocity
- **THEN** the backend SHALL NOT invoke the after-solve hook with a fabricated result

### Requirement: Common algorithm and safety ownership
Robot-specific subclasses SHALL customize task composition and task-local lifecycle only; the common backend SHALL retain the planning, solve invocation, integration, mapping, feedback-limit, and command-limit algorithms.

#### Scenario: Customized streaming solve reaches final safety enforcement
- **WHEN** a subclass-defined task stack produces a streaming joint command
- **THEN** the command SHALL pass through the same feedback normalization, finite-result validation, and inward command-limit saturation as plain `PinkIK`

#### Scenario: Customized planning solve uses common convergence behavior
- **WHEN** a robot-specific `PinkIK` subclass is used for iterative planning
- **THEN** its task stack SHALL run through the common convergence, retry, joint-limit, and collision-checking workflow

### Requirement: Default behavior compatibility
Plain `PinkIK` SHALL preserve its existing frame-task parameters, posture regularization semantics, public planning API, and public streaming API.

#### Scenario: Default streaming posture target
- **WHEN** plain `PinkIK` performs consecutive streaming ticks
- **THEN** its default posture task SHALL target each tick's measured configuration rather than retaining the first tick's posture target

#### Scenario: Existing caller supplies no custom backend
- **WHEN** a pose-target control task is constructed without an injected backend
- **THEN** it SHALL continue to construct and use plain `PinkIK` from `PinkKinematicsConfig`

### Requirement: Backend instance injection
Robot-specific composition SHALL be able to construct a `PinkIK` subclass and inject that instance into the generic pose-target control task without subclassing the control task.

#### Scenario: Quest task receives a robot-specific backend
- **WHEN** a generic `TeleopIKTask` is initialized with a fresh robot-specific `PinkIK` instance
- **THEN** Quest engagement and target semantics SHALL remain generic while the injected backend's named task stack controls the Pink objective
