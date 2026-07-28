## ADDED Requirements

### Requirement: Declared commands form the remote task interface
The coordinator SHALL invoke only commands listed in the registered task type's `TASK_EXPOSES` manifest.

#### Scenario: Declared command is invoked
- **WHEN** a caller invokes a registered task and a manifest-declared command with valid arguments
- **THEN** the coordinator calls that command under the task lock

#### Scenario: Undeclared method exists
- **WHEN** a caller names a Python method that exists on the task but is absent from `TASK_EXPOSES`
- **THEN** the coordinator raises the same command-absence error used for an unknown command and does not call the method

### Requirement: Task and command lookup failures raise errors
`task_invoke()` SHALL raise an invocation error when the named task is unregistered and SHALL raise `AttributeError` when the command is not exposed.

#### Scenario: Task is missing
- **WHEN** a caller invokes a command on an unregistered task name
- **THEN** invocation raises instead of returning `None`

#### Scenario: Command is not exposed
- **WHEN** a caller invokes an unknown or undeclared command
- **THEN** invocation raises `AttributeError` identifying the task, command, and declared commands

### Requirement: Command arguments are validated before dispatch
The coordinator SHALL bind supplied command arguments against the declared handler signature before calling the handler.

#### Scenario: Arguments are invalid
- **WHEN** supplied arguments are missing, unexpected, or otherwise incompatible with the handler signature
- **THEN** invocation raises `TypeError` before the task handler runs

### Requirement: Command results pass through unchanged
The coordinator SHALL return a declared task command's value unchanged and SHALL NOT project it into a coordinator-specific result.

#### Scenario: Typed result is returned
- **WHEN** a declared trajectory command returns a typed semantic result
- **THEN** `task_invoke()` returns that same result to the caller

### Requirement: Joint trajectory task has one canonical identity
The trajectory-task package SHALL define and enforce the canonical task name `joint_trajectory`, and generic coordinator task-name uniqueness SHALL prevent registration of a second task with that name.

#### Scenario: Canonical JTT is registered
- **WHEN** a trajectory task is created through its constructor or registry factory
- **THEN** its task name is `joint_trajectory`

#### Scenario: Configuration supplies another name
- **WHEN** generic task configuration attempts to create a trajectory task under another name
- **THEN** task creation fails before registration

### Requirement: Joint trajectory execution pulls validation state
`JointTrajectoryTask.execute()` SHALL accept only a trajectory argument and SHALL obtain its current joint positions from the bound task context.

#### Scenario: No coordinator observation exists
- **WHEN** `execute()` is called before the context can return a complete observation
- **THEN** it returns `START_STATE_UNAVAILABLE` without activating execution

#### Scenario: Start state is available
- **WHEN** `execute()` receives a structurally valid trajectory and the context observation contains all commanded joint positions
- **THEN** it validates the first trajectory point against those positions before accepting execution

#### Scenario: Caller attempts to inject positions
- **WHEN** a remote caller supplies a current-position argument to `execute`
- **THEN** argument validation rejects the command before execution

### Requirement: Coordinator remains task-type agnostic
`ControlCoordinator` SHALL have no trajectory-specific methods, imports, fields, cardinality rules, or result handling.

#### Scenario: Coordinator has no trajectory task
- **WHEN** a coordinator is configured without `joint_trajectory`
- **THEN** its generic control, registration, and command interfaces remain available without trajectory-specific state

### Requirement: Manipulation dispatch uses the generic task command
`PlanExecutionManager` SHALL execute and cancel planned motion by invoking manifest-declared commands on `joint_trajectory` through the required injected coordinator.

#### Scenario: Generated plan is executed
- **WHEN** the manager maps a valid generated plan to a joint trajectory
- **THEN** it invokes the `execute` command on `joint_trajectory` and maps the returned JTT result into its execution outcome

#### Scenario: Execution task invocation fails
- **WHEN** generic task invocation raises before or during trajectory execution
- **THEN** the manager applies its caller-layer failure policy without requiring a coordinator trajectory result

### Requirement: Missing-task statuses are absent from JTT results
Trajectory execution and cancellation result enums SHALL describe only semantic outcomes produced by an existing JTT and SHALL NOT contain `NO_TRAJECTORY_TASK`.

#### Scenario: Canonical task is absent
- **WHEN** a caller invokes `joint_trajectory` but it is not registered
- **THEN** generic task invocation raises and no JTT semantic result is constructed
