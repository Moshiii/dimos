## ADDED Requirements

### Requirement: Complete observations are available outside compute
The control coordinator SHALL retain the latest complete `CoordinatorState` produced by the tick loop before active-task filtering and SHALL make it available to registered base control tasks through their bound context.

#### Scenario: Inactive task reads the latest observation
- **WHEN** a complete tick observation has been published and a registered base task is inactive
- **THEN** the task context returns that complete observation without activating the task or invoking `compute()`

#### Scenario: Active tasks share the published observation
- **WHEN** the tick loop publishes an observation and computes active tasks in the same tick
- **THEN** the coordinator cache and every active task receive the same immutable observation object

### Requirement: Task context is a stateless capability bundle
`ControlTaskContext` SHALL retain only coordinator-owned function handles and SHALL initially expose `get_state()` without owning observations, clocks, locks, threads, or hardware adapters.

#### Scenario: Task pulls coordinator state
- **WHEN** a bound task calls `context.get_state()`
- **THEN** the context delegates to the coordinator-owned getter and returns its cached observation

#### Scenario: No observation exists yet
- **WHEN** a bound task calls `context.get_state()` before the first complete tick or after cache invalidation
- **THEN** the context returns `None`

### Requirement: Published observations are immutable
`CoordinatorState`, its joint snapshot, and their collection fields SHALL be structurally read-only after construction.

#### Scenario: Observation is shared by reference
- **WHEN** multiple tasks receive the same published observation
- **THEN** no task can mutate its fields or collections through the observation interface

### Requirement: Registration owns context binding
The coordinator SHALL bind its shared context to each successfully registered `BaseControlTask` and SHALL detach that context when the task is removed.

#### Scenario: Base task is registered
- **WHEN** a previously unbound base task is added successfully
- **THEN** its read-only `context` property returns the coordinator's shared context

#### Scenario: Registration rolls back
- **WHEN** task registration fails after context binding
- **THEN** the task is detached and remains unregistered

#### Scenario: Task is already bound elsewhere
- **WHEN** a task bound to one coordinator is added to another coordinator
- **THEN** registration fails without replacing the existing binding

#### Scenario: Task is removed
- **WHEN** a base task is removed from its coordinator
- **THEN** its context is detached and subsequent context access raises a lifecycle error

#### Scenario: Structural task remains compatible
- **WHEN** a task implements `ControlTask` without inheriting `BaseControlTask`
- **THEN** registration remains supported and no context is injected implicitly

### Requirement: Runtime discontinuities invalidate observations
The coordinator SHALL clear its cached observation after stopping the tick loop and during runtime-state reset while preserving context bindings for tasks that remain registered.

#### Scenario: Coordinator restarts
- **WHEN** a coordinator stops and starts with its registered tasks intact
- **THEN** their contexts remain bound, return `None` before the first new tick, and return the new observation afterward

### Requirement: Observation access preserves lock ordering
The tick loop SHALL release the coordinator state lock before acquiring the task lock for task computation.

#### Scenario: Command and tick access overlap
- **WHEN** a task command reads context state while a control tick publishes another observation
- **THEN** both operations complete without cyclic acquisition of state, task, or hardware locks
