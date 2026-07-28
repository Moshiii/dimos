## Why

Inactive control tasks cannot observe hardware state outside `compute()`, so command handlers cannot validate requests against the coordinator's authoritative observation without reading hardware directly or adding task-specific coordinator wrappers. Stateful tasks need a generic, lock-safe way to pull the latest complete coordinator state while preserving activity semantics and keeping optional task capabilities out of `ControlCoordinator`.

## What Changes

- Add a stateless `ControlTaskContext` that gives registered `BaseControlTask` instances access to the coordinator's latest complete, immutable state observation.
- Cache each tick observation in `ControlCoordinator` before active-task filtering, clear it across runtime discontinuities, and bind or detach task contexts with registration.
- **BREAKING** Make declared task commands a strict remote allowlist: missing tasks and undeclared commands raise errors, while valid command results pass through unchanged.
- **BREAKING** Give `JointTrajectoryTask` the canonical name `joint_trajectory`, make it obtain start-state validation data through its context, and remove caller-supplied current positions.
- **BREAKING** Remove trajectory-specific methods, imports, fields, cardinality checks, and result handling from `ControlCoordinator`.
- Migrate manipulation execution, blueprints, tests, and documentation to the canonical task name and generic task-command route.

## Capabilities

### New Capabilities

- `control-task-context`: Coordinator observation ownership, immutable snapshots, context binding, and task-side state access outside `compute()`.
- `control-task-commands`: Strict manifest-declared task-command dispatch and optional trajectory-task execution through the generic command route.

### Modified Capabilities

None.

## Impact

- Affects the control task lifecycle, tick loop, coordinator command dispatch, trajectory task, manipulation execution manager, manipulator blueprint helpers, and their tests.
- Removes the coordinator-level trajectory RPC interface and custom trajectory-task names.
- Tightens reflective task invocation into an explicit declared-command contract.
- Requires callers of trajectory execution to target the canonical `joint_trajectory` task through the generic coordinator command interface.
