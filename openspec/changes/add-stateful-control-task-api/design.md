## Context

`TickLoop` currently constructs `CoordinatorState` once per control tick and passes it only to active tasks through `compute(state)`. Command handlers on inactive tasks therefore have no authoritative observation before first execution or after completion. PR #3183 worked around this for trajectory acceptance by adding trajectory-specific coordinator methods that read hardware on demand, but that couples `ControlCoordinator` to an optional task and creates a second hardware-read path.

The coordinator already owns task registration and serializes task commands with `_task_lock`. Hardware reads use `_hardware_lock`, so a command handler that reads adapters while holding `_task_lock` would reverse the tick path's lock order. The new interface must expose the completed tick observation without direct task-to-hardware access or lock inversion.

## Goals / Non-Goals

**Goals:**

- Let any registered `BaseControlTask` pull the latest complete coordinator observation outside `compute()`.
- Keep observation storage, synchronization, and lifecycle ownership in `ControlCoordinator`.
- Preserve the distinction between task activity and observation availability.
- Make remote task commands explicit, validated, and generic.
- Move trajectory acceptance entirely into the optional trajectory-task capability.
- Keep manipulation execution typed while routing through the generic coordinator task-command interface.

**Non-Goals:**

- Detecting a stalled control tick or applying per-task freshness limits.
- Allowing tasks to read hardware adapters directly.
- Renaming `task_invoke()` in this change.
- Adding coordinator-specific helpers for other optional task types.
- Preserving custom trajectory-task names or reflective undeclared command invocation.

## Decisions

### Coordinator-owned observation with a stateless task context

`ControlCoordinator` owns `_latest_state`, a dedicated `_state_lock`, and one frozen `ControlTaskContext`. The context retains only coordinator callback functions and initially exposes `get_state() -> CoordinatorState | None`; it owns no observation, clock, lock, thread, or hardware reference.

`TickLoop` receives one observation callback. After all hardware and IMU reads complete, it constructs an immutable `CoordinatorState`, invokes the callback, and only then filters active tasks and calls `compute(state)`. The callback releases `_state_lock` before the tick loop acquires `_task_lock`. Command handlers may therefore hold `_task_lock` while briefly calling the context getter without creating a lock cycle.

Alternatives rejected:

- Passing the entire coordinator to tasks exposes unrelated mutation and lifecycle operations and creates a broad circular dependency.
- Giving every task its own observation cache duplicates state and requires per-task publication.
- Reading hardware on demand from a task reverses lock order and does not represent one coherent tick observation.
- A stateful runtime or view object duplicates ownership that belongs in the coordinator.

### Immutable observations shared by reference

`CoordinatorState`, `JointStateSnapshot`, and their collection fields become structurally read-only. The tick loop creates one observation per tick; the coordinator cache and active `compute()` calls share that object by reference. This avoids defensive copies on every command while preventing one task from changing another task's observation.

The observation timestamp used by control tasks remains monotonic `CoordinatorState.t_now`. The context performs no age calculation. A stalled tick is a whole-control-system failure and is outside this interface.

### Registration controls context ownership

`BaseControlTask` exposes a read-only `context` property and private coordinator lifecycle hooks for binding and detachment. `add_task()` binds the shared context, rolls the binding back if registration fails, and rejects a task already bound to another coordinator. Rebinding the same context is idempotent. `remove_task()` detaches the context, after which context access raises a lifecycle error.

Tasks remain registered and bound across coordinator stop/start, matching existing coordinator behavior. Stop and `reset_runtime_state()` clear `_latest_state`; `get_state()` returns `None` until the next completed tick. External structural `ControlTask` implementations that do not inherit `BaseControlTask` remain valid but receive no context implicitly.

### Strict declared task commands

`TASK_EXPOSES` becomes the allowlist for `task_invoke()`. The coordinator:

- raises when the named task is absent;
- treats existing but undeclared Python methods exactly like unknown commands;
- validates arguments against the declared handler before calling it;
- propagates handler failures as invocation errors; and
- returns successful handler values unchanged.

The method remains named `task_invoke()` for this change. Renaming it to a more explicit command-oriented name is deferred.

### Trajectory capability owns its interface

The trajectory-task package owns the canonical name `joint_trajectory`. `JointTrajectoryTask` does not accept a configurable name, and its factory rejects mismatched generic task configuration. Generic coordinator name uniqueness therefore prevents a second canonical JTT without the coordinator importing or recognizing the trajectory class.

`JointTrajectoryTask.execute(trajectory)` pulls the observation through `self.context.get_state()`, validates the first trajectory point against `state.joints`, and returns `START_STATE_UNAVAILABLE` if no observation exists. It no longer accepts caller-supplied positions. The task retains its typed semantic acceptance, mismatch, and cancellation results, but `NO_TRAJECTORY_TASK` statuses are removed because absence is an invocation error.

`ControlCoordinator` removes trajectory imports, `_trajectory_task`, trajectory cardinality checks, and the public `execute_trajectory()` and `cancel_trajectory()` methods.

### Manipulation uses the generic route

`ManipulationModule` already requires an injected `ControlCoordinator`. `PlanExecutionManager` invokes `task_invoke()` with the canonical task name and declared `execute` or `cancel` command. The coordinator returns typed JTT results unchanged; the manager maps execution results into its caller-level `ExecutionDispatchResult` and retains its existing uncertainty policy for invocation failures.

Blueprint helpers always configure the canonical task name and remove hardware-derived or caller-overridden trajectory names.

## Risks / Trade-offs

- **Breaking custom task names and coordinator trajectory RPC callers** → Migrate all in-repo blueprints, execution code, tests, examples, and documentation in the same change; fail fast on noncanonical JTT configuration.
- **Strict allowlisting breaks reflective callers** → Audit all `task_invoke()` call sites and add legitimate commands to task manifests before removing the fallback.
- **Immutable mapping types expose compatibility issues in tests or tasks** → Update type annotations to `Mapping`, replace fixture mutation with construction-time values, and run focused control plus manipulation suites.
- **Context callback accidentally acquires task or hardware locks** → Keep `ControlTaskContext` callbacks coordinator-owned and document that helpers must be safe while `_task_lock` is held.
- **A stopped tick can leave an old observation** → Clear the cache after stopping the tick loop and during runtime reset.

## Migration Plan

1. Introduce immutable observations, coordinator caching, context binding, and lifecycle tests without migrating JTT.
2. Make task-command dispatch strict and migrate declared command callers.
3. Move JTT validation to the bound context, adopt the canonical name, and remove obsolete result statuses.
4. Remove trajectory-specific coordinator state and methods.
5. Migrate manipulation execution and all blueprint/task-name references to generic dispatch.
6. Update documentation and run focused control, manipulation, blueprint-generation, lint, and type checks.

Rollback is a single-branch revert before release because the interface changes are intentionally atomic and no persistent data migration is involved.

## Open Questions

None. The API decisions are recorded in ADR 0001.
