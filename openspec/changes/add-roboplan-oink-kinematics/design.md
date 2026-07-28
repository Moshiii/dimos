## Context

The manipulation stack exposes inverse kinematics through the runtime-checkable
DimOS `KinematicsSpec` Protocol. Pink is currently constructed as an independent
backend and is the default in `ManipulationModuleConfig`, even when
`world_backend="roboplan"`. RoboPlan-native path planning already follows a
different ownership model: `RoboPlanWorld` owns the scene, model, group
mappings, and lock, and the factory returns that same object as `PlannerSpec`.

RoboPlan 0.0.100, the version pinned by DimOS, includes
`roboplan.optimal_ik.Oink` in the normal RoboPlan distribution. OInK depends on
RoboPlan-native scene, group, frame, and joint-index objects and retains
request-specific numerical state. Building a generic adapter over arbitrary
`WorldSpec` implementations would require a second RoboPlan scene and ongoing
state synchronization.

The existing public `KinematicsSpec` contract supplies pose-target groups,
auxiliary groups, an optional partial `JointState` seed, convergence tolerances,
collision policy, and retry count. `RoboPlanModel` already maps DimOS planning
groups and public joint names to RoboPlan-native groups and names.

## Goals / Non-Goals

**Goals:**

- Make `RoboPlanWorld` satisfy `KinematicsSpec` using OInK.
- Select OInK by default for a RoboPlan world and Pink by default for other
  worlds while preserving explicit backend overrides.
- Support every non-overlapping pose-target and auxiliary-group selection
  represented by `RoboPlanModel.groups`, including composite multi-robot
  selections.
- Preserve `KinematicsSpec` seed, tolerance, collision, result-ordering, and
  status semantics.
- Keep shared RoboPlan scene state correct under concurrent world, planner, and
  IK use.

**Non-Goals:**

- Making OInK work with non-RoboPlan `WorldSpec` implementations.
- Runtime switching between kinematics backends.
- Exposing OInK tuning in public configuration.
- Adding an OInK self-collision barrier or validating intermediate numerical
  search states as a trajectory.
- Persisting or caching OInK instances across requests.
- Supporting target frames other than `world`.

## DimOS Architecture

Add a discriminator-only `RoboPlanKinematicsConfig` with
`backend: Literal["roboplan"]` to `ManipulationKinematicsConfig` and the legacy
name conversion. Make `ManipulationModuleConfig.kinematics` optional so omitted
configuration remains distinguishable from an explicit
`PinkKinematicsConfig`. `kinematics_name`, when present, remains an explicit
legacy override.

Resolve omitted configuration once in `create_planning_specs` after the world
backend is known:

- RoboPlan world: `RoboPlanKinematicsConfig`
- any other world: `PinkKinematicsConfig`

Extend backend validation so the RoboPlan kinematics configuration requires a
RoboPlan world. `create_kinematics` receives the selected world and returns that
same `RoboPlanWorld` instance for the native backend, mirroring native planner
construction. Explicit Pink, Jacobian, and Drake optimization construction
continues to return independent solver objects.

`RoboPlanWorld` implements both methods of `KinematicsSpec`. `solve` is a
convenience wrapper that resolves a robot only when it has exactly one
pose-targetable planning group, then delegates to `solve_pose_targets`.
`solve_pose_targets` uses `PlanningGroupSelection` and `RoboPlanModel.groups` to
validate a non-overlapping selection and construct the native composite group.
DimOS/public joint names are used at the API boundary; native names and velocity
indices are confined to `RoboPlanModel`.

There are no new modules, streams, transports, blueprint components, RPC
references, skills, MCP schemas, or CLI commands. Existing manipulation APIs
continue to call the injected `KinematicsSpec`. No generated blueprint registry
input changes, so `all_blueprints.py` regeneration is not expected.

## Decisions

### World-owned native kinematics

`RoboPlanWorld` directly implements `KinematicsSpec`; there is no shallow OInK
adapter class. The world already owns every backend-specific resource that OInK
needs, and returning the same object avoids exposing those internals or
duplicating the scene.

Keep request validation, joint-index mapping, retries, and convergence in a
private `roboplan_oink` module. This is an implementation seam rather than a
second `KinematicsSpec`: `RoboPlanWorld` remains responsible for identity,
locking, context capture, and scene restoration.

The entire request holds `RoboPlanWorld._lock`. Before applying a seed, it
snapshots the shared scene configuration; a `finally` block restores the
snapshot on success, failure, or exception. This serializes planning and other
scene operations with IK but makes the request atomic.

### Request-local OInK lifecycle

Construct one OInK instance per `solve_pose_targets` request and reuse it across
all numerical iterations and restart attempts. Release it when the request
returns. Use RoboPlan's default `FrameTaskOptions`, `PositionLimit`, and
regularization, with an internal cap of 100 OInK steps per attempt.

OInK's `solveIk` produces one displacement step, not a complete DimOS IK
result. After each step, evaluate forward kinematics independently. A candidate
converges only when every target is within both caller-provided tolerances.
Report the maximum position error and maximum orientation error across targets.

### Targets, groups, and seeds

Version one accepts only `PoseStamped.frame_id == "world"`. Construct each
native Cartesian target with the RoboPlan root convention and the group's
native tip frame. Unsupported frames and group selections return
`IKStatus.UNSUPPORTED`.

Support multiple targets, disjoint groups on one robot, groups spanning
multiple robots, auxiliary groups, and auxiliary-only selections when the
selection exists in `RoboPlanModel.groups`. Auxiliary-only requests return the
seeded/current selected configuration without numerical solving.

For attempt one, apply the caller's named seed and fill missing selected robot
joints from the live world state. Later attempts randomize only joints belonging
to pose-targeted groups within their joint limits. Auxiliary-group joints remain
at seeded positions. Successful `JointState` results contain only selected
joints, in planning-group selection order.

### Collision and failure semantics

Do not add `SelfCollisionBarrier` in the first version. Numerical iterations are
search states, not an executable path. If `check_collision=True`, validate each
converged endpoint using the world collision API with all selected robot states
applied together. If false, return the first converged candidate without
collision validation.

Across attempts, retain the closest non-converged candidate and remember
whether any converged endpoint was rejected for collision. Rank closeness by:

```text
max(position_error / position_tolerance,
    orientation_error / orientation_tolerance)
```

If no acceptable solution is found, return `IKStatus.COLLISION` when any
candidate converged but collided; otherwise return the closest
`IKStatus.NO_SOLUTION`. Failure results have `joint_state=None`. Unexpected
RoboPlan setup or solve errors fail the request with an actionable message and
never fall back to Pink.

### Atomic RoboPlan dependency

Import `roboplan.optimal_ik` with the other RoboPlan modules. Do not add a
separate dependency, feature probe, or fallback path: OInK is part of the pinned
RoboPlan package. A broken or incomplete RoboPlan installation fails when the
RoboPlan stack is selected.

## Safety / Simulation / Replay

IK only computes joint endpoints; it does not execute hardware motion.
Collision validation checks the final composite configuration when requested,
but the first release offers no OInK collision barrier and makes no claim that
the numerical iterations form a safe path. Existing motion planning remains
responsible for producing a collision-free executable trajectory.

The behavior is identical for RoboPlan-backed hardware and simulation worlds.
Replay does not alter solver selection or numerical behavior. Manual QA should
exercise the existing manipulation IK API in a RoboPlan simulation, verify a
reachable target, verify collision rejection, and confirm subsequent world and
planning operations see the pre-IK scene state.

## Risks / Trade-offs

- Holding the scene lock for up to `max_attempts * 100` steps can delay planning
  and state queries. Correctness is preferred initially; benchmark before
  introducing scene copies or persistent solver state.
- RoboPlan numerical defaults may need robot-specific tuning. Keep them internal
  until hardware evidence identifies a stable public control.
- Endpoint-only collision validation can spend work converging to colliding
  candidates. A future collision barrier may improve this, but changes solver
  behavior and requires separate validation.
- World-sensitive defaults change behavior for callers that omitted kinematics
  configuration. Explicit `backend="pink"` provides a direct compatibility
  path.

## Migration / Rollout

Existing explicitly configured backends continue unchanged. Callers that need
the former RoboPlan-world/Pink combination must set `kinematics.backend=pink`.
No dependency or lockfile update should be necessary because OInK ships with
RoboPlan. No generated registry update is expected.

Roll out with unit tests for configuration/factory resolution and OInK request
behavior, followed by an integration test against the pinned RoboPlan package
and simulation QA through the existing manipulation inverse-kinematics surface.
Update manipulation documentation and preserve the architectural decisions in
the existing ADRs.

## Open Questions

None blocking implementation. Persistent OInK caching, non-world target-frame
transforms, configurable tuning, and collision barriers are explicitly deferred
until benchmarks or hardware testing justify them.
