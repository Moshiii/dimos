## Why

DimOS currently defaults manipulation kinematics to Pink even when the planning
world and planner are backed by RoboPlan. That splits inverse kinematics from
the native scene, planning-group mappings, collision state, and model semantics
already owned by the RoboPlan world.

RoboPlan bundles its OInK solver, so DimOS can provide native inverse kinematics
without another dependency or a parallel world representation. The backend
selection policy should prefer this native integration while preserving Pink as
the generic default and as an explicit override.

## What Changes

- Add RoboPlan-native inverse kinematics for RoboPlan planning worlds, covering
  single and multiple pose targets, auxiliary groups, and composite
  multi-robot selections represented by the world model.
- Select RoboPlan OInK when kinematics configuration is omitted for a RoboPlan
  world; retain Pink as the omitted-config default for other planning worlds.
- Preserve explicit backend configuration, including explicit Pink with a
  RoboPlan world.
- Fail fast when RoboPlan-native kinematics is selected but RoboPlan cannot be
  loaded; do not silently fall back to Pink.
- Validate convergence against the public kinematics tolerances and optionally
  reject converged endpoints that collide in the RoboPlan scene.
- Restrict the initial RoboPlan backend to world-frame targets and omit an OInK
  self-collision barrier.

## Affected DimOS Surfaces

- Modules/streams: manipulation planning world, kinematics configuration and
  factory selection, the `KinematicsSpec` implementation, RoboPlan model/group
  mappings, and IK result behavior; no stream contract changes.
- Blueprints/CLI: manipulation stacks that omit kinematics configuration gain a
  world-sensitive default; no new blueprint or CLI command.
- Skills/MCP: no direct skill or MCP schema changes; manipulation clients using
  the existing inverse-kinematics API observe the selected backend.
- Hardware/simulation/replay: RoboPlan-backed hardware and simulation stacks use
  native OInK by default; replay has no special behavior. Endpoint collision
  validation remains controlled by the existing request flag.
- Docs/generated registries: update manipulation kinematics documentation; no
  generated blueprint registry change is expected.

## Capabilities

### New Capabilities

- `manipulation-kinematics`: Backend selection, target support, convergence,
  collision validation, failure reporting, and state-isolation behavior for
  manipulation inverse kinematics.

### Modified Capabilities

None.

## Impact

RoboPlan-backed stacks that relied on the implicit Pink default will now use
OInK unless they configure Pink explicitly. The public IK method signatures and
result types remain unchanged. RoboPlan remains one atomic dependency:
`roboplan.optimal_ik` is not separately installed or probed.

Implementation requires focused unit tests for backend selection, group and
joint mapping, retries, convergence, collision outcomes, and scene restoration,
plus integration coverage against the pinned RoboPlan package. Documentation
must explain automatic selection, explicit overrides, world-frame limitations,
and the absence of a self-collision barrier in the initial version.
