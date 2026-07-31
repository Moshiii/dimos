---
title: "Manipulation"
---

Motion planning and teleoperation for robotic manipulators. RoboPlan provides
the default world and native path planner.

## Quick Start

### Prompted xArm6 pick pipeline

`xarm6-prompted-pick` connects the wrist RGB-D recording directly to prompted
object localization and the GraspGenX pick pipeline. It is a real-hardware
blueprint and exposes the existing `pick` skill through MCP without starting an
LLM agent.

Start the robot stack:

```bash
dimos --xarm6-ip <ROBOT_IP> run xarm6-prompted-pick
```

For integration testing, disable physical pick execution while retaining
localization, grasp generation, IK, full candidate planning, selection, and
Viser updates:

```bash
dimos --xarm6-ip <ROBOT_IP> run xarm6-prompted-pick \
  -o promptedpickandplacemodule.execute_pick=false
```

In this mode, a successful `pick` response includes `planning_only: true` and
states that no motion was executed. The arm and gripper are not commanded, and
the module does not record a last physical pick pose. `pick_and_place` also
stops after the plan-only pick instead of proceeding to its physical place
phase. Other explicitly invoked motion skills remain live; use only `pick` or
`pick_and_place` for this pipeline test.

Wait at least 30 seconds for the active Memory2 recording window to populate,
then call the skill from another terminal:

```bash
dimos mcp call pick --arg object_name="white and red marker"
```

Open the Viser URL printed by the stack. During the pick, **Grasp / Object
Cloud** shows the localized target and **Grasp / Proposals** highlights the
candidate currently being evaluated. After selection, only the selected grasp
remains.

This command can move real hardware. Confirm that the workspace is clear, the
robot is enabled, the wrist camera is calibrated, and an operator can stop the
robot before calling `pick`. Localization failure stops before grasp generation
or motion. The localized target cloud drives grasp generation and visualization
only; it is not inserted into the collision scene.

Recent addition: the A-750 keyboard teleop blueprint is now available via:

```bash
dimos run keyboard-teleop-a750
```

### Keyboard Teleop (single command)

Each blueprint launches the full stack — keyboard UI, mock controller, IK solver, and Drake visualization:

```bash
dimos run keyboard-teleop-a750    # A-750 6-DOF
dimos run keyboard-teleop-piper   # Piper 6-DOF
dimos run keyboard-teleop-xarm6   # XArm6 6-DOF
dimos run keyboard-teleop-xarm7   # XArm7 7-DOF
```

Open the Meshcat URL printed in the terminal (default `http://localhost:7000`) to see the robot.

Keyboard controls:

| Key | Action |
|-----|--------|
| W/S | +X/-X (forward/back) |
| A/D | +Y/-Y (left/right) |
| Q/E | +Z/-Z (up/down) |
| R/F | +Roll/-Roll |
| T/G | +Pitch/-Pitch |
| Y/H | +Yaw/-Yaw |
| ESC | Quit |

### Motion Planning (two terminals)

```bash
# Terminal 1: Mock coordinator
dimos run coordinator-mock

# Terminal 2: Planner with Drake visualization
dimos run xarm7-planner-coordinator
```

Pink IK is the default solver. Tune it with nested module config overrides:

```bash
dimos run xarm7-planner-coordinator \
  -o manipulationmodule.kinematics.backend=pink \
  -o manipulationmodule.kinematics.max_iterations=100 \
  -o manipulationmodule.kinematics.dt=0.02
```

For blueprints that instantiate `PickAndPlaceModule`, use the corresponding
module prefix:

```bash
dimos run xarm-perception-sim \
  -o pickandplacemodule.kinematics.backend=pink
```

Then use the IPython client:

```bash
python -m dimos.manipulation.planning.examples.manipulation_client
```

```python skip
joints()                # Get current joints
plan([0.1] * 7)         # Plan to target
preview()               # Preview in Meshcat
execute()               # Execute via coordinator
```

### Planning backend selection

Manipulation planning separates the world backend from the planner algorithm:

- `world_backend` selects the robot/world/collision representation.
- `planner.backend` selects the path-planning algorithm.
- `kinematics.backend` selects the IK backend. The legacy `kinematics_name`
  field remains available as a compatibility shim.


```bash
dimos run xarm7-planner-coordinator
```

Select the legacy Drake world and generic RRT planner explicitly when needed:

```bash
dimos run xarm7-planner-coordinator \
  -o manipulationmodule.world_backend=drake \
  -o manipulationmodule.planner.backend=rrt_connect
```

Valid combinations:

| `world_backend` | `planner.backend` | `kinematics.backend` | Status |
|-----------------|----------------|-------------------|--------|
| `roboplan` | `roboplan` | `pink` or `jacobian` | Default path; RoboPlan-native planner |
| `drake` | `rrt_connect` | `pink` | Legacy Drake world |
| `drake` | `rrt_connect` | `jacobian` | Legacy Jacobian IK |
| `drake` | `rrt_connect` | `drake_optimization` | Drake-only IK |
| `roboplan` | `rrt_connect` | `pink` or `jacobian` | Generic RRT over RoboPlan collision checks |

Invalid combinations fail during startup instead of waiting for the first plan
request. For example, `planner.backend=roboplan` requires
`world_backend=roboplan`, and `kinematics.backend=drake_optimization` requires
`world_backend=drake`.

RoboPlan Cartesian options are supplied per planning request:

```python skip
from dimos.manipulation.planning.planners.config import (
    RoboPlanCartesianPathConfig,
)

path_config = RoboPlanCartesianPathConfig(
    speed_mode="bounded",
    max_linear_speed=0.1,
    max_angular_speed=0.5,
    max_position_error=0.005,
    max_orientation_error=0.01,
)

module.plan_cartesian_targets(
    {"arm/manipulator": (current_tcp_pose, goal_tcp_pose)},
    path_config,
)
```

The remaining settings mirror RoboPlan's standard Cartesian planner options,
including bounded and time-optimal speed modes, sample time, solver weights,
linear/angular acceleration limits, joint velocity/acceleration scaling,
TOPP-RA corner blending, joint-limit handling, and per-step attempts.

Cartesian path planning remains a low-level internal capability in this
release. `ManipulationModule.plan_cartesian_targets()` accepts an ordered
waypoint sequence for each target planning group. A sequence contains only
`PoseStamped` absolute waypoints or only `Transform` displacements relative to
the planning start, and begins at the current TCP pose or identity transform.
RoboPlan plans all target groups simultaneously. The Viser panel constructs a
two-waypoint absolute path for interactive planning. There is no skill, MCP
tool, or CLI motion command yet.

Install the manipulation dependencies:

```bash
uv sync --extra manipulation --inexact
```

The `manipulation` extra includes RoboPlan via `roboplan` from PyPI.
The `--inexact` flag preserves other extras already installed in your current
environment.

Safety behavior for unsupported RoboPlan features:

- Planning-critical unsupported inputs fail loudly before planning. Examples
  include unsupported obstacle geometry, unavailable robot loading APIs, or
  unavailable collision query APIs. RoboPlan worlds generate a minimal SRDF from
  the DimOS robot config, including configured collision-exclusion pairs.
- Unverified non-critical query methods raise explicit `NotImplementedError`.
  In particular, signed minimum-distance semantics are not implemented for
  RoboPlan until a safe equivalent is verified.
- Embedded Meshcat visualization requires a world implementing `VisualizationSpec`;
  use Viser or `none` with the RoboPlan backend.

### Planning Visualization

Manipulation visualization is configured on `ManipulationModuleConfig.visualization`.
It is independent from the global Rerun stream viewer in `docs/usage/visualization.md`.

Backend choices:

- `meshcat`: embedded Drake/Meshcat visualizer. The planning world must be created with
  embedded visualization enabled, so this is selected through the visualization config.
- `viser`: in-process Viser visualizer. It renders pushed current robot state,
  target controls, transient preview ghosts, synchronized trajectory previews,
  and optional panel controls.
- `none`: no manipulation planning visualization.

CLI example:

```bash
uv run dimos run xarm7-planner-coordinator \
  -o manipulationmodule.visualization.backend=viser
```

Blueprint example:

```python skip
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig

manipulation = ManipulationModule.blueprint(
    config=ManipulationModuleConfig(
        robots=[...],
        visualization={
            "backend": "viser",
            "host": "127.0.0.1",
            "port": 8095,
            "open_browser": True,
            "panel_enabled": True,  # default; set False for scene-only Viser
        },
    )
)
```

Viser support is included in the `manipulation` extra:

```bash
uv sync --extra manipulation --inexact
```

### Grasp proposal providers and live visualization

`PickAndPlaceModule` requires exactly one module implementing `GraspGenSpec`.
Provider selection is explicit in the blueprint:

- `xarm-perception` and `xarm-perception-sim` use
  `HeuristicGraspModule`.
- `xarm-graspgenx` uses `GraspGenXModule` and the Viser manipulation
  visualization backend.

Both providers receive the same request: the segmented object point cloud plus
the selected detection's center and size. A provider error fails grasp
generation; the pick pipeline does not silently switch to another proposal
method.

When `PickAndPlaceModuleConfig.grasp_visualization` supplies the robot's gripper
and grasp-frame-to-TCP geometry, the pick pipeline automatically publishes two
Viser layers:

- `grasp/object-cloud` contains the point cloud sent to the proposal provider.
- `grasp/proposals` shows pending candidates in gray, the candidate currently
  undergoing connected motion planning in yellow, and rejected candidates in
  red. After a connected plan succeeds, only the selected candidate remains,
  in green.

These are visualization-only layers. They do not add collision geometry or
change planning outcomes, and visualization errors do not fail a pick.

The Viser panel talks to the concrete `ManipulationOperator` bound into its
`VisualizationSession`. GUI callbacks enqueue operations through that operator
for target evaluation, planning, preview, execution, cancellation, reset, and
clear-plan actions. The panel owns only target drafts, selection state, and
callback generations; it does not touch `WorldSpec`, IK, planner objects,
`ManipulationModule`, `WorldMonitor`, or live Drake contexts directly.

The panel's **Planning mode** selector chooses how the current target is
reached:

- **Joint space** is the default. It resolves the target to joints and invokes
  the configured collision-free joint-path planner.
- **Cartesian space** sends the existing transform-control poses as absolute
  world-frame TCP goals to RoboPlan's Cartesian path planner. Selected groups
  without a TCP participate as auxiliary groups.

Cartesian planning failure never falls back to joint-space planning. A backend
without Cartesian path support reports `UNSUPPORTED`; collision or tracking
failure leaves the plan unavailable. Preview and execution use RoboPlan's
original synchronized timestamps and velocities.

External manipulation visualizers are initialized from a backend-neutral
`VisualizationSession` after the planning world has added its robots. The
session contains static `PlanningSceneInfo` metadata: world robot IDs,
`RobotModelConfig` values, and resolved planning groups. Runtime joint state is
then pushed through `VisualizationStateFrame` updates so renderers do not poll
world/module state or own freshness policy. Embedded Meshcat visualization does
not need extra setup because it observes the Drake world directly.

Previews use the stored synchronized `JointTrajectory` from the generated plan.
Viser projects the globally named trajectory into robot-local preview ghosts and
plays the stored timestamped points directly; optional preview duration only
scales the stored delays. Execute freshness is enforced by the manipulation
module/operator immediately before dispatch, not by Viser-side telemetry
snapshots.

### Perception + Agent

```bash
# Coordinator + perception + manipulation + LLM agent (single command)
XARM7_IP=<ip> dimos run coordinator-xarm7 xarm-perception-agent
```

For a simulation walkthrough, see [Agentic xArm simulation](/docs/capabilities/manipulation/agentic.md).

## Architecture

```
KeyboardTeleopModule ──→ ControlCoordinator ──→ ManipulationModule
  (pygame UI)              (100Hz tick loop)      (WorldSpec backend)
       │                        │                       │
  TwistStamped           EEFTwistTask             RRT planner
  spatial EEF twist      (Pinocchio FK/IK)        JacobianIK
                               │                   DrakeWorld
                          JointState ────────────→ (visualization)
```

- **KeyboardTeleopModule** — Pygame UI publishing routed spatial EEF twist intent
- **ControlCoordinator** — 100Hz control loop with mock or real hardware adapters
- **ManipulationModule** — world backend, optional visualization, RRT motion planning, obstacle management

Internally, planning code depends on `WorldSpec` for world, collision, and
kinematics behavior. Meshcat preview and publishing are exposed separately
through `VisualizationSpec`, so non-visual planning paths do not require a
visualization backend.

All `WorldSpec` obstacle operations are runtime operations and require the
world to be finalized first. `update_obstacle(obstacle)` replaces the complete
obstacle identified by `obstacle.name`; callers must provide every geometry and
appearance field. `update_obstacle_pose(name, pose)` is the pose-only fast path
and preserves the other fields. Each update is serialized with native scene
queries, so collision checking sees either the old obstacle or the new one,
never the remove/add intermediate state. This boundary applies to each native
operation, not to an entire generic planning run; RoboPlan's opaque native
planner is locked for its whole native call.

## Blueprints

| Blueprint | Description |
|-----------|-------------|
| `keyboard-teleop-a750` | A750 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-piper` | Piper 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm6` | XArm6 6-DOF keyboard teleop with Drake viz |
| `keyboard-teleop-xarm7` | XArm7 7-DOF keyboard teleop with Drake viz |
| `xarm7-planner-coordinator` | XArm7 planner with coordinator integration |
| `dual-xarm6-planner-coordinator` | Dual XArm6 planning with mock coordinator hardware |
| `xarm-perception` | XArm7 + RealSense camera for perception |
| `xarm-perception-agent` | XArm7 perception + LLM agent |
| `xarm-perception-sim` | XArm7 simulation perception stack |
| [`xarm-perception-sim-agent`](/docs/capabilities/manipulation/agentic.md) | XArm7 simulation perception stack + LLM agent |

## Supported Robots

| Robot | DOF | Teleop | Planning | Perception |
|-------|-----|--------|----------|------------|
| [A-750](/docs/capabilities/manipulation/a750.md) | 6 | Y | Y | — |
| Piper | 6 | Y | Y | — |
| XArm6 | 6 | Y | Y | — |
| XArm7 | 7 | Y | Y | Y |

## Adding a Custom Arm

[guide is here](/docs/capabilities/manipulation/adding_a_custom_arm.md)

## Key Files

| File | Description |
|------|-------------|
| [`manipulation_module.py`](/dimos/manipulation/manipulation_module.py) | Main module (RPC interface, state machine) |
| [`robot/manipulators/common/blueprints.py`](/dimos/robot/manipulators/common/blueprints.py) | Shared coordinator, planner, and task helpers |
| [`robot/manipulators/a750/config.py`](/dimos/robot/manipulators/a750/config.py) | A-750 model and hardware config |
| [`robot/manipulators/a750/blueprints/teleop.py`](/dimos/robot/manipulators/a750/blueprints/teleop.py) | A-750 keyboard teleop blueprint |
| [`robot/manipulators/piper/blueprints/basic.py`](/dimos/robot/manipulators/piper/blueprints/basic.py) | Piper coordinator blueprint |
| [`robot/manipulators/piper/blueprints/teleop.py`](/dimos/robot/manipulators/piper/blueprints/teleop.py) | Piper teleop blueprints |
| [`robot/manipulators/xarm/blueprints/basic.py`](/dimos/robot/manipulators/xarm/blueprints/basic.py) | XArm coordinator and planner blueprints |
| [`robot/manipulators/xarm/blueprints/perception.py`](/dimos/robot/manipulators/xarm/blueprints/perception.py) | XArm perception blueprint |
| [`teleop/keyboard/keyboard_teleop_module.py`](/dimos/teleop/keyboard/keyboard_teleop_module.py) | Keyboard teleop module |
| [`planning/world/drake_world.py`](/dimos/manipulation/planning/world/drake_world.py) | Drake physics backend |
| [`planning/planners/rrt_planner.py`](/dimos/manipulation/planning/planners/rrt_planner.py) | RRT-Connect motion planner |
