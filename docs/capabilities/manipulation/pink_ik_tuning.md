---
title: "Pink IK Configuration and Tuning"
---

DimOS uses [Pink](https://github.com/stephane-caron/pink) for Cartesian,
EEF-twist, and engagement-relative teleoperation control. This guide explains
how to tune that shared backend after a robot model and control task are wired.
For the integration steps, see [Adding a Custom Arm](/docs/capabilities/manipulation/adding_a_custom_arm.md).

## Understand the two solve modes

Planning and streaming control use the same model and objective weights but run
them differently:

| Mode | Behavior |
| --- | --- |
| Planning IK | Iterates toward a target up to `max_iterations` |
| Streaming control | Takes exactly one QP step using the measured coordinator tick duration |

Streaming control does not run convergence retries or random restarts. It must
produce one small, bounded command from the current command trajectory on every
tick. `PinkKinematicsConfig.dt` is therefore a planning default; set the
coordinator rate correctly instead of using `dt` to tune teleoperation speed.

## Tune the common objective

Start with scalar configuration before adding robot-specific tasks:

```python skip
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig

pink = PinkKinematicsConfig(
    position_cost=1.0,
    orientation_cost=0.3,
    posture_cost=1e-3,
    joint_limit_posture_margin=0.15,
    lm_damping=1e-6,
    damping=1e-8,
    gain=0.3,
    safety_break=True,
)
```

| Field | Effect | Tuning direction |
| --- | --- | --- |
| `position_cost` | Translation tracking weight | Raise when position loses to other objectives |
| `orientation_cost` | Rotation tracking weight | Lower when orientation makes translation stiff or unreachable |
| `posture_cost` | Preference for the reference posture | Raise to reduce redundant motion; lower if it resists the target |
| `joint_limit_posture_margin` | Moves the posture reference inward near finite limits | Raise when redundant joints settle at their limits |
| `lm_damping` | Frame-task damping near singularities | Raise gradually when motion becomes unstable near singular poses |
| `damping` | Global QP velocity regularization | Raise to suppress large velocities; too much feels sluggish |
| `gain` | Fraction of task error corrected per solve | Raise for faster response; lower to reduce overshoot |
| `safety_break` | Rejects invalid Pink configurations | Keep enabled for hardware control |

Costs are relative. Multiplying all costs by the same factor rarely changes the
motion. First balance position and orientation, then add only enough posture
cost to shape redundant joints.

Pass the same scalar configuration to planning when both paths should start
with the same objective:

```python skip
from dimos.robot.manipulators.common.blueprints import planner

yourarm_planner = planner(robots=[robot_model], kinematics=pink)
```

## Customize the task stack

Subclass `PinkPoseTargetSolver` when scalar weights are insufficient—for
example, when a robot needs per-joint posture weights or a manipulability task.
Override `_create_tasks()`, call `super()`, and change only the required values:

```python skip
import numpy as np
import pink

from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver


class YourArmPinkPoseTargetSolver(PinkPoseTargetSolver):
    """Tune Pink's objective for YourArm."""

    def _create_tasks(
        self,
        configuration: pink.Configuration,
        target_frames: tuple[str, ...],
    ) -> dict[str, pink.Task]:
        tasks = super()._create_tasks(configuration, target_frames)

        posture = tasks.get("posture/current")
        if posture is None:
            raise ValueError("YourArm requires a positive posture cost")
        posture.cost = self.config.posture_cost * np.array(
            [4.0, 3.0, 0.2, 2.0, 1.0, 0.5]
        )

        for frame_name in target_frames:
            tasks[f"manipulability/{frame_name}"] = pink.tasks.ManipulabilityTask(
                frame_name,
                configuration.model,
                cost=0.005,
                manipulability_rate=0.05,
                mask="position",
            )
        return tasks
```

The common stack uses `frame/<target frame>` for frame tasks and
`posture/current` when `posture_cost > 0`. A generic bimanual subclass can add
shared tasks, and a robot-specific subclass can call `super()` and modify one
value. Each returned dictionary must contain new task instances because Pink
tasks are stateful and cannot be shared across control-task instances.

Pass the solver class, not an instance. The coordinator constructs one solver
for each control task:

```python skip
from dimos.robot.manipulators.common.blueprints import teleop_ik_task

teleop_task = teleop_ik_task(
    hardware,
    name="teleop_arm",
    robot_model=robot_model,
    bindings=[{"hand": "right", "target_frame": "link6"}],
    solver_type=YourArmPinkPoseTargetSolver,
    params={"pink": pink},
)
```

Use `_before_solve()` and `_after_solve()` only for a genuinely temporal Pink
task. Most tuning belongs in `_create_tasks()`.

## Bound streaming commands

Teleoperation task parameters bound the QP output against the URDF and live
hardware feedback. They are independent of the objective weights:

```python skip
teleop_task = teleop_ik_task(
    hardware,
    name="teleop_arm",
    robot_model=robot_model,
    bindings=[{"hand": "right", "target_frame": "link6"}],
    params={
        "pink": pink,
        "timeout": 0.5,
        "max_joint_velocity_rad_s": 1.0,
        "max_command_tracking_error_deg": 10.0,
        "feedback_limit_tolerance": 1e-3,
        "command_limit_margin": 1e-4,
    },
)
```

| Field | Purpose |
| --- | --- |
| `timeout` | Drops a stale controller target and resets the command trajectory |
| `max_joint_velocity_rad_s` | Per-task velocity cap; the lower of this and each URDF limit wins |
| `max_command_tracking_error_deg` | Maximum distance between the generated command trajectory and measured hardware state |
| `feedback_limit_tolerance` | Permits small sensor error beyond a URDF position limit |
| `command_limit_margin` | Keeps generated commands inside finite URDF position limits |

Start hardware tests with a conservative velocity cap. Raise it only after the
robot tracks smoothly. The tracking-error limit must tolerate normal execution
delay without allowing the command trajectory to run far ahead. Keep feedback
tolerance small: it accounts for encoder noise, not extra workspace.

An accepted command lies inside the configured and URDF velocity step, the
measured-state tracking window, and the inward position margin. Invalid
feedback or a failed Pink solve produces no new command for that tick.

## Tune in order

1. Verify joint names, order, base link, target frames, and startup forward
   kinematics.
2. Use fake hardware to confirm that small translation and rotation targets
   move in the expected directions.
3. Balance position and orientation with posture cost near zero.
4. Add posture weights or a manipulability task to shape redundant motion.
5. Exercise singular poses and joint limits; adjust damping and the inward
   posture margin.
6. Move to hardware with a conservative velocity cap. Tune tracking error for
   measured latency and sensor noise.
7. Test disengagement, target timeout, preemption, stop, and E-STOP. Each must
   clear the persistent command trajectory before re-engagement.

| Symptom | Check first |
| --- | --- |
| Position moves but rotation does not | `orientation_cost` and target-frame orientation |
| Arm barely moves | Excessive posture cost, low gain, or an unreachable target |
| Redundant joints drift or fold poorly | Per-joint posture weights or a manipulability task |
| Motion jitters near a singularity | `lm_damping`, QP `damping`, and gain |
| Commands stop near a joint limit | URDF limits, feedback tolerance, command margin, and inward posture target |
| Simulation works but hardware feels stuck | Command tracking error versus measured execution delay |
| Hardware jumps after lag | Velocity cap and command tracking error are too permissive |
| One side dominates a bimanual solve | Balance frame costs and shared-joint posture weights |

## OpenArm canonical-zero example

OpenArm starts in the canonical all-zero pose, where both joint-4 coordinates
are at their lower limits and the Cartesian Jacobian is rank-deficient. Its
solver uses a joint-limit posture margin to point only near-limit posture
coordinates inward while keeping the measured configuration as the streaming
seed. This creates a deterministic escape direction without a random restart
or a multi-iteration loop in the control tick.

Treat this as a model-specific response to a verified startup singularity, not
as a default reason to alter a robot's home pose. Validate the complete IK path
with fake hardware, then add a self-hosted test that loads the real model and
takes bounded steps from its canonical startup pose.
