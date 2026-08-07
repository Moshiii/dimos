---
title: "OpenArm Integration"
---

DimOS drives the [OpenArm](https://openarm.dev) bimanual platform (two 7-DOF
arms + grippers, Damiao motors, one CAN bus per arm) as a single whole-body
device through the generic Damiao adapter stack introduced for OpenYAM.

Related:
- Upstream hardware + C++ reference: [enactic/openarm_can](https://github.com/enactic/openarm_can)
- How to integrate any new arm: [adding_a_custom_arm.md](/docs/capabilities/manipulation/adding_a_custom_arm.md)

## Architecture

```
ControlCoordinator (100 Hz)
  └── HardwareComponent "openarm" (WHOLE_BODY, 16 joints)
        └── OpenArmDamiaoAdapter          # dimos/hardware/whole_body/openarm_damiao/
              └── DamiaoWholeBodyAdapter  # generic Damiao lifecycle + gravity comp
                    └── can-motor-control # Rust CAN transport + Damiao codec (PyPI)
```

One adapter owns both arms: bus `left` (default `can1`) and bus `right`
(default `can0`) are commanded together in one synchronized tick per control
cycle. The command vector order is `left_arm/joint1..7`, `right_arm/joint1..7`,
`left_arm/gripper`, `right_arm/gripper`; gripper joints are normalized
(`0.0` closed, `1.0` open).

Per arm, shoulder to wrist (send ids `0x01..0x07`, feedback `send | 0x10`):
2x DM8009, 2x DM4340, 3x DM4310, plus a DM4310 gripper at `0x08`.

Gravity compensation uses the bimanual URDF
(`openarm_description/urdf/robot/openarm_v20_bimanual.urdf`, resolved lazily
from LFS at connect time) and is preflighted against the declared joint order
before the motors enable.

Planning also uses the bimanual URDF: one robot model with a
`left_manipulator` and a `right_manipulator` planning group, since collision
exclusions cannot span robots.

## Bring-up

```bash
dimos hardware can setup can0
dimos hardware can setup can1
dimos run keyboard-teleop-openarm
```

Linux assigns `can0`/`can1` in USB enumeration order. If the arms come up
swapped, override the mapping through
`DamiaoRuntimeConfig(bus_addresses={"left": ..., "right": ...})` rather than
editing the adapter topology.

## Blueprints

| Blueprint | Contents |
|---|---|
| `coordinator-openarm` | coordinator + trajectory task over both arms |
| `openarm-planner-coordinator` | planner (bimanual model) + coordinator |
| `keyboard-teleop-openarm` | keyboard + per-arm EEF twist + viser |
| `keyboard-teleop-openarm-planner` | teleop + planner + preempting trajectory task |
| `teleop-quest-openarm` | Quest + bimanual Pink + planner/Viser + arm trajectory task + unconditional in-memory hardware |

All blueprints run against the in-memory whole-body adapter under
`--simulation`; the physical adapter is selected automatically otherwise.
The exception is `teleop-quest-openarm`, which always uses `mock_whole_body`
even without `--simulation`. This safety-sensitive Quest blueprint never
selects physical hardware implicitly. Its mock adapter and model both use the
canonical all-zero OpenArm start.

Run the bimanual Quest stack with:

```bash
dimos run teleop-quest-openarm
```

The left and right controller pose streams carry absolute controller poses to
distinct coordinator inputs but name the same `teleop_openarm` task. That task
binds the controllers to `openarm_left_grasp_frame` and
`openarm_right_grasp_frame` in the existing bimanual model, captures both
references atomically, and performs one Pink step for both arms. Both primary
buttons must remain held; releasing either one stops all arm and gripper output
until both hands re-engage.

The canonical zero state places both joint-4 coordinates at their lower limits
and makes the Cartesian Jacobian rank-deficient. The OpenArm control task and
planner therefore share an opt-in Pink joint-limit posture margin of `0.3` rad.
The measured seed remains zero, while Pink's low-cost posture target points only
near-limit coordinates inward. This does not add a random restart or convergence
loop to the control tick, and all outputs retain the normal 5-degree per-tick
joint-delta check.

When Viser targets only one arm, Pink holds every joint in the unselected arm at
its measured seed. These locks are committed through Pink's configuration update
API because Pink exposes its integrated joint vector as read-only.

The same blueprint wires `ManipulationModule` to the bimanual model with Viser
visualization and gives the coordinator a `traj_arm` joint-trajectory task over
all fourteen arm joints. The trajectory task runs at priority 20 versus Quest
teleoperation at priority 10, so planner execution preempts and resets an
engaged Quest session through the existing coordinator arbitration path. The
grippers remain Quest-controlled because they are outside the planning model.

The keyboard jogs the left arm (`eef_twist_left_arm`); the right arm's twist
task holds its anchor pose. Keyboard gripper bindings for the two grippers are
a follow-up; the gripper joints accept normalized `/joint_command` targets in
the meantime.

## Files

| Path | Role |
|---|---|
| `dimos/hardware/whole_body/openarm_damiao/adapter.py` | physical topology (motors, buses, gravity URDF) |
| `dimos/robot/manipulators/openarm/config.py` | joints, gains, hardware + planning model configs |
| `dimos/robot/manipulators/openarm/blueprints/` | coordinator/planner/teleop blueprints |

## Validation

```bash
uv run pytest dimos/hardware/whole_body/openarm_damiao \
    dimos/hardware/test_adapter_registries.py
```
