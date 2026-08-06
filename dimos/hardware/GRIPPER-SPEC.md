# Spec — Gripper API Refactor

## 1. Principle

> **`write_joint_positions()` is the API for all joints. A gripper joint is just a robot
> joint of a special type, unified from the consumer's perspective.**

The API must serve **first-party grippers** (a capability of an arm's SDK) and
**third-party grippers** (a separate device with its own protocol) equally. A specific
third-party integration is a follow-up; this spec is the abstraction plus evidence it
works.

---

## 2. At a glance

A gripper comes in two shapes. **Everything above the hardware layer is identical between
them** — same task, same units, same joint representation, same `write_joint_positions()`.
Only the adapter and the connection beneath it differ, because that is where the two
devices genuinely differ.

```
  PROPOSED ARCHITECTURE

                  FIRST-PARTY (integrated)      |   THIRD-PARTY (standalone)
                  xArm . Piper . a1z . a750     |   its own protocol
 -----------------------------------------------------------------------------
  consumer        agent skill . teleop . grasp policy
                  speaks 0-1 or native units; no new ports          [R26 R27]
                            |
                            |  task_invoke("gripper", ...)
                            v
  control task    GripperControlTask - sole owner of gripper joints
                  command-driven . converts once . two modes    [R14 R16-R22]
                            |
                            v
  wire            joint_command / coordinator_joint_state
                  vectors, in each joint's declared unit            [R11 R12]
                            |
                            v
  arbitration     tick loop - priority, preemption, e-stop
                  unchanged; grippers now take part for the first time
                            |
                            v
  wrapper         ConnectedHardware - one ordered array, no conversion   [R25]
 ========================= the only divergence ==============================
  adapter         the arm's own adapter         |   its own GripperAdapter
                  joints = arm                  |   joints = gripper
                  gripper_joints = gripper      |   gripper_joints = empty
                  [R4 R6 R7 R8 R13]             |   [R4 R9 R10 R13 R24]
                            |                   |
                            v                   v
  hardware        one arm connection            |   its own connection
                  no sharing introduced   [R3]  |   [R1 R3]
```

> A third-party gripper mounted on an arm and wired through that arm's bus is still
> **standalone**: the arm supplies a *transport*, not the gripper's identity.

### 2.1 What changes

| | Added | Deleted |
|---|---|---|
| **Protocol** | `GripperAdapter`, `HardwareType.GRIPPER`, a gripper registry | `read/write_gripper_position` |
| **Control** | `GripperControlTask` — command-driven, sole owner | both gripper RPCs; `claim_with_gripper`, `append_gripper_position` |
| **Units** | adapter declares its range via `get_limits()`; task converts | `gripper_open/closed_position`; `_normalized_to_physical` |

---

## 3. Why this refactor (Current Status)

### 3.1 A shortcut route to hardware

Arm joints travel: task → arbitration → tick loop → adapter. Gripper joints skip all of it:

```python
# CURRENT — coordinator.py:901-916
@rpc
def set_gripper_position(self, hardware_id, position) -> bool:
    return hw.adapter.write_gripper_position(position)      # straight to hardware
```

No task, no arbitration, no priority, no preemption.

### 3.2 A single-joint ceiling

`write_gripper_position(position: float)` takes one scalar (`manipulators/spec.py:229`) and
`make_gripper_joints()` returns exactly one name (`components.py:106-115`). A multi-joint
gripper cannot be expressed.

### 3.3 A third-party gripper cannot be supported at all

Gripper control lives inside each arm's adapter, so a gripper is reachable only through the
arm that ships with it. A gripper from another vendor mounted on that arm has no path.

### 3.4 A gripper joint has no single owner

`teleop_task` and `eef_twist_task` both claim gripper joints via `claim_with_gripper`
(`cartesian_ik_task.py:61`) at differing priorities, and the RPC in §3.1 bypasses
arbitration entirely. Which input reaches the hardware depends on configuration and on
blueprint ordering.

### 3.5 An undefined unit contract, with a live bug

`a1z` and `piper` emit normalized `0.0–1.0`; `xarm` emits adapter-native `0.85`. All three
feed the same `ConnectedHardware._normalized_to_physical`, so the xArm value is mapped twice:

```
CURRENT BEHAVIOUR — measured against the real xarm7 teleop blueprint

task emits for FULLY OPEN      = 0.85
value reaching adapter         = 0.7225        (0.85 × 0.85)
xArm SDK set_gripper_position  = 722.5         (expected 850.0)
shortfall                      = 15.0% of travel lost
```

Reproduced against the real repository configuration. Root cause: conversion happens in
three places — blueprints, the hardware wrapper, and adapters — with no rule saying which is
authoritative.

---

## 4. Requirements

### 4.1 Two device shapes, one control path

**R1.** A gripper is modelled as **joints**, in one of two shapes:

| Shape | Example | Component | Adapter |
|---|---|---|---|
| **Integrated** | xArm, Piper, a1z, a750 — a capability of the arm's SDK | joints on the arm's `HardwareComponent` | the arm's adapter |
| **Standalone** | a third-party gripper with its own protocol | own `HardwareComponent`, `HardwareType.GRIPPER` | own `GripperAdapter` |

**R2.** The distinction is confined to the hardware layer. Above it both shapes are
identical: same task, same units, same joint representation, same `write_joint_positions()`.

**R3.** No connection-sharing mechanism is introduced. An integrated gripper is reached
through the arm's existing adapter because it *is* part of that device. A standalone
gripper owns its connection.

> A third-party gripper physically mounted on an arm and wired through that arm's bus is
> still **standalone** — the arm provides a *transport*, not the gripper's identity. The
> transport layering is a follow-up; this spec only requires that the gripper
> adapter be independent of the arm adapter, which R1 guarantees.

### 4.2 Adapter protocols

**R4.** `write_joint_positions()` and `read_joint_positions()` MUST cover **all** joints
the adapter owns, in `all_joints` order. Gripper entries occupy the trailing indices on an
arm adapter, and are the whole list on a gripper adapter.

```python
# PROPOSED

# integrated: arm adapter — arm joints in radians, gripper in the gripper's native unit
write_joint_positions([0.1, -0.4, 0.0, 0.9, 0.0, 1.2, 0.0,  0.042])
#                      └──── 7 arm joints, radians ──────┘  └ gripper, native (R12)

# standalone: gripper adapter — every joint is a gripper joint
write_joint_positions([0.3, 0.3, 0.5, 0.5, 0.2, 0.2])
```

> **Which field holds what.** `all_joints` is `joints + gripper_joints`
> (`components.py:100-103`) — disjoint lists, concatenated; a joint is in exactly one.
>
> | Component | `joints` | `gripper_joints` |
> |---|---|---|
> | Integrated (`MANIPULATOR`) | the arm's | the gripper's |
> | Standalone (`GRIPPER`) | the gripper's | *empty* — `hardware_type` already says so |
>
> Populating both on a standalone component duplicates every joint in `all_joints`.

**R5.** `read_gripper_position()` and `write_gripper_position()` are **removed** from
`ManipulatorAdapter` (`manipulators/spec.py:225-231`). They are a single-scalar API, which
R11 replaces with vectors.

**R6.** How an adapter fulfils `write_joint_positions()` internally is **not specified**.
An integrated gripper is, by definition, a capability of the arm's own SDK, so that
adapter necessarily reaches it through the vendor's own call — but that is an
implementation detail of one adapter, invisible above the hardware layer, and it places no
constraint on any other gripper.

> **This does not limit third-party support.** A gripper from another vendor never routes
> through an arm adapter; it takes the standalone path (R1) with its own `GripperAdapter`
> and its own protocol. Nothing about how the xArm adapter talks to its own gripper
> affects it.

<details><summary>Illustrative — one way an integrated adapter might do it</summary>

```python
# PROPOSED — illustrative only, not required by this spec
def write_joint_positions(self, positions, velocity=1.0):
    arm, grip = positions[: self._arm_dof], positions[self._arm_dof :]
    ok = self._arm.set_servo_angle_j([math.degrees(p) for p in arm], ...) == 0
    if grip:
        ok = self._arm.set_gripper_position(grip[0] * _XARM_GRIPPER_MAX_SDK, wait=False) == 0 and ok
    return ok
```

Non-normative. An adapter may satisfy R4 however its SDK requires.

</details>

**R7.** An arm adapter MUST learn its gripper joint count at construction
(`gripper_dof: int = 0`). It MUST NOT infer it from array length.

**R8.** `get_dof()` reports **arm joints only** — its meaning today is unchanged.
`get_gripper_dof()` reports gripper joints, returning `0` when there are none. A caller
wanting the total adds them. On a `GripperAdapter`, `get_dof()` reports its own joints.

**R9.** `GripperAdapter` is a **narrower** protocol than `ManipulatorAdapter` — the same
core method names, fewer of them:

```
PROPOSED — new protocol, does not exist today

MANDATORY
  connect() / disconnect() / is_connected()
  get_dof() -> int
  get_limits() -> JointLimits
  set_control_mode(mode) -> bool
  read_joint_positions() -> list[float]
  write_joint_positions(positions) -> bool
  write_stop() -> bool

OPTIONAL — defined fallback where unsupported
  read_joint_velocities() -> zeros
  read_joint_efforts()    -> zeros        (grasp force, when the hardware has it)
  activate() / deactivate()
  write_enable() / read_enabled()

ABSENT — arm-only, no meaning for a gripper
  read_cartesian_position, write_cartesian_position,
  read_force_torque, read_state
```

**R10.** A **joint** means an *actuated degree of freedom*, not necessarily a mechanical
hinge. A pneumatic or suction gripper is one joint with limits `[0, 1]`; a
pressure-controlled gripper is one joint whose native unit is kPa. A soft continuum
gripper with no discrete actuation state is **out of scope** — it would need its own
message type, not a looser version of this one.

### 4.3 Units — joint-native on the wire, task converts

**R11.** All gripper command and read APIs are **vectors**, never scalars.

**R12.** A gripper joint carries **the unit its adapter actually speaks** on
`joint_command` and `coordinator_joint_state` — metres for a sliding jaw, radians for a
rotating knuckle, or the vendor's own scale where the SDK is dimensionless. Arm joints are
unchanged (radians).

**R13.** The **adapter declares its range** and exposes it via `get_limits()`. A private
module constant remains the implementation; `get_limits()` makes it readable. Two adapters
already have the constant — `piper/adapter.py:44-45` (`0.08` m),
`galaxea_a1z/config.py:45-49` (`0.1` m).

> **The declared unit is what the adapter actually speaks.** Piper (`0.08`) and a1z (`0.1`)
> are genuinely metres. xArm is not: `MM_TO_M`/`M_TO_MM` belong to cartesian pose
> (`:347-349, 366-368`) and were reused on the gripper path (`:386, :397`), where the SDK
> takes a dimensionless `0–850`. Today's `0.85` is just `850/1000`. xArm therefore declares
> `(0.0, 850.0)` and drops the mm conversion.

**R14.** `GripperControlTask` reads `get_limits()` at construction and exposes **both**
interfaces:

```python
# PROPOSED — GripperControlTask
set_position(values: list[float])     # native units, per joint
set_normalized(values: list[float])   # 0.0–1.0 per joint → lo + (hi-lo)*v
```

Nothing above the task ever needs a vendor range; anything needing real units can have them.

**R15.** `HardwareComponent.gripper_open_position` / `gripper_closed_position` are removed
(`components.py:97-98`), together with `_normalized_to_physical` /
`_physical_to_normalized` (`hardware_interface.py:232-244`). Conversion happens in exactly
one place: the task.

> Normalization is `0.0` = fully closed, `1.0` = fully open. This is not a new convention —
> it is what `hardware_interface.py:237` already implements, what
> `galaxea_a1z/adapter.py:479` states in its docstring, and what
> `keyboard_teleop_module.py:64-65` already uses.

### 4.4 `GripperControlTask`

**R16.** A new task type `gripper`, **command-driven** rather than stream-driven:

```python
# PROPOSED — new task type, does not exist today
TASK_CONSUMES = {"gripper": {}}                       # no input streams
TASK_EXPOSES  = {"gripper": ["set_position", "set_normalized", "set_sweep",
                             "set_reference_pose", "get_position", "get_state"]}
```

This mirrors `trajectory_task` (`trajectory_task/_registry.py:19-25`). It is invoked via
the existing `task_invoke` RPC (`coordinator.py:803`) — an RPC **into a task**, which then
flows through arbitration and the tick loop.

> **The task owns the gripper completely** — it commands it, and it reports it.
> `get_position` returns the **measured** position, taken from the `CoordinatorState`
> snapshot every task receives; `get_state` reports what the task is doing, mirroring
> `trajectory_task.get_state()`.
>
> This cannot disagree with `coordinator_joint_state`: `tick_loop.py:180-195` passes the
> *same* `joint_states` object to `compute()` and to the publisher. The stream and this RPC
> are one snapshot, offered as a subscription and as a one-shot read.

**R17.** It is the **sole claimant of gripper joints**. `teleop_task` and `eef_twist_task`
drop `claim_with_gripper` and `append_gripper_position`; the helpers at
`cartesian_ik_task.py:61-72` are deleted. Both tasks instead forward gripper intent into
`GripperControlTask`. One joint, one owner.

**R17a.** Their gripper **configuration** goes with it. `gripper_joint`,
`gripper_open_pos`, and `gripper_closed_pos` are removed from both task configs
(`teleop_task.py:72-74, 268-270`; `eef_twist_task.py:49-51, 193-195`), and the blueprint
constants that feed them are deleted — `XARM_GRIPPER_PARAMS` (`xarm/config.py:61-65`) plus
the inline `params={...}` gripper dicts in the `a1z` and `piper` teleop blueprints.

> Endpoint values disappear from the task layer entirely. Under R13/R14 the range comes
> from `get_limits()` and normalization happens inside `GripperControlTask`, so no
> blueprint declares an open/closed endpoint any more. This is what removes the duplicated
> endpoints that caused §3.5 — there is no longer a second place to disagree.

**R18.** Two control modes, per review:

| Mode | API | Use |
|---|---|---|
| **Fine-grained** | `set_position([...])` / `set_normalized([...])` | per-joint control; grasp policies |
| **Scalar** | `set_sweep(value: float)` | one number: `0.0` = fully at the **reference pose**, `1.0` = fully open |

**R19.** `set_sweep(v)` interpolates between an **open posture** and a **reference
posture**, both joint vectors. **`v` has the same meaning as in `set_normalized`: `0.0` is
fully closed, `1.0` is fully open.**

```
PROPOSED

sweep(v) = reference_pose + (open_posture - reference_pose) * v
           v = 0.0 → fully at the reference pose (the grasp, applied)
           v = 1.0 → fully open
```

**R19a.** The **reference pose is the vendor's grasp pose**, not a derivation from joint
limits. Joint limits describe travel, not grasping — driving every joint of a multi-finger
hand to its extreme is a fist or a self-collision, not any grasp the vendor ships.

| Gripper | `open_posture` | `reference_pose` |
|---|---|---|
| Single joint | its open limit | its closed limit — for a jaw, closed *is* the grasp |
| Multi-joint | its joints' open limits | **MUST be declared in config.** No default exists |

**R19b.** `set_sweep` MUST refuse, with a clear error, on a multi-joint gripper that has no
reference pose configured. It MUST NOT fall back to interpolating against joint limits.

> A vendor's documented grasp poses become named vectors in blueprint config — the H100's
> four gestures (two-finger, three-finger, and two pinches) are four entries, any of which
> may be selected as the reference. `set_reference_pose()` replaces it at runtime, so a
> grasp planner can supply a computed pose and `set_sweep` then scales *that* grasp.

> **Polarity is identical across every scalar API.** `set_normalized([0.0, ...])`,
> `set_sweep(0.0)`, and `close_gripper()` all close; `1.0` opens in each case. A mismatch
> here produces no error — just a gripper that opens when told to close.

The protocol itself knows only vectors — postures live in config, never in `GripperAdapter`.

**R20.** **Every** command in `TASK_EXPOSES` MUST return immediately — `set_position`,
`set_normalized`, `set_sweep`, `set_reference_pose`, `get_position`, `get_state`. The
setters record the target and reactivate the task; the readers return the latest snapshot.
None may block, wait, or sleep.

> **Hard constraint.** `task_invoke` holds `_task_lock` (`coordinator.py:813`) and the tick
> loop needs that same lock every tick (`tick_loop.py:268`). Blocking inside a task command
> would stall the **entire coordinator** — no arbitration, no hardware writes for any
> joint — not merely the gripper.

**R21.** The task emits its target for a bounded hold duration, then deactivates. It MUST
NOT deactivate on "measured position reached target": a gripper stalled on a grasped object
never reaches its commanded position, so that condition would misreport every successful
grasp. On deactivation `ConnectedHardware` holds the last commanded value, so closing force
is maintained.

**R22.** It uses `ControlMode.SERVO_POSITION`, matching `servo_task` and
`trajectory_task` so that an integrated gripper never disagrees on mode with the arm tasks
sharing its component (`tick_loop.py:386-397`). On a standalone component it is the only
task, so the question does not arise.
### 4.5 Coordinator and `ConnectedHardware`

**R23.** `@rpc set_gripper_position` and `get_gripper_position` are **removed**
(`coordinator.py:901-931`). The capability moves to
`task_invoke("gripper", ...)` — `set_normalized` for the agent skills (R26), which is
the scale they already think in. Only the shortcut route dies.

**R24.** `HardwareType.GRIPPER` is added, with a gripper adapter registry following the
existing `LazyAdapterRegistry` pattern used by `drive_trains`, `manipulators`, and
`whole_body`.

**R25.** `ConnectedHardware` builds one ordered array from `all_joints` and makes one call.
Its gripper branch is deleted (`hardware_interface.py:125-137, 192-203, 213-221`). It
performs **no** unit conversion, and wraps either adapter kind unchanged.

### 4.6 `ManipulationModule`

**R26.** **No new ports.** `_set_gripper_position()` calls
`task_invoke("gripper", "set_normalized", {...})`; `get_gripper()` reads the gripper entry
from `coordinator_joint_state`, which the module already subscribes to (`:171`, `:326`).
Its docstring MUST be corrected — it claims metres while returning `0.85` today.

**R27.** The three `@skill` methods keep their synchronous `bool`, reporting **command
acceptance** — identical to today, where the `bool` means "the SDK accepted it", not "the
gripper arrived". `pick_and_place_module`'s three `time.sleep()` calls are **retained**;
replacing them requires grasp detection, which is out of scope here.

### 4.7 Configuration

Two changes to how a blueprint declares a gripper — and one thing to deliberately leave
alone.

**R28. Nothing changes here.** `HardwareComponent.joints`, `gripper_joints`, and
`all_joints` keep their current fields and meanings. This requirement exists only to
*prevent* a refactor that looks obvious and is wrong.

```python
# CURRENT — how a1z/xarm/piper declare a gripper today. PROPOSED: identical.
HardwareComponent(
    joints         = make_joints("arm", 6),   # ["arm/joint1" … "arm/joint6"]
    gripper_joints = ["arm/gripper"],
)
# all_joints == joints + gripper_joints == all 7
```

**The tempting change, and why it must not happen.** Given that this spec unifies gripper
joints with arm joints everywhere else, folding them into a single `joints` list looks like
the natural cleanup:

```python
# REJECTED — do NOT do this
HardwareComponent(
    joints = [*make_joints("arm", 6), "arm/gripper"],   # one list
)
```

The split is what keeps them safe: **`joints` / `gripper_joints` are what a human writes;
`all_joints` is what the runtime reads.**

**R28a. The two fields carry any joint count**, including a 6-joint dexterous hand. Worked
examples, all on an **xArm6** so the arm DOF stays constant:

```python
# CURRENT — xArm6 with its first-party gripper (a1z, xarm, piper all look like this)
HardwareComponent(
    hardware_id="arm", hardware_type=MANIPULATOR,
    joints         = make_joints("arm", 6),        # 6 arm joints
    gripper_joints = ["arm/gripper"],              # 1 gripper joint, on the arm's SDK
)                                                  # all_joints -> 7

# PROPOSED — the same xArm6, first-party gripper REPLACED by a third-party 6-joint hand.
#            Two components, two adapters. This is the motivating case.
HardwareComponent(
    hardware_id="arm", hardware_type=MANIPULATOR,
    joints         = make_joints("arm", 6),
    gripper_joints = [],                           # the arm's own gripper is gone
)                                                  # all_joints -> 6
HardwareComponent(
    hardware_id="hand", hardware_type=GRIPPER,     # <- new type, R24
    joints         = make_joints("hand", 6),       # all 6 ARE the gripper
    gripper_joints = [],                           # empty; hardware_type says it
    adapter_type   = "<vendor>",                   # its own GripperAdapter, R9
)                                                  # all_joints -> 6
```

The third-party hand is **standalone even when bolted to the arm and wired through its
bus** — the arm supplies a transport, not the gripper's identity (R3). The arm adapter
never learns the hand exists.

A first-party gripper with several joints would also work — `gripper_joints` would simply
hold more than one name, which is what R29 enables — but no such device exists in-tree
today, so the two shapes above are the ones that matter.

**R29.** `make_gripper_joints()` takes a joint count. It is hardcoded to exactly one name
today (`components.py:106-115`), which is the single-joint ceiling in §3.2:

```python
CURRENT   make_gripper_joints("arm")      → ["arm/gripper"]   # hardcoded to one
PROPOSED  make_gripper_joints("hand", 6)  → ["hand/gripper1" … "hand/gripper6"]
```

**R30.** A blueprint exposing gripper control adds a `gripper` task. Which field it claims
depends on the shape (R4):

```python
# PROPOSED

# integrated — the gripper rides on the arm's component
TaskConfig(name="gripper", type="gripper", joint_names=_arm_hw.gripper_joints)

# standalone — every joint on the component is a gripper joint
TaskConfig(name="gripper", type="gripper", joint_names=_hand_hw.joints)
```

Two things worth knowing about the blueprints as they stand:

- **`keyboard_teleop_a1z` already ships this pattern**, hand-rolled as a servo task
  (`a1z/blueprints/teleop.py:46-51`) — a dedicated task claiming only `["arm/gripper"]`.
  R30 replaces it with the `gripper` task type.
- **`coordinator_teleop_dual` has no gripper path at all** (`common/mixed.py:75-88`):
  `gripper=True` on both arms, but neither `teleop_ik_task` is given gripper config and no
  gripper task exists. Its grippers are unreachable today — a pre-existing defect this
  refactor surfaces, worth filing separately.

## 5. Untouched

`servo_task`, `trajectory_task`, `ManipulationModule`'s ports, every `pick_and_place` call
site, and all 13 `joint_names=hw.joints` claim sites.

`teleop_task` and `eef_twist_task` **are** modified (R17) — they lose their gripper claims
and forward intent instead.

## 6. Blast radius

| Area | Files | Nature |
|---|---|---|
| **New protocol** | `hardware/grippers/spec.py`, `registry.py` | `GripperAdapter` + registry |
| **New task** | `control/tasks/gripper_task/` | `GripperControlTask` + `_registry.py` |
| **New mock** | `hardware/grippers/mock/` | multi-joint gripper, proves the standalone path |
| Adapter protocol | `manipulators/spec.py` | Delete 2 methods; add `get_gripper_dof()` |
| Adapters | `xarm`, `piper`, `mock`, `sim`, `openarm`, `galaxea_a1z`, `a750` | Unified array, `gripper_dof`, range via `get_limits()` |
| Hardware wrapper | `hardware_interface.py` | One array; delete gripper branch + normalization |
| Components | `components.py` | `HardwareType.GRIPPER`; delete 2 fields; widen `make_gripper_joints` |
| Tasks | `teleop_task`, `eef_twist_task`, `cartesian_ik_task` | Drop gripper claims + gripper config fields; delete 2 helpers (R17, R17a) |
| Coordinator | `coordinator.py` | Delete 2 RPCs; gripper adapter branch |
| Manipulation | `manipulation_module.py` | 2 method bodies + 1 docstring |
| Blueprints | `xarm`, `a1z`, `piper` teleop; gripper-bearing blueprints | Add the gripper task; delete `XARM_GRIPPER_PARAMS` and inline gripper `params` (R17a) |
| Message docs | `msgs/sensor_msgs/JointState.py` | Note gripper joint units |

## 7. Delivery plan

**This PR is the spec — documentation only, no code.** What it defines then ships in two
implementation PRs, followed by the vendor work that motivated all of it.

| Step | Delivers | Requirements | Roughly |
|---|---|---|---|
| **0 — this PR** | The spec. Agreement on the API before anything is written | — | docs only |
| **1 — adapter API** | `write_joint_positions()` covers every joint; gripper wrappers deleted; `ConnectedHardware` builds one array; adapters declare their range | R4–R13, R25, R28–R29 | ~12 files, mechanical |
| **2 — control path** | `GripperControlTask`, `GripperAdapter`, `HardwareType.GRIPPER`, sole ownership, RPC removal, teleop rerouting, mock multi-joint gripper | R1–R3, R14–R24, R26–R27, R30 | ~18 files, behavioural |
| **3 — vendor integration** | A real third-party gripper: adapter, driver, protocol | — | against a settled API |
| **4 — transport layering** | Routing that gripper's bytes through an arm's bus, for a gripper mounted on an arm | — | R3 already permits it |

