# Spec — Unified Gripper Joint API


## 1. Principle

> **`write_joint_positions()` is the API for all joints. A gripper joint is just a robot
> joint of a special type, unified from the consumer's perspective.**

Every requirement below follows from that sentence.

## 2. Why

### 2.1 A shortcut route to hardware

Arm joints travel: task → arbitration → tick loop → adapter. Gripper joints skip all of it:

```python
# coordinator.py:901-916
@rpc
def set_gripper_position(self, hardware_id, position) -> bool:
    return hw.adapter.write_gripper_position(position)      # straight to hardware
```

No task, no arbitration, no priority, no preemption.

### 2.2 A single-joint ceiling

`write_gripper_position(position: float)` takes one scalar (`manipulators/spec.py:229`) and
`make_gripper_joints()` returns exactly one name (`components.py:106-115`). A multi-joint
gripper cannot be expressed.

### 2.3 An undefined unit contract, with a live bug

Nothing specifies what unit a gripper value carries, so blueprints disagree — `a1z` and
`piper` emit normalized `0.0–1.0`, `xarm` emits adapter-native `0.85`, and all three feed
the same `ConnectedHardware._normalized_to_physical`. The xArm value is mapped twice:

```
task emits for FULLY OPEN      = 0.85
value reaching adapter         = 0.7225        (0.85 × 0.85)
xArm SDK set_gripper_position  = 722.5         (expected 850.0)
shortfall                      = 15.0% of travel lost
```

Reproduced against the real repository configuration. Root cause: conversion happens in
three places — blueprints, the hardware wrapper, and adapters — with no rule saying which
is authoritative.

---

## 3. Requirements

### 3.1 Adapter — one implementation for all joints

**R1.** `write_joint_positions()` and `read_joint_positions()` MUST cover **all** joints
the component owns, in `HardwareComponent.all_joints` order. Gripper entries occupy the
trailing indices.

```python
adapter.write_joint_positions([0.1, -0.4, 0.0, 0.9, 0.0, 1.2, 0.0,  1.0])
#                              └──────── 7 arm joints, radians ───┘  └ gripper
```

**R2.** `read_gripper_position()` / `write_gripper_position()` are **retained** on
`ManipulatorAdapter` as convenience wrappers, but MUST delegate to the unified methods —
they are no longer an independent implementation:

```python
def write_gripper_position(self, position: float) -> bool:
    """position is normalized 0.0–1.0, per R6 — NOT adapter-native units."""
    positions = self.read_joint_positions()
    positions[self._arm_dof :] = [position] * self._gripper_dof
    return self.write_joint_positions(positions)
```

> **Their units change.** They previously took and returned adapter-native values
> (`0.85` on xArm, metres on Piper). Delegating to `write_joint_positions()` means they now
> speak normalized `0.0–1.0` like every other gripper value. The signature is unchanged, so
> this is a silent semantic change — one more reason nothing should call them without
> reading R6.
>
> After this PR nothing in-tree does: `ConnectedHardware` and the deleted RPC were the only
> callers. They remain as a documented convenience for adapter authors.

**R3.** An adapter needing a distinct SDK call for gripper indices MUST split inside
`write_joint_positions()`:

```python
def write_joint_positions(self, positions, velocity=1.0):
    arm, grip = positions[: self._arm_dof], positions[self._arm_dof :]
    ok = self._arm.set_servo_angle_j([math.degrees(p) for p in arm], ...) == 0
    if grip:
        ok = self._arm.set_gripper_position(grip[0] * _XARM_GRIPPER_MAX_SDK, wait=False) == 0 and ok
    return ok
```

**R4.** The adapter MUST learn its gripper joint count at construction (`gripper_dof: int = 0`).
It MUST NOT infer it from array length.

**R5.** `get_dof()` MUST return the total (arm + gripper). Adapters branching on arm DOF
internally — e.g. xArm's 6- vs 7-DOF initial pose (`xarm/adapter.py:278-280`) — keep a
private `_arm_dof`.

### 3.2 Units — the adapter owns conversion

**R6.** A gripper joint value on `joint_command` and `coordinator_joint_state` is
normalized: `0.0` fully closed, `1.0` fully open. Out-of-range values are clamped. Arm
joints are unchanged (radians).

*Why not a physical unit.* Gripper mechanisms differ in kind — sliding jaws, rotating
knuckles, mechanically coupled linkages — so no single physical unit covers them, and
firmware-locked mechanisms have none at all. A travel fraction is valid across every
mechanism and portable: `0.6` means the same thing on every device.

**R7.** The **adapter** performs the conversion, in **both** directions. No other layer
converts. This collapses §2.3's three conversion sites into one.

*Why the adapter.* It is already the unit boundary for every joint —
`xarm/adapter.py:222-227` converts wire radians into vendor degrees. R7 applies that
existing rule rather than inventing a second site. Any higher placement would force the
converting layer to branch on *which joints are grippers*, re-introducing the exact
special-casing this PR removes. And the range constants already live in the adapters.

**R8.** Each adapter MUST declare its own gripper travel range as a module constant. Two
already do — `piper/adapter.py:44-45` (`GRIPPER_MAX_OPENING_M = 0.08`) and
`galaxea_a1z/config.py:45-49` (`max_opening_m`).

**R9.** `HardwareComponent.gripper_open_position` / `gripper_closed_position` are removed
(`components.py:97-98`), together with `_normalized_to_physical` /
`_physical_to_normalized` (`hardware_interface.py:232-244`).

**R10.** Task-level `gripper_open_pos` / `gripper_closed_pos` are now interpreted as
normalized. **No task code changes** — the fields and their `0.0` defaults stay as they
are (`teleop_task.py:73-74, 269-270`; `eef_twist_task.py:50-51, 194-195`), and every
gripper-bearing blueprint already passes explicit values, so the defaults are never
relied on.

Only one blueprint value changes: `xarm/config.py:63` (`0.85` → `1.0`). `a1z` and `piper`
already pass `1.0` / `0.0`.

**R11.** `get_limits()` entries for gripper joints MUST report `0.0` / `1.0`.

**R12.** `JointState.position` is documented as *"(rad or m)"*
(`msgs/sensor_msgs/JointState.py:59`). Gripper entries are a dimensionless fraction,
outside that contract. **This MUST be documented on the message type and in the
glossary.** Replacing an undocumented convention with a documented one is the point.

### 3.3 `ConnectedHardware`

**R13.** Builds one ordered array from `all_joints` and makes one call. Its gripper branch
is deleted (`hardware_interface.py:125-137, 192-203, 213-221`). It performs no unit
conversion.

### 3.4 `GripperTask` — new, command-driven

**R14.** A new task type `gripper`, driven by commands rather than streams:

```python
TASK_CONSUMES = {"gripper": {}}                                  # no input streams
TASK_EXPOSES  = {"gripper": ["set_position", "get_position"]}
```

This mirrors `trajectory_task` (`trajectory_task/_registry.py:19-25`), which is also
command-driven. It is invoked through the existing `task_invoke` RPC
(`coordinator.py:803`) — an RPC **into a task**, which then flows through arbitration and
the tick loop. That is not a shortcut route; §2.1 is.

**R15.** It MUST claim **only** the gripper joints — never arm joints. This makes it
structurally incapable of moving the arm and lets a gripper command stand alone.

**R16.** It MUST use `ControlMode.SERVO_POSITION`, matching `servo_task` and
`trajectory_task`. A different mode on the same component would trigger the mode-conflict
path in `_route_to_hardware` (`tick_loop.py:386-397`).

**R17.** Priority defaults to **10**, matching `trajectory_task`
(`common/blueprints.py:66`).

> **Known item.** `eef_twist` is also priority 10 and also claims the gripper via
> `claim_with_gripper`. If both are active the tie is resolved by blueprint `tasks=[...]`
> order, not by policy. Accepted for now; see §9.1.

**R18.** `set_position(position: float)` MUST return immediately. It sets the target and
reactivates the task; it MUST NOT block.

> **Hard constraint.** `task_invoke` holds `_task_lock` (`coordinator.py:813`) and the
> tick loop needs that same lock every tick (`tick_loop.py:268`). Blocking inside a task
> command would stall the **entire coordinator** — no arbitration, no hardware writes for
> any joint — not merely the gripper.

**R19.** The task emits its target for a fixed hold duration after each `set_position`,
then deactivates. It MUST NOT deactivate on "measured position reached target" — that is
the same assumption R26 rejects, and it fails identically: a gripper stalled on a grasped
object never reaches its commanded position.

On deactivation `ConnectedHardware` holds the last commanded value, so closing force is
maintained on a grasped object and the joint is released for other tasks. The next
`set_position` reactivates the task.

> One constant — the hold duration. It bounds how long the task claims the joint; it is
> not a completion check and makes no claim about whether the gripper arrived.

**R20.** Blueprints with a gripper add the task, claiming `hw.gripper_joints`.

### 3.5 Coordinator

**R21.** `@rpc set_gripper_position` and `get_gripper_position` are **removed**
(`coordinator.py:901-931`). The *capability* remains — it moves to
`task_invoke("gripper", "set_position", ...)`. Only the shortcut route dies.

**R22.** No new RPC is added; `task_invoke` already exists.

### 3.6 `ManipulationModule`

**R23.** **No new ports.** It keeps `coordinator_joint_state: In[JointState]` and its
existing coordinator handle.

**R24.** `_set_gripper_position()` calls `task_invoke("gripper", "set_position", {...})`.

**R25.** `get_gripper()` reads the gripper entry from `coordinator_joint_state`, which the
module already subscribes to (`:171`, handler `:326`). This is better than the deleted RPC:
measured position, no round trip.

> Its docstring MUST be updated. It currently claims metres while returning `0.85` for
> xArm (`manipulation_module.py:1652-1663`); under R6 it returns `0.0–1.0`.

**R26.** The three `@skill` methods keep their synchronous `bool`, reporting **command
acceptance** — identical to today's semantics, where the `bool` means "the SDK accepted
it", not "the gripper arrived."

> **Why not a completion check.** An earlier draft had the skill poll
> `coordinator_joint_state` until the gripper reached target. That is **wrong for
> grippers**: closing onto an object stalls the gripper short of the commanded position,
> so a *successful grasp* would time out and report `GRIPPER_FAILED`.
>
> The correct condition is "motion settled", not "target reached" — and settling short of
> target is itself the signal that something is held. But xArm and Piper both return
> all-zero velocities (`xarm/adapter.py:173`, `piper/adapter.py:350`), so settle detection
> needs position-delta sampling plus per-gripper effort thresholds. That is grasp
> detection, a design problem in its own right, and out of scope here.

**R27.** `pick_and_place_module`'s three `time.sleep(0.5/1.5/1.0)` calls
(`:518, :536, :627`) are **retained**. They remain the only completion signal, exactly as
today. Replacing them requires the grasp detection R26 defers.

### 3.7 Control modes

**R28.** An adapter MUST refuse a mode it cannot honour by returning `False` from
`set_control_mode(mode)`. Existing mechanism, already handled upstream
(`hardware_interface.py:173-177`) and already used by `galaxea_a1z/adapter.py:260-262`
and `openarm/adapter.py:238-246`. No new capability API.

**R29.** Gripper joints here are position-only because every gripper SDK in play is
position-only — a property of the devices, not a scoping decision.

### 3.8 Configuration

**R30.** `HardwareComponent.joints` and `gripper_joints` are retained as **authoring**
fields; `all_joints` (`components.py:100-103`) is what the runtime consumes. The 13
existing `joint_names=hw.joints` claim sites keep working unchanged.

**R31.** `make_gripper_joints(hardware_id, n=1)` MUST accept a joint count
(`components.py:106-115`).

---

## 4. Resulting flow

```
agent skill ──► task_invoke("gripper", "set_position", 1.0)
                     │
                     ▼
              GripperTask         claims gripper joints only, SERVO_POSITION
                     │
                     ▼
              arbitration ──► tick loop ──► ConnectedHardware
                                                  │  one array from all_joints
                                                  ▼
                                          adapter.write_joint_positions([...arm, 1.0])
                                                  │  0.0–1.0 → vendor units
                                                  ▼
                                              vendor SDK
```

Only the adapter knows what a specific gripper's travel is.

## 5. Untouched

**Task code**: `servo_task`, `trajectory_task`, `eef_twist_task`, `teleop_task`.
**Paths**: the `gripper_command: In[Bool]` stream, the VR trigger.
**Interfaces**: `ManipulationModule`'s ports, every `pick_and_place` call site.

Only *blueprint values* consumed by those tasks change (R10), not the task code itself.

Continuous `0–1` control already exists via `teleop_task.on_gripper_trigger` (`:220-234`);
the `Bool` stream stays with `eef_twist` this PR. Consolidating all gripper handling into
`GripperTask` is a clean follow-up once it exists.

## 6. Blast radius

| Area | Files | Nature |
|---|---|---|
| Adapter protocol | `manipulators/spec.py` | Wrappers delegate (R2) |
| Adapters | `xarm`, `piper`, `mock`, `sim`, `openarm`, `galaxea_a1z`, `a750` | Slicing, `gripper_dof`, range constant, bidirectional conversion |
| Sim bridge | `simulation/engines/mujoco_shm.py:370` | Own `read_gripper_position` |
| Hardware wrapper | `hardware_interface.py` | One array; delete gripper branch + normalization |
| Components | `components.py` | Delete 2 fields; widen `make_gripper_joints` |
| **New task** | `control/tasks/gripper_task/` | `GripperTask` + `_registry.py` |
| Coordinator | `coordinator.py:901-931` | Delete 2 RPCs |
| `ManipulationModule` | `manipulation_module.py` | 2 method bodies + 1 docstring |
| Robot configs | `xarm`, `piper`, `a1z`, `a750`, `openyam` | Endpoint removal |
| Blueprints | gripper-bearing blueprints | Add `GripperTask`; `xarm/config.py:63` `0.85`→`1.0` |
| Message docs | `msgs/sensor_msgs/JointState.py` | Document gripper unit (R12) |

## 7. Non-goals

- A `GripperAdapter` protocol / `HardwareType.GRIPPER`.
- Vendor-specific gripper transport or driver.
- Motion planning for gripper joints — the planner deliberately excludes terminal
  prismatic joints (`planning/groups/discovery.py:301,333-350`).
- Closed-loop gripper completion and grasp detection (R26).
- The pre-existing equal-priority mode-conflict ordering in `_route_to_hardware`
  (`tick_loop.py:386-397`). R16 avoids triggering it by matching `SERVO_POSITION`.
- Retargeting or dexterous grasp generation.

