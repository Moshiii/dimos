# Verifying real-time simulator navigation tasks

## Goal

This catalog defines distinct navigation task shapes for `dimos eval` and the
private evidence required to verify each one. It covers navigation within an
existing PiMSim or DimSim scene. It excludes manipulation and new scene work.

The main design problem is verification. Memory preparation, target visibility,
start pose, route length, and similar variations can be added later without
creating new verifier categories.

## Acceptance bar

A category is delivered only when:

1. A deterministic baseline satisfies one concrete task in an existing scene.
2. Controlled evidence proves that the checker accepts success and rejects
   failure.
3. The case runs through `dimos eval run` and writes normal attempt artifacts.
4. A reviewer inspects the episode and checker result in Rerun.

An LLM-agent pass is useful evidence, but it is not an implementation gate.

The public instruction names every scored behavior. The private verifier may
hide exact entity IDs, scene geometry, and tolerances, but it may not impose a
secret route, safety, or efficiency requirement.

## Verifier model

Each checker is a small state machine over evaluator-only simulator evidence:

```text
reset receipt
    + episode identity
    + reset pose
    + scene revision
             |
             v
timestamped simulator snapshots ---- evaluator-owned intervention events
    + robot pose/velocity                    + applied simulator tick
    + individual entity state               + stable intervention ID
    + contact events                                  |
             |                                        |
             +------------------+---------------------+
                                v
                      task-specific state machine
                                |
                    success, failure, or timeout
```

The checker uses simulator time, not agent completion messages. Each update
returns `CONTINUE`, `PASS`, or `FAIL`. `PASS` and `FAIL` terminate the episode
immediately and record the triggering snapshot or event; `CONTINUE` leaves the
episode active until another update or timeout. The evaluated agent cannot
access the private snapshots, compiled entity IDs, or checker state.

The evaluation control thread may poll periodically, but each poll processes a
complete ordered delta of compact control-step frames rather than one latest
snapshot. A cursor identifies the last processed frame. A missing frame,
out-of-order stamp, episode mismatch, or buffer overflow makes the attempt an
infrastructure failure; it cannot become a task pass or failure.

The case serializes a stable checker ID, checker revision, and validated facts
such as target IDs, order, thresholds, and budgets. Trusted repository code
implements the state machine. The case does not serialize executable logic or
a general temporal-expression tree. This follows the benchmark patterns
summarized in
[How embodied benchmarks represent goals and checking](embodied-benchmark-goal-representation.md).

## Category catalog

### 1. Single destination

**Task:** “Go to the bed.”

**Checker:** Resolve the compiled target entity, calculate planar distance from
the robot to its bounds on every snapshot, and pass when the distance enters
the allowed radius. Reject a trace that never enters the radius.

**Existing scene:** `dimsim-apartment`.

**Readiness:** Implemented in the current go-to-bed case. This is the reference
checker for the other categories.

### 2. Multi-stop itinerary

**Task:** “Visit the television, then the bed.”

**Checker:** Compile an ordered list of target regions. Advance an index only
when the robot enters the next region. Pass after the final transition. A bed
visit before the television does not advance the checker.

**Existing scene:** `dimsim-apartment`.

**Readiness:** Local work. PiMSim already supplies timestamped robot poses;
`pimsim_dimos` must expose them to a trace-aware DimOS checker.

### 3. Coverage or patrol

**Task:** “Visit the television, sofa, bed, and bathtub.”

**Checker:** Maintain a set of visited compiled target regions. Pass when the
set contains every required target. Order does not matter. Reject a trace with
one or more missing targets.

**Existing scene:** `dimsim-apartment`.

**Readiness:** Local work using the same snapshot exposure as the itinerary
checker.

### 4. Constrained route

**Task:** “Go to the bed via the sofa without entering the television area.”

**Checker:** Combine three predicates: the sofa region must be visited, the
television exclusion region must never be entered, and the bed must be reached
after the sofa. A forbidden-region entry fails the task immediately.

**Existing scene:** `dimsim-apartment`, after a deterministic baseline confirms
that the route is feasible.

**Readiness:** Local work. It needs trace predicates but no PiMSim-core change.

### 5. Round trip

**Task:** “Visit the television, then return to where you started.”

**Checker:** Store the applied reset pose, wait for a television-region visit,
then pass when the robot returns within the start tolerance. Returning before
visiting the television does not pass.

**Existing scene:** `dimsim-apartment`.

**Readiness:** Local work. The adapter must include the applied reset pose and
episode identity in evaluator evidence.

### 6. Precise arrival

**Task:** “Stop 1–2 metres from the television, face it, and remain stopped for
5 seconds.”

**Checker:** Require a continuous simulator-time window during which distance
stays inside the band, planar speed stays below the stop threshold, and heading
stays within the target-facing tolerance. Reset the dwell timer whenever any
condition fails.

**Existing scene:** `dimsim-apartment`.

**Readiness:** Local work. PiMSim frames already contain robot pose, linear
velocity, angular velocity, and simulator time.

### 7. Budgeted navigation

**Task:** “Reach the television within 45 seconds while travelling no more than
15 metres.”

**Checker:** Integrate planar distance between successive robot poses and read
elapsed simulator time from frame stamps. Pass only if the target is reached
before both public limits. Reject non-finite or discontinuous trace data.

**Existing scene:** `dimsim-apartment`, after the deterministic baseline sets
feasible public limits with a declared margin.

**Readiness:** Local work. PiMSim already supplies the required pose and time
trace.

### 8. Disturbance recovery

**Task:** “Reach the goal. If the direct route becomes blocked, find another
way.”

**Checker:** Record the evaluator-owned wall insertion and its applied simulator
tick. Pass only if the robot reaches the goal after that event. The baseline
must show that the inserted wall blocks the direct route and leaves another
route open.

**Existing scene:** The existing flat-ground dynamic-wall scenario used by the
path-replanning system test.

**Readiness:** Local work. PiMSim already supports wall insertion. The adapter
and evaluator must retain its ID, acknowledgement, and timing.

### 9. Moving-target navigation

**Task:** “Follow the person while remaining 1–2 metres behind for 10 seconds.”

**Checker:** Track the exact moving entity and robot on the same simulator
timeline. Require the distance band and relative behind-angle for a continuous
window while also requiring minimum target displacement. This prevents a
stationary pair from passing.

**Existing scene:** None with a controllable, authoritative moving navigation
entity.

**Readiness:** Blocked by PiMSim core and existing-scene support. The simulator
needs an addressable moving entity with a scripted trajectory and timestamped
state. This work will not create a scene locally.

### 10. Semantic selection

**Task:** “Go to the office chair nearest the printer.”

**Checker:** At reset, resolve individual entities by stable scene-owned tags
and IDs, calculate the unique nearest chair to the printer, and freeze that
exact target ID into the checker state. Pass on proximity to that chair. Fail
closed if the relation is tied or the required entities are missing.

**Existing scene:** `office`, whose package declares 17 chair entities and one
printer entity with IDs, tags, poses, and bounds.

**Readiness:** Local work. PiMSim already loads individual package entities and
publishes tracked entity states internally. `pimsim_dimos` must expose them to
the evaluator instead of returning one aggregate semantic box.

### 11. Safety-constrained navigation

**Task:** “Go to the bed without colliding with furniture.”

**Checker:** Pass only if the bed is reached and no disallowed contact event
occurs. The checker must distinguish normal foot–floor support from contact
with furniture or obstacles.

**Existing scene:** `dimsim-apartment`.

**Readiness:** Blocked by PiMSim core. Current robot frames provide aggregate
external-contact count and force, but normal support contacts are external too.
They do not identify the bodies or geoms in contact.

## Verification primitives to implement locally

The locally ready categories reduce to a small set of reusable predicates:

| Primitive | Used by |
| --- | --- |
| Enter a point, bounds, or distance band | Single destination, itinerary, coverage, constrained route, round trip, precise arrival, budgeted navigation, semantic selection |
| Visit targets in order | Multi-stop itinerary, constrained route, round trip |
| Visit every target | Coverage or patrol |
| Never enter a region | Constrained route |
| Hold conditions continuously for simulator time | Precise arrival |
| Accumulate planar path length | Budgeted navigation |
| Observe an acknowledged intervention before success | Disturbance recovery |
| Resolve an unambiguous individual entity relation at reset | Semantic selection |

Checker implementations may reuse these predicates internally. A case selects
one versioned checker and supplies its validated data. PiMSim should expose
generic state and controls; it should not implement task-specific methods such
as `check_go_to_bed()`.

## Ownership report

### PiMSim already provides

- Session and episode identity, physics ticks, control ticks, and simulator
  time through `FrameStamp`.
- Robot and tracked-entity pose and velocity through `ObservationFrame`.
- Deterministic reset with a seed and `ScenarioSpec`.
- Online observation hooks through `SessionObserver`.
- Pose, observation, and action trace recording in the characterization tools.
- Repeatability comparison across reset episodes.
- Stable IDs, tags, initial poses, and bounds for separately declared scene
  entities.
- Dynamic wall insertion.

### Implement in `pimsim_dimos`

- A provider-neutral evaluator snapshot containing the frame stamp, robot
  state, relevant robot metrics, and individual tracked entity states.
- A reset receipt containing seed, scenario ID, session ID, episode ID, world
  revision, and applied spawn pose.
- A bounded evaluator trace or equivalent online snapshot stream that remains
  private from the agent.
- Cursor-based draining that reports frame gaps and buffer overflow explicitly.
- Stable IDs, acknowledgements, and applied ticks for interventions.
- Scene control that is not bound directly to the Go2 runtime class.

### Implement in the DimOS evaluator

- A trusted registry of code-defined checkers selected by stable ID and
  revision, with validated data models for their inputs.
- Shared internal implementations of the verifier primitives above.
- Simulator-time polling and trace evaluation.
- Bounded private trace and checker-transition artifacts.
- Deterministic category baselines.
- Positive and negative checker tests.
- General source preparation for reset poses and evaluator interventions.
- One headless command and one Rerun Web command per delivered task.

## Report to the PiMSim owner

Only three current categories require PiMSim-core work.

### Moving entities

Provide an addressable dynamic entity with an evaluator-controlled trajectory,
stable identity, and state on the episode timeline. This unlocks moving-target
following and moving-obstacle avoidance. No compatible existing navigation
scene currently exposes this capability.

### Contact identity

Expose contact events with both entity, body, or geom identities; contact
point; normal; force; and frame stamp. At minimum, classify normal support
contacts separately from obstacle contacts. Aggregate count and force cannot
verify collision-free locomotion.

### Semantic regions and orientation

Scene packages do not yet expose authoritative named rooms, aisles, doorways,
or canonical object-facing directions. Adding these facts would unlock room
navigation and object-relative approach tasks. Benchmarks should not recreate
them in private annotations.

Two API improvements are useful but do not block the current local catalog:

- Return individual semantic query results instead of merging every string
  match into one axis-aligned box. Preserve ID, tags or classes, pose, bounds,
  frame, and provenance.
- Define supported `ScenarioSpec.parameters`. The type accepts arbitrary
  parameters, but `MujocoBackend.reset()` currently rejects any non-empty
  mapping.

## Review worksheet

Each delivered category adds one row after automated verification and manual
review:

| Category | Case path | Baseline | Accept/reject tests | CLI run | Manual Rerun review | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| Single destination | `dimos/benchmark/realtime_sim/cases/go2-apartment-go-to-bed/case.json` | Pending formal record | Existing proximity tests; rerun after refactor | Previously exercised | Pending catalog review | Reference task |

Headless run:

```bash
uv run dimos --viewer none eval run \
  dimos/benchmark/realtime_sim/cases/<case-id>/case.json \
  --output=/tmp/dimos-eval-<case-id>
```

Manual Rerun Web review:

```bash
uv run dimos \
  --viewer rerun \
  --rerun-web \
  --rerun-open web \
  eval run \
  dimos/benchmark/realtime_sim/cases/<case-id>/case.json \
  --output=/tmp/dimos-eval-<case-id>
```
