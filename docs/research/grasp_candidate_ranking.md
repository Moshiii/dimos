# Feasibility-aware grasp candidate ranking

## Question

Grasp proposal networks usually score the local hand-object interaction. That
score does not answer whether the robot can reach the grasp, approach it without
collision, or leave with the object. What other signals should order candidate
evaluation and select the grasp that is ultimately executed?

This note focuses on a fixed-base arm and the DimOS pick sequence:

```text
current joint state -> pre-grasp -> grasp -> retreat
```

## Conclusion

Do not replace the learned grasp score with one new heuristic. Treat proposal
quality and robot feasibility as different evidence, and use a staged policy:

1. Use cheap checks to reject malformed, unreachable, colliding, or near-singular
   candidates.
2. Plan the **entire connected sequence** for every surviving candidate within
   the configured budget.
3. Reject a candidate unless all required segments succeed from the preceding
   segment's actual endpoint.
4. Select among complete plans, not among poses. For an MVP, keep candidates in
   a configurable quality band near the best feasible proposal, then prefer the
   lowest full-sequence motion cost. Use clearance and manipulability as safety
   tie-breakers.
5. When post-pick intent is known, include its lift, retreat, transport, or place
   segment before choosing the grasp.

Candidate **evaluation order** should minimize time to find a feasible plan.
Final **execution order** should maximize expected task success. They need not
use the same score.

For the current implementation, planning is cheap enough that the first version
should plan all top-\(K\) candidates rather than train a reachability model.
Logging the resulting labels and costs creates the dataset needed for a learned
feasibility critic later.

## What the available oracles measure

| Oracle | Cost | What it catches | What it misses | Suggested role |
|---|---:|---|---|---|
| Proposal score | Already available | Hand-object grasp quality learned by GraspGenX | Robot embodiment, scene path, downstream task | Quality constraint or band |
| IK existence | Very low | Workspace and joint-limit infeasibility | A collision-free path to the IK state | Early hard filter |
| Collision-aware IK | Low | Invalid hand/arm state at key poses | Collision along the motion between poses | Early hard filter |
| Manipulability / singularity margin | Low after IK | Awkward or singular terminal posture | Obstacles and global path difficulty | Filter extreme cases; tie-break |
| Reachability/capability map | Very low online, offline setup | Likelihood that a TCP pose has IK | Current scene obstacles and exact start-state connectivity | Evaluation-order prior |
| Approach-corridor clearance | Low to medium | Gripper/object obstruction near contact | Whole-arm global path | Early filter/ranker |
| Full connected motion plan | Highest, but authoritative | Actual start-state connectivity and path collision | Model/perception error and execution dynamics | Required hard gate |
| Planned path cost | Free once planned | Joint travel, duration, clearance, mechanical work | Grasp stability unless combined with proposal quality | Final selection |
| Post-grasp task plan | Task-dependent | Grasp that blocks retreat, placement, or use | Unmodeled execution uncertainty | Required when intent is known |
| Perturbation pass rate | Multiplies planning cost | Sensitivity to pose/geometry/calibration error | Unmodeled uncertainty distributions | Robustness ranker |
| Learned feasibility critic | Very low online, data/training cost | Approximation to IK, collision, or planner success | Distribution shift; cannot certify safety | Evaluation-order prior only |

### IK feasibility and manipulability

MoveIt Grasps explicitly separates generation, filtering, and planning. Its
filter checks grasp and pre-grasp IK and collisions; only candidates that pass
are sent to a planner that constructs approach, lift, and retreat motions
([MoveIt Grasps documentation](https://moveit.github.io/moveit_tutorials/doc/moveit_grasps/moveit_grasps_tutorial.html#conceptual-overview)).
This is strong evidence that IK is a useful cheap filter, but not a sufficient
feasibility oracle.

Yoshikawa's manipulability measure quantifies how well a configuration can
produce end-effector motion; it approaches zero at singular configurations
([Yoshikawa, 1985](https://doi.org/10.1177/027836498500400201)). For grasp
selection, useful variants are:

- minimum manipulability at pre-grasp, grasp, and retreat;
- minimum singular value of the Jacobian, which directly exposes the weakest
  Cartesian direction;
- task-direction manipulability, especially along approach and retreat rather
  than an isotropic scalar.

Manipulability is not a proxy for reachability. A high-manipulability goal can
still have no collision-free path from the current state. It should reject
pathological postures or break ties, not replace motion planning.

There is direct evidence for using it to order expensive checks. Vahrenkamp et
al. precomputed a six-dimensional workspace representation, filtered
unreachable grasps, and ordered the survivors by manipulability before running
IK. In their reported 100-query comparison, total selection time fell from
9.97 s without precomputation to 2.24 s with binary reachability and 0.01 s
with the manipulability distribution
([Vahrenkamp et al., 2012](https://h2t.iar.kit.edu/pdf/Vahrenkamp2012c.pdf)).
These numbers demonstrate the value of a prior when candidate counts are high;
they do not make manipulability an exact path-feasibility test.

DimOS already has `get_manipulability()` and singularity checks in
`dimos/manipulation/planning/utils/kinematics_utils.py`, so this oracle does not
require a new kinematics stack.

### Reachability maps

Reachability maps precompute which end-effector poses admit IK. Akinola et al.
represent the reachable workspace as a signed-distance field and use it to
quickly screen and rank grasps
([Workspace Aware Online Grasp Planning](https://arxiv.org/abs/1806.11402)).
Their experiments report more reachable and successfully executed grasps and
less planning time. The SDF is valuable because it provides not just a Boolean
answer but a margin: poses deeper inside reachable space rank above poses near
its boundary.

This is a good future oracle when the number of proposals or planning latency
becomes large. It is robot-specific, must include orientation, and normally
does not represent the live obstacle scene or connectivity from the current
joint state. It therefore orders expensive checks; it does not certify a grasp.

### Collision clearance

Binary collision checking says whether a state or path is valid. Clearance
estimates how fragile that validity is. A candidate with a few millimeters of
clearance is more sensitive to segmentation, calibration, and controller error
than one with a larger margin.

OMPL officially supports path length, minimum path clearance, state-cost
integrals, and mechanical work as optimization objectives
([OMPL optimal-planning documentation](https://docs.ros.org/en/iron/p/ompl/doc/markdown/optimalPlanning.html)).
Its clearance tutorial uses inverse clearance as a path cost and notes the
accuracy/computation tradeoff when interpolating between states
([OMPL optimization-objectives tutorial](https://docs.ros.org/en/ros2_packages/jazzy/api/ompl/doc/markdown/optimizationObjectivesTutorial.html)).

For a grasp pipeline, distinguish:

- terminal clearance at pre-grasp and grasp;
- clearance along the local Cartesian approach, including the open gripper;
- whole-arm minimum clearance along `current -> pre-grasp`;
- attached-object clearance during retreat and later transport.

The local approach check is a useful cheap ranker. The minimum clearance over
the actual full plan is the more meaningful final metric. DimOS's Drake world
already exposes minimum signed distance; any clearance integration should use
the same planning-world collision semantics rather than a separate point-cloud
heuristic.

### Full motion-plan feasibility and cost

A successful full plan is the best available model-based oracle for
reachability from the robot's **actual** state. Berenson et al. combined object
grasp quality, local environment information, and robot kinematics to rank
precomputed grasps, showing that this ranking is important for efficient
selection in clutter
([Grasp Planning in Complex Scenes](https://publications.ri.cmu.edu/grasp-planning-in-complex-scenes)).

MoveIt Task Constructor expresses pick-and-place as interdependent stages,
passes their resulting states between stages, and supports path length,
trajectory duration, distance-to-reference, link motion, and clearance cost
terms
([MoveIt Task Constructor concepts](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html)).
This closely matches the connected-plan invariant in DimOS: a segment planned
from a fresh current state is not evidence that it connects to the preceding
segment.

GOMP goes further by optimizing motion and grasp choice jointly over a set of
candidate grasps. It chooses grasps that permit faster collision-free motions
and reports a 9x speedup over its baseline on a UR5
([Ichnowski et al., 2020](https://arxiv.org/abs/2003.02401)). Joint
optimization is more machinery than the DimOS MVP needs, but it supports the
principle that the planned trajectory—not only the contact score—is part of
grasp quality at the system level.

Useful plan costs, in increasing implementation complexity, are:

1. total joint-space path length across every segment;
2. time-parameterized trajectory duration;
3. weighted joint travel, penalizing joints near limits;
4. minimum or integrated collision clearance;
5. energy or torque cost when dynamics are modeled.

Path length is not a proof of execution ease, but it is deterministic,
inspectable, and already supported by DimOS utilities.

### Task and post-grasp feasibility

The easiest grasp to approach may make the object impossible to lift, place, or
use. MoveIt represents pre-grasp approach and post-grasp retreat explicitly in
its grasp message and pick pipeline
([MoveIt pick-and-place tutorial](https://moveit.picknik.ai/main/doc/examples/pick_place/pick_place_tutorial.html)).
MoveIt Task Constructor's serial stages only produce an end-to-end result when
the dependent subtasks connect.

TOG-Net gives a learning-based example of the same principle: it defines grasp
value using the probability that a grasp-conditioned downstream manipulation
policy completes a tool task, rather than lift success alone
([TOG-Net paper](https://www.roboticsproceedings.org/rss14/p12.pdf)).
GOMP-FIT gives a dynamics-based example. It adds acceleration constraints for
fragile or open-top objects; the reported experiments spilled up to 90% of
material with unconstrained GOMP and 0% with GOMP-FIT
([GOMP-FIT paper](https://arxiv.org/abs/2110.15326)). These are specialized
systems, but both show why the known post-grasp intent belongs in the selection
horizon.

Therefore the definition of “valid candidate” should follow the known task
horizon:

```text
pick only:       current -> pre-grasp -> grasp -> close -> retreat
pick and place:  current -> pre-grasp -> grasp -> close -> retreat
                -> pre-place -> place -> open -> retreat
tool use:        current -> grasp -> required tool trajectory
```

If only picking is currently in scope, including retreat is the correct MVP.
When a place goal becomes available, selection should be delayed until the
grasp-conditioned place path is checked.

### Robustness under uncertainty

Dex-Net's analytic labels estimate grasp robustness under uncertainty in object
pose, gripper pose, and friction rather than scoring a single nominal contact
configuration
([Dex-Net 2.0 project and paper](https://berkeleyautomation.github.io/dex-net/)).
Weisz and Allen more directly tested candidate reranking under pose error. The
highest nominal epsilon-quality grasp was generally not the most robust; ranking
by estimated force-closure probability under pose perturbations achieved 91%
physical success (85/93) versus 67% (63/93) for nominal epsilon ranking
([Weisz and Allen, 2012](https://www.cs.columbia.edu/~allen/PAPERS/weisz_icra12.pdf)).
The same idea can be applied to robot feasibility without treating Dex-Net as a
motion oracle:

1. sample small, calibrated perturbations of object pose, camera-to-world
   transform, and obstacle inflation;
2. rerun IK and/or the connected plan;
3. rank by pass fraction and worst-case clearance.

This is an engineering inference from robust grasp analysis, not a method
claimed by Dex-Net for arm motion. It is attractive after nominal planning
works because it directly measures sensitivity in the system's own planner.
Start with a handful of perturbations on the final few candidates; otherwise
the multiplicative planning cost is wasteful.

### Learned reachability and success critics

Learned models can predict whether a candidate will pass later stages. They are
most useful for ordering, not certification:

- Akinola et al. use a reachability SDF plus a learned motion-conditioned grasp
  quality model to reduce a large database to a few candidates in real time
  ([Dynamic Grasping with Reachability and Motion Awareness](https://arxiv.org/abs/2103.10562)).
- Jiang et al. jointly predict suction grasp quality and robot reachability.
  Adding the reachability prediction reduced reported motion-planning time from
  1.71 s to 0.90 s while slightly improving grasp success
  ([Jiang et al., 2022](https://doi.org/10.3389/fnbot.2022.806898)).
- SceneCollisionNet predicts collisions directly from partial scene and object
  point clouds and was used inside an MPPI policy for collision-free grasps and
  placements in simulated and physical clutter
  ([project page and paper](https://sites.google.com/nvidia.com/scenecollisionnet)).
- Lou et al. learn a reachability predictor from simulated robot experience
  alongside stable six-DoF grasp generation, reporting 82.5% success on unknown
  objects
  ([6-DoF Grasping with Reachability Awareness](https://arxiv.org/abs/1910.06404)).

For DimOS, the natural training target is not a manually invented “easy” label.
Log labels from the authoritative stack:

- IK pass/fail for each stage;
- connected-plan pass/fail and failed stage;
- planner runtime and iterations;
- full path length or duration;
- minimum clearance and manipulability;
- execution success, when hardware feedback exists.

A critic trained on these labels can rank candidates before RoboPlan, while
RoboPlan remains the hard gate. This avoids silently changing safety semantics
when the learned model encounters a new robot or scene distribution.

## Recommended DimOS policy

### MVP: plan all candidates, then choose

The current code sorts by proposal score and stops at the first candidate whose
connected pre-grasp, grasp, and retreat plans succeed. Replace the “first
success wins” decision with:

```text
proposals
  -> validate and deduplicate
  -> take top K by proposal score
  -> for each candidate:
       compute pre-grasp and retreat
       plan current -> pre-grasp -> grasp -> retreat as one connected hypothesis
       retain all paths or record the rejection stage
  -> discard incomplete hypotheses
  -> apply grasp-quality band
  -> choose minimum full-sequence motion cost
       tie-break by clearance, then manipulability, then proposal rank
```

A quality band prevents an extremely easy but visibly poor grasp from winning.
For example, keep candidates whose score is within a configurable delta of the
best **feasible** proposal. The delta must be tuned on GraspGenX validation
data; it should not be assumed that its raw score is a calibrated probability.
If candidate scores are nearly identical, as often happens for top proposals,
motion cost naturally decides.

The connected sequence already retains segment paths in
`ConnectedPoseSequenceResult.paths`. Sum joint-space lengths across these
paths for the first motion-cost implementation. This makes the change local
and explainable. It does not require a second planner or a parallel heuristic
world model.

### Evaluation order

Even while planning all top-\(K\), order affects latency and visualization.
Use this order:

1. proposal score, until data shows a better policy;
2. optionally, collision-aware IK and manipulability at key poses;
3. later, a robot-specific reachability SDF or a learned planner-success critic.

Do not use planner runtime as a final quality metric: failed and difficult
sampling-based searches are noisy, hardware-dependent, and affected by timeout.
Runtime/iteration count is useful for training an evaluation-order critic.

### Final selection tuple

Prefer a lexicographic policy over an opaque weighted sum for the MVP:

```text
(
  full_sequence_feasible,        # required
  within_grasp_quality_band,     # required
  robust_pass_fraction,          # later, maximize
  full_sequence_duration,        # minimize; path length until timing exists
  minimum_clearance,             # maximize
  minimum_manipulability,        # maximize
  proposal_score,                # maximize final tie-break
)
```

Feasibility and the quality band are constraints, not weights. The remaining
terms are easy to expose in logs and visualization, making candidate decisions
debuggable.

## What not to do

- Do not multiply an uncalibrated proposal score by arbitrary normalized motion
  heuristics. Small scaling changes can silently reverse decisions.
- Do not count independent success of `current -> pre-grasp` and
  `current -> grasp` as a connected grasp plan.
- Do not use IK feasibility as proof that a collision-free path exists.
- Do not let a reachability map or learned critic bypass exact collision-aware
  planning.
- Do not select before checking retreat or another known downstream task.
- Do not build a second point-cloud collision heuristic with semantics that
  differ from the planning world.

## Suggested evaluation

Measure candidate ordering separately from grasp execution:

- rank of first connected-feasible candidate;
- number of planner calls and wall time until first feasible candidate;
- percentage of scenes with at least one full feasible plan;
- selected plan length/duration and minimum clearance;
- proposal score of selected versus highest-scored proposal;
- physical pick success and failure phase;
- when robustness is added, success under held-out calibration and object-pose
  perturbations.

Compare three policies on the same recorded scenes and deterministic planner
seeds:

1. current proposal-score order, first feasible;
2. proposal-score order, plan all and select by the recommended policy;
3. later, reachability/critic order, same authoritative final selection.

This isolates whether an oracle improves search latency, final plan quality, or
actual execution success.
