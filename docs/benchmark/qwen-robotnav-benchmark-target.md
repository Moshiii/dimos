# Choosing a navigation benchmark from Qwen-RobotNav

## Recommendation

Use **HM3D-OVON as the first benchmark protocol to integrate with `dimos eval`**.
It best matches the capability already exercised by the PiMSim apartment case:
search for an object named in free-form text, approach a valid instance, and
declare completion. Its episode data and reference evaluator are public. The
benchmark also supplies a better oracle than the current object-AABB radius:
geodesic distance to a set of navigable viewpoints from which the object is
visible.

This recommendation does **not** mean that an apartment case with an OVON-like
checker is an HM3D-OVON result. Running the actual benchmark is blocked on a
PiMSim-core capability: loading the corresponding licensed HM3D scenes and
their semantic/navmesh data, or providing an equivalent, validated import.
Until then, DimOS can implement and test the episode adapter, explicit-finish
contract, trajectory recorder, metrics, and checker against fixtures, but must
label PiMSim apartment cases “OVON-style,” not “HM3D-OVON.”

The paper does not introduce one new navigation dataset. It evaluates existing
benchmarks. Its three stated embodied-navigation evaluations are VLN-CE,
open-vocabulary ObjectNav, and EVT-Bench; it reports VLN-CE R2R/RxR, HM3D-OVON,
and EVT-Bench Single Target results separately ([paper, evaluation
section](https://arxiv.org/html/2606.18112#S5.SS2)). The official Qwen-RobotNav
repository currently contains documentation and media, not its evaluation
harness or model weights, and says the weights are not planned for release
([official repository](https://github.com/QwenLM/Qwen-RobotNav)). We therefore
need to integrate the original benchmark implementations, not reverse-engineer
Qwen's unpublished runner.

## Candidate comparison

| Target | Capability | Public episode/evaluator status | Main metrics | PiMSim fit | Recommendation |
| --- | --- | --- | --- | --- | --- |
| HM3D-OVON | Search for a free-form object category and stop at any valid instance | Episode archive and reference Habitat code are public; HM3D scenes require Matterport access | SR, SPL, distance-to-goal; Seen, Synonyms, and Unseen splits | Closest to current static semantic navigation | **First** |
| HM3Dv2 ObjectNav | Search for one of six fixed categories and stop at any valid instance | Official 245 MB episode release and challenge evaluator are public; HM3D scenes require Matterport access | SR, SPL, soft-SPL, distance, steps, and collisions | Same underlying integration, but less representative of open-language agent use | Conformance subset |
| VLN-CE R2R, then RxR | Follow a route described by language | R2R/RxR ports and evaluator are public; MP3D scenes require Matterport access | NE, oracle success, SR, SPL, and nDTW | Requires reference-path trace scoring and instruction-following behavior not yet isolated by current cases | Second |
| EVT-Bench STT | Track a moving person while maintaining view and distance | Code is public, but the repository does not clearly publish or document the episode archive | tracking rate, success rate, collision rate | Requires moving humanoids, target motion programs, visibility, and contacts | Defer |

Every exact integration listed above requires original scene assets and
simulator semantics. Choosing OVON first minimizes the *additional* task
machinery; it does not remove the shared scene-import blocker.

The paper also reports closed-vocabulary HM3Dv2 ObjectNav. The 2023 Habitat
challenge defines six goals—chair, couch, potted plant, bed, toilet, and TV—on
216 HM3D-Semantics v0.2 scenes and publishes a 245 MB episode set
([official challenge](https://github.com/facebookresearch/habitat-challenge),
[Habitat dataset catalog](https://github.com/facebookresearch/habitat-lab/blob/main/DATASETS.md)).
Its episode/checker foundation is the same ObjectNav family used by OVON. We
should let one adapter support both, using HM3Dv2's six-category episodes as a
small conformance profile rather than building a separate evaluation stack.

## HM3D-OVON: the first target

### Task and episode contract

HM3D-OVON expands ObjectNav to free-form goals, with more than 15,000 annotated
object instances across 379 categories ([official
README](https://github.com/naokiyokoyama/ovon#overview)). Qwen-RobotNav reports
the benchmark's Seen, Synonyms, and Unseen splits using SR and SPL
([paper](https://arxiv.org/html/2606.18112#S5.SS2.SSS2)).

The serialized episode contract is:

- `episode_id`, `scene_id`, `start_position`, and `start_rotation`;
- `object_category`, plus optional child categories;
- `info`, including episode geodesic and Euclidean distance;
- a scene/category key resolving all acceptable object instances;
- for each object: stable object ID, position, category, and valid view points;
- for each view point: an agent pose, visibility IoU, and optional radius.

The official loader deduplicates the potentially large goal sets by
scene/category and restores every object and view point when reading an episode
([episode loader](https://github.com/naokiyokoyama/ovon/blob/main/ovon/dataset/ovon_dataset.py)).
The 168 MB episode repository is labelled MIT
([episode archive](https://huggingface.co/datasets/nyokoyama/hm3d_ovon)). That
label does not cover the underlying HM3D scans.

### Success, metrics, and termination

The reference distance measure computes the shortest navigable **geodesic**
distance from the current agent position to any valid view point of any
acceptable instance. It can include child categories. It does not measure
Euclidean distance to an object center or AABB
([OVON distance implementation](https://github.com/naokiyokoyama/ovon/blob/main/ovon/measurements/nav.py)).

The registered Habitat `Success` measure requires the agent to issue `STOP`
while inside the success distance
([Habitat v0.2.3 success implementation](https://github.com/facebookresearch/habitat-lab/blob/v0.2.3/habitat-lab/habitat/tasks/nav/nav.py)).
OVON's published experiment configs set that distance to **0.25 m**, use the
OVON view-point distance measure, and report standard SPL
([reference experiment config](https://github.com/naokiyokoyama/ovon/blob/main/config/experiments/transformer_dagger.yaml)).
The task permits 500 steps; the Locobot profile uses 30-degree turns, a 0.18 m
agent radius, and disables sliding
([task config](https://github.com/naokiyokoyama/ovon/blob/main/config/tasks/objectnav_locobot_hm3d.yaml)).
An episode ends on `STOP` or the step limit. A successful `STOP` sets SR to one;
SPL multiplies success by the shortest-path length divided by the larger of
the shortest and traveled path lengths.

The checker implication is concrete: `semantic_object_bounds(query)` is not
enough. Exact evaluation needs a finish event, the full agent path, the
episode's valid approach viewpoints, and navmesh geodesic queries.

### Access, license, and runtime

The official setup pins Python 3.7, Habitat-Sim 0.2.3 headless,
Habitat-Lab 0.2.3, PyTorch 1.12.1, and CUDA 11.3
([installation](https://github.com/naokiyokoyama/ovon#hammer_and_wrench-installation)).
The episode archive is directly downloadable. HM3D scene downloads require
Matterport credentials. HM3D contains 1,000 scans, is offered for academic,
non-commercial research, and withholds test scenes
([official HM3D access page](https://aihabitat.org/datasets/hm3d/)).

The licenses are therefore layered: the episode repository says MIT, while the
scene data remains governed by Matterport's terms. We should not redistribute
the scenes or describe the combined benchmark as MIT.

## VLN-CE R2R and RxR

VLN-CE tests instruction following in continuous Matterport3D environments.
R2R provides English route instructions; RxR adds longer routes, multiple
languages, and guide/follower annotations. The official implementation pins
Habitat-Sim and Habitat-Lab 0.1.7 and supports both datasets
([official README](https://github.com/jacobkrantz/VLN-CE)).

An episode contains an ID, trajectory ID, scene and start pose, instruction,
goal position, reference path, and reference geodesic distance. Preprocessed
ground truth adds per-step locations and actions
([official R2R-VLNCE schema](https://jacobkrantz.github.io/vlnce/data)). R2R's
current archive is 3 MB, or 250 MB with preprocessed traces. The public splits
contain 10,819 train, 778 val-seen, 1,839 val-unseen, and 3,408 test episodes.

The reference task allows `STOP`, forward 0.25 m, and 15-degree turns, with a
500-step limit and 3 m success threshold
([task config](https://github.com/jacobkrantz/VLN-CE/blob/master/habitat_extensions/config/vlnce_task.yaml)).
It reports navigation error, oracle success, success, SPL, and nDTW. Oracle
success records whether the trajectory ever enters the success radius. nDTW
compares the executed position trace with the reference path, while success
still depends on stopping near the destination
([measure implementations](https://github.com/jacobkrantz/VLN-CE/blob/master/habitat_extensions/measures.py)).

The code is MIT, but the task datasets and trained models are MP3D-derived and
distributed under Matterport terms and CC BY-NC-SA 3.0 US
([license statement](https://github.com/jacobkrantz/VLN-CE#license)). Exact
PiMSim evaluation therefore needs MP3D scene import, its navmesh, reference-path
coordinates, explicit finish, and complete path capture. Unlike OVON, merely
reaching the final region misses a central capability: following the instructed
route, measured by nDTW.

## EVT-Bench

EVT-Bench evaluates active visual tracking. Qwen-RobotNav uses its Single Target
Tracking split and reports tracking rate, success rate, and collision rate
([paper](https://arxiv.org/html/2606.18112#S5.SS2.SSS3)). The official release
also names Distractor Tracking and Adversarial Tracking splits
([official README](https://github.com/wsakobe/TrackVLA#-evaluation)).

The released runner uses Habitat-Sim 0.3.1, a vendored Habitat-Lab, a Spot-like
robot, a leader humanoid, and up to seven additional humanoids
([STT configuration](https://github.com/wsakobe/TrackVLA/blob/main/habitat-lab/habitat/config/benchmark/nav/track/track_infer_stt.yaml)).
At each step it records robot-to-leader distance and whether the target is
visually detected. `human_following` requires the target to be visible and no
farther than 3 m; the success band is 1-3 m. A distance below 0.5 m is a human
collision
([measure code](https://github.com/wsakobe/TrackVLA/blob/main/evt_bench/additional_metric.py)).
The evaluation loop terminates after 300 steps, after more than 20 consecutive
steps beyond 4 m, or on collision, and aggregates frame-level following,
episode success, and collisions
([runner](https://github.com/wsakobe/TrackVLA/blob/main/baseline_agent.py),
[aggregation](https://github.com/wsakobe/TrackVLA/blob/main/analyze_results.py)).

The repository declares CC BY-NC-SA 4.0 and provides code for STT, DT, and AT.
However, its README tells users how to download scene and humanoid assets but
does not give a download link or command for `data/datasets/track/...`, which
the configs require. The repository also loads these files as generic
`PointNav-v1` episodes and accesses extra `info` keys from task code instead of
publishing a formal schema. Exact episode availability and schema are therefore
**ambiguous in the official release**. This alone makes EVT-Bench a poor first
integration target, even before PiMSim's missing dynamic-human and contact
capabilities.

## DimOS/PiMSim implementation boundary

### Work we can own here

Once a PiMSim scene can satisfy an episode's scene identity, DimOS and the
`pimsim_dimos` adapter can provide:

1. An HM3D-OVON episode reader that preserves source IDs and private goal data.
2. Reset to the serialized start pose and validation that expected object IDs
   and view points exist.
3. An explicit `finish`/`STOP` event; automatic proximity success is not
   benchmark-equivalent.
4. Complete per-control-step pose capture, with buffer-loss treated as an
   infrastructure error.
5. `CONTINUE`, `PASS`, and terminal `FAIL` outcomes for stop-too-far and timeout.
6. SR, distance-to-goal, traveled path length, and SPL artifacts.
7. Unit tests with satisfying and rejecting traces, a deterministic baseline,
   a `dimos eval run` smoke, and manual Rerun inspection.

### PiMSim-core blockers

Exact HM3D-OVON execution remains blocked until PiMSim core can provide all of:

- an HM3D v0.2 scene/semantic import with stable source object IDs;
- the benchmark navmesh or a validated equivalent for the configured robot;
- multi-target geodesic distance to the episode's valid view points;
- collision and motion behavior compatible enough to state what has changed
  from the Habitat Locobot protocol.

VLN-CE adds MP3D import and reference-path coordinate fidelity. EVT-Bench adds
scripted articulated humans, target/distractor identity, visual visibility,
and contact events. Per the project boundary, these core gaps block exact
features; adapter and evaluator gaps do not.

## Compatibility claim

There are three distinct outcomes, and result artifacts should name which one
was run:

- **Exact benchmark episode:** original episode, original scene semantics,
  checker, thresholds, and action/termination protocol. Scores may be compared
  only if embodiment and dynamics also match the benchmark requirements.
- **Protocol-compatible port:** original episode semantics and metrics on a
  materially different embodiment or dynamics. Useful, but not leaderboard
  comparable.
- **Benchmark-inspired case:** the same capability and checker shape on a
  different PiMSim scene. This is what can be built before HM3D import; it must
  not be reported as HM3D-OVON.

This distinction lets us adopt a proven benchmark design immediately without
claiming equivalence that the simulator backend cannot yet support.
