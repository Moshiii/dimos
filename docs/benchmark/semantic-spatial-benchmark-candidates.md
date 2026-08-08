# Semantic spatial benchmark candidates

Research date: 2026-08-06

## Recommendation

Use **GOAT-Bench as the primary agent benchmark**. Keep **BenchBot active
Semantic SLAM** as a second, sensor-native benchmark for semantic mapping.

The distinction matters:

- GOAT-Bench asks the agent to understand and remember semantic goals while
  navigating: object categories, free-form descriptions, and goal images.
- BenchBot asks the system to actively explore and produce a geometrically and
  semantically correct object map. It exposes laser directly, but it is not an
  instruction-following or goal-understanding benchmark.

No candidate simultaneously offers GOAT-style semantic goal reasoning, an
official lidar observation, current container distribution, and official
automatic scoring. GOAT-Bench is the closer match to "agent spatial
understanding"; its official depth observation can be projected into a camera
frustum `PointCloud2` without inventing observations, although this is not a
lidar scan.

## Comparison

| Benchmark | What it evaluates | Official geometry | Official action contract | Official scoring | Runtime and access | Fit |
|---|---|---|---|---|---|---|
| **GOAT-Bench** (CVPR 2024) | Lifelong, open-vocabulary navigation through sequences of 5–10 category, language-description, and image goals | 360×640 RGB + depth and relative GPS/compass; **no lidar** | Discrete forward 0.25 m, turn/look 30°, and stop; 500 actions per subtask | A subtask succeeds when stop is called within 1 m; official Habitat evaluation aggregates success/path-efficiency metrics | Official code pins Python 3.7, Habitat-Sim/Lab 0.2.3; no official OCI image. HM3D requires requesting dataset access | **Best primary target:** semantic grounding, spatial memory, exploration, and efficient navigation are all exercised |
| **BenchBot active Semantic SLAM / BEAR** (IJRR 2022) | Active exploration followed by an object map containing probabilistic labels and 3D cuboids | RGB, depth, camera info, poses, and **laser are explicit official observations** | `move_angle` and `move_distance`; active and passive variants, with ground-truth or noisy pose | Official Object Map Quality (OMQ): combines 3D IoU and label probability, penalizing false positives and false negatives | Official stack uses Docker images (~50 GB), NVIDIA Omniverse/Isaac Sim 2022.2.1, and supports containerized submissions; code is BSD-3-Clause. Last evaluation release was 2022 | **Best second target:** exact lidar compatibility and automatic semantic-map scoring, but less natural agent goal reasoning |
| **OpenEQA A-EQA** (CVPR 2024) | Answer open-vocabulary questions through active, information-gathering navigation | HM3D/Habitat RGB-D in the active formulation; **no lidar contract** | Navigation actions, then a natural-language answer | Official GPT-4-based LLM-Match against reference answers | MIT code, but the official repository mainly distributes data/baselines/evaluator and was archived in Nov. 2025; it is not a packaged active simulator runner | Semantically excellent, but weaker as a first reproducible external-runtime integration |
| Habitat HM3D ObjectNav | Navigate to an object category in unseen scanned homes | Challenge profiles use RGB-D/pose variants; no lidar | Habitat discrete or continuous actions depending on challenge profile | Success, SPL and related Habitat metrics | Habitat supports Docker, but scene datasets have separate access terms | Established fallback, but GOAT-Bench exercises richer language, instance, and memory capability |

## GOAT-Bench contract

An episode spawns the agent in an unseen HM3D environment and supplies 5–10
goals in sequence. Each goal is represented by an object category, a language
description identifying an instance, or an image. At every step the official
observation contains RGB, depth, relative GPS/compass, and the current goal.
The official action set is forward, left, right, look up, look down, and stop.
The benchmark, not DimOS, decides success and computes its metrics. See the
[official GOAT-Bench repository and evaluation commands](https://github.com/Ram81/goat-bench)
and the [official CVPR paper](https://arxiv.org/abs/2404.06609).

For DimOS, the benchmark connection should expose the official information,
without adding privileged scene data:

```text
GoatConnection outputs: rgb, depth, depth_camera_info,
                        relative_pose, semantic_goal
GoatConnection action:  synchronous discrete benchmark step
Private eval output:    official Habitat/GOAT metrics
```

Depth may be losslessly projected using camera intrinsics into a frustum-shaped
`PointCloud2` for the existing mapper. The report and case metadata must call
this **RGB-D-derived geometry**, not lidar. The official discrete action remains
the action boundary. Exactly one action may be in flight: the connection waits
for GOAT to complete it and publish the next coherent observation epoch before
the DimOS action skill returns. `STOP` completes the current subgoal and may
advance the same lifelong episode to its next goal. The connection must not
change termination or score.

There is no organizer-published GOAT OCI image in the official instructions.
The reproducible integration should therefore build a pinned image from their
documented environment, pinning the GOAT commit, Habitat 0.2.3 versions, base
image digest, and final image digest. HM3D data should be mounted at runtime
after the user accepts its access terms; it should not be copied into the image.

## BenchBot contract

The official active/dead-reckoning task declares `image_depth`, RGB, camera
metadata, `laser`, and poses as observations; `move_angle` and `move_distance`
are the actions. Its result is an object map. See the exact
[official task YAML](https://github.com/benchbot-addons/tasks_ssu/blob/master/tasks/sslam_adr.yaml).

OMQ compares submitted and ground-truth object cuboids using the geometric mean
of spatial quality (3D IoU) and semantic label probability. Its denominator
accounts for matched objects, false negatives, and confidence-weighted false
positives. This is an official, fully automatic semantic-spatial score; DimOS
should store it unchanged. See the
[official OMQ implementation documentation](https://github.com/benchbot-addons/eval_omq).

The [official BenchBot stack](https://github.com/qcr/benchbot) already packages
its dependencies in Docker images and supports native or containerized
submissions. The practical drawbacks are its large installation, older pinned
Isaac Sim stack, and its map-production rather than semantic-goal-navigation
task shape.

## Why not OpenEQA first

OpenEQA is the strongest question-answering formulation: its active task asks an
agent to explore until it can answer a spatially grounded natural-language
question. It has 1,600+ questions across 180+ real environments and an official
automatic LLM-based evaluator. However, the
[official repository](https://github.com/facebookresearch/open-eqa) distributes
the dataset, episode histories, baselines, and evaluator rather than a pinned
closed-loop A-EQA runtime, and it is archived. Its official evaluator also needs
an external GPT-4 call. That makes faithful orchestration and score
reproducibility harder than GOAT-Bench.

## Proposed integration order

1. Integrate one complete GOAT-Bench evaluation split through `dimos eval`,
   returning the official metrics unchanged. This is the agent spatial
   understanding benchmark.
2. Integrate one BenchBot `semantic_slam:active:dead_reckoning` batch and retain
   official OMQ unchanged. This validates direct laser-based active semantic
   mapping.
3. Reconsider active OpenEQA only after a maintained, reproducible active runner
   is selected; do not recreate its evaluator or invent a substitute score.

## Primary sources

- [GOAT-Bench official repository](https://github.com/Ram81/goat-bench)
- [GOAT-Bench official paper](https://arxiv.org/abs/2404.06609)
- [BenchBot official stack](https://github.com/qcr/benchbot)
- [BenchBot SSU official task definitions](https://github.com/benchbot-addons/tasks_ssu)
- [BenchBot official OMQ evaluation](https://github.com/benchbot-addons/eval_omq)
- [OpenEQA official repository](https://github.com/facebookresearch/open-eqa)
- [OpenEQA official paper](https://open-eqa.github.io/assets/pdfs/paper.pdf)
- [Habitat-Sim official repository](https://github.com/facebookresearch/habitat-sim)
