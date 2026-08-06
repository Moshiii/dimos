---
title: "Single-Frame VQA"
---

# Single-Frame Perception VQA

This pipeline generates and evaluates visual question-answering examples from independent calibrated recording frames. It separates question proposal, private ground-truth generation, and visual evaluation so an evaluated model cannot see the answer evidence.

## Goals

- Generate questions from the visible image only.
- Establish answers from private segmentation and 3D geometry.
- Reject questions without sufficient geometric evidence.
- Evaluate a visual model using only an image and question.
- Preserve every input, tool trace, threshold, and result needed to audit an example.

## Model Roles

| Role | Input | Implementation | Access restrictions |
|---|---|---|---|
| Question proposer | Rectified image | OpenAI vision model, default `gpt-4o-mini` | No depth, point cloud, calibration, or answer evidence |
| Ground-truth perception agent | Calibrated frame and question intent | MoonDream, EdgeTAM, and geometry | Private; not exposed to the evaluated model |
| Evaluated visual model | Rectified image and accepted question | MoonDream currently | No tools or ground-truth artifacts |

MoonDream is used for private box detection and point localization. EdgeTAM produces masks from MoonDream box prompts or positive point prompts. The OpenAI proposer is intentionally separate from MoonDream so proposal and grounding do not use the same semantic model.

## Per-Frame Flow

1. The recording adapter loads one image, its corresponding geometry input, and calibration.
2. For the Go2 adapter, the source image is rectified and LiDAR is transformed from the world frame to the camera optical frame using the nearest recorded odometry pose and fixed camera mount. This happens only while constructing the self-contained frame.
3. The OpenAI question proposer lists salient visible objects. The pipeline expands each object into presence, horizontal-direction, coarse-distance, and left-versus-right nearest-object comparison question intents.
4. The private perception agent tries MoonDream box detection for each object query.
5. When no box is returned, MoonDream point localization supplies a positive point prompt to EdgeTAM.
6. EdgeTAM produces a foreground mask. Small masks are rejected.
7. The pipeline projects visible geometry into the image and retains foreground points inside the mask. Objects with too few points are rejected.
8. Accepted objects generate deterministic answers. Rejected intents have no answer and are excluded from evaluation.
9. The evaluated visual model receives only the rectified image and accepted question. Its response is normalized and compared with the private answer.

## Geometry Sources

| Source | Private geometry path | Evaluated model input |
|---|---|---|
| Go2 RGB plus LiDAR | World LiDAR to camera transform, then project foreground points into the image mask | Image only |
| RealSense RGB-D | Read aligned depth values in the image mask, then unproject through camera intrinsics | Image only |

Choose the source adapter based on the recording and available sensors. Both paths preserve the same evaluation boundary: the evaluated model receives only the image and question.

## Question Types

The current accepted intents are `presence`, `horizontal_direction`, `within_distance`, and `compare_nearest_by_side`.

Presence is accepted only with grounded foreground evidence. A failed detector or segmentation call is not proof of absence; it is a rejection. Direction uses left, center, and right image regions. Distance answers use a configured threshold rather than exact values. Side comparison requires at least one grounded left object and one grounded right object; it compares the nearest supported object on each side and rejects equal ranges.

## Quality Gates

| Parameter | Default | Meaning |
|---|---:|---|
| `--min-mask-area-px` | 128 | Minimum foreground-mask area in image pixels. This filters tiny or noisy masks; it is not a physical object-size threshold. |
| `--min-foreground-points` | 3 | Minimum depth or point-cloud samples inside the accepted mask. |

The frame manifest records the values used for every generated frame.

## Commands

Generate one explicit-query frame with `uv run dimos vqa single-frame --recording go2_hongkong_office --frame-index 1 --query plant`.

Generate one image-proposed frame with `OPENAI_API_KEY=... uv run dimos vqa single-frame --recording go2_hongkong_office --frame-index 1 --propose-questions`.

Generate a sampled, resumable dataset with `OPENAI_API_KEY=... uv run dimos vqa generate --recording go2_hongkong_office --start-index 0 --stop-index 100 --stride 10 --propose-questions`.

The batch command requires a finite `--stop-index` to avoid an accidental unbounded API run. Rerunning a dataset skips every frame directory that already contains `frame.json` and rebuilds aggregate manifests.

## Output Layout

The default root is `STATE_DIR/datasets/vqa/`, normally `~/.local/state/dimos/datasets/vqa/`. One dataset contains `frame-<index>/` directories plus `frames.jsonl`, `ground_truth.jsonl`, and `manifest.json`.

Each frame directory contains:

| Artifact | Purpose |
|---|---|
| `original_image.jpg` | Raw recording image before rectification, when available |
| `image.jpg` | Rectified image used for proposal, grounding, and evaluation |
| `grounding_overlay.jpg` | Private visual audit image: foreground masks, MoonDream detector boxes or point prompts, `Q` labels, and a side legend with every question and its answer or rejection reason |
| `frame.json` | Frame identity, source recording, artifact names, counts, model role, and quality thresholds |
| `intents.json` | Image-only proposed or explicit constrained question intents |
| `ground_truth.json` | Private answers or rejections, grounded-object evidence, and tool traces |
| `examples.json` | Accepted image/question/expected-answer records |
| `evaluations.json` | Image-only model responses and pass/fail results |

## Verifying A Frame

Review `original_image.jpg`, `image.jpg`, and `grounding_overlay.jpg` together. Confirm that a mask covers the intended visible object, the corresponding `ground_truth.json` evidence has enough foreground points, and its range/direction agrees with the overlay. Reject or remove any accepted example that remains semantically ambiguous despite passing automated thresholds.

`ground_truth.json` records `detect_objects`, `segment_objects`, `locate_object_point`, `segment_object_point`, `get_foreground_geometry`, and `reuse_grounding` operations. A point-prompt trace means MoonDream did not supply a box, but did localize a positive point that EdgeTAM then segmented.

## Evaluation Integration

The generated dataset is independent of the frozen-recording evaluation framework in PR #3378. After that framework merges, its case/task/validator models can be extended to evaluate generated VQA records. The image-only evaluation mode must pass only the image and question to the agent; expected answers and geometry evidence remain private. A separate future tool-use benchmark can intentionally expose perception tools to an agent.
