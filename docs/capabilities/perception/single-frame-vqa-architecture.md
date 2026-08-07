---
title: "Single-Frame VQA Architecture"
---

# Single-Frame VQA Architecture

This guide describes the code structure behind the [single-frame VQA pipeline](/docs/capabilities/perception/single-frame-vqa.md). It is intended for changing or debugging generation behavior, not for running the commands.

## Data Flow

One command selects independent recording frames. Each frame follows this private-data path:

```mermaid
flowchart TD
    command["vqa single-frame or generate"] --> recording["recording.py"]
    recording --> frame["CalibratedFrame: rectified image, geometry, calibration"]

    frame --> proposer["question_agent.py: image only"]
    proposer --> intents["QuestionIntent values"]

    frame --> ground_truth["ground_truth_agent.py: private"]
    intents --> ground_truth
    ground_truth --> detect["MoonDream object detection"]
    detect --> boxes{"Boxes found?"}
    boxes -->|yes| box_segment["EdgeTAM box segmentation"]
    boxes -->|no| point_locate["MoonDream point localization"]
    point_locate --> point_segment["EdgeTAM point segmentation"]
    box_segment --> geometry["geometry.py and grounding.py"]
    point_segment --> geometry
    geometry --> evidence{"Enough mask and point support?"}
    evidence -->|no| rejected["Rejected intent with reason and tool trace"]
    evidence -->|yes| rule["Fixed intent answer rule"]
    rule --> accepted["Accepted example with private answer and evidence"]

    accepted --> dataset["dataset.py"]
    rejected --> dataset
    dataset --> output["Images, public/private cases, and manifests"]
    output --> evaluator["vqa evaluate: LangChain gpt-5.6-luna"]
    evaluator --> score["Private oracle scoring"]
    score --> results["Model-specific results and summary"]
```

1. `recording.py` loads and rectifies the RGB image, aligns the recording's geometry to that image, and creates a `CalibratedFrame`.
2. `question_agent.py` sees only the rectified image and proposes object queries. It expands each query into supported `QuestionIntent` values.
3. `ground_truth_generator.py` receives the full calibrated frame and one intent. It detects the requested object, segments it, projects visible geometry into the image, and derives grounded objects.
4. The same generator applies the deterministic rule for the intent and returns either an answer with evidence or a rejection.
5. `dataset.py` persists images, accepted examples, rejections, private evidence, and self-contained public/private evaluator cases.
6. `evaluation/` asks a LangChain visual model only for an image, question, and allowed answers. It has no point cloud, mask, tool trace, or expected answer; the runner then scores the response against the private oracle.

The evaluation boundary is intentional: `ground_truth.json` must remain private when evaluating an image-only model.

## Source Layout

| File | Responsibility |
|---|---|
| `dimos/cli/vqa.py` | Typer commands. `single-frame` processes one Go2 recording frame; `generate` loops over sampled frames, skips completed frame directories, and rebuilds aggregate manifests. |
| `dimos/benchmark/vqa/models.py` | Typed contracts shared by VQA dataset generation and evaluation. |
| `dimos/benchmark/vqa/generation/recording.py` | Go2 recording adapter. Reads color image, LiDAR, and nearest odometry; rectifies the fisheye image and constructs the LiDAR-to-camera transform. |
| `dimos/benchmark/vqa/generation/question_agent.py` | Image-only OpenAI question proposer. Validates returned object names or structured intents and expands object names into the supported question types. |
| `dimos/benchmark/vqa/generation/ground_truth_generator.py` | Private deterministic executor. Calls detection, box segmentation or point-prompt fallback segmentation, grounds masks, caches per-object grounding, selects evidence, and accepts or rejects each intent. |
| `dimos/benchmark/vqa/generation/adapters.py` | Bridges MoonDream and EdgeTAM APIs to the private grounding interfaces. |
| `dimos/benchmark/vqa/generation/geometry.py` | Static calibrated pinhole projection and z-buffer visibility filtering. It returns only the nearest point at each image pixel. |
| `dimos/benchmark/vqa/generation/grounding.py` | Intersects projected visible points with each segmentation mask and computes the supported point count, median range, and left/center/right direction. |
| `dimos/benchmark/vqa/generation/questions.py` | Deterministic construction of presence, direction, and threshold-distance examples from grounded objects. |
| `dimos/benchmark/vqa/generation/dataset.py` | Writes per-frame artifacts and rebuilds `frames.jsonl`, `ground_truth.jsonl`, and `manifest.json` for a batch dataset. |
| `dimos/benchmark/vqa/evaluation/models.py` | Public VQA case, private oracle, LangChain configuration, and result contracts. |
| `dimos/benchmark/vqa/evaluation/scoring.py` | Normalizes closed image-only responses using public answer choices and compares them with private expected answers. |
| `dimos/benchmark/vqa/evaluation/langchain.py` | No-tools LangChain `gpt-5.6-luna` vision adapter. |
| `dimos/benchmark/vqa/evaluation/runner.py` | Loads public case inputs, invokes an answerer, then loads the private oracle and scores the response. |

## Ground-Truth Execution

The implementation does not let an LLM invent a perception procedure. The question type selects a fixed plan, while the object name and threshold are inputs to that plan.

| Intent | Input values | Deterministic answer rule |
|---|---|---|
| `presence` | Object query | Accept only if at least one object has a valid mask and sufficient foreground geometry. Answer `yes`; otherwise reject. |
| `horizontal_direction` | Object query | Select the nearest grounded object and return its `left`, `center`, or `right` image region. |
| `within_distance` | Object query and distance threshold | Select the nearest grounded object and compare its median range to the threshold. |
| `compare_nearest_by_side` | Object query | Select the nearest grounded object on the left and on the right, then return the closer side. Reject if either side lacks evidence or the ranges are equal. |

For every intent, grounding first tries MoonDream box detection followed by EdgeTAM box segmentation. If detection returns no boxes, it can fall back to MoonDream positive-point localization followed by EdgeTAM point-prompt segmentation. Small masks or masks with too few projected foreground points do not establish a negative answer; they cause rejection.

## Frame And Dataset Records

`single-frame` writes the record directly. `generate` uses `write_frame_record()` for each frame and then uses `write_dataset_manifest()` to aggregate all completed frames.

| Artifact | Contains | Privacy |
|---|---|---|
| `image.jpg` | Rectified image used by private generation | Copied into each public evaluator case. |
| `original_image.jpg` | Raw recording image, if available | Audit artifact only. |
| `grounding_overlay.jpg` | Private masks, MoonDream detector boxes or point prompts, `Q` labels, and an answer/rejection legend | Private audit artifact. |
| `intents.json` | Proposed or explicit constrained intents | Generation metadata. |
| `examples.json` | Accepted public case summaries | Public metadata. |
| `ground_truth.json` | Answered and rejected results, grounded-object evidence, and tool traces | Private ground truth. |
| `cases/<case-id>/case.json` | Public question, image path, and allowed answers | The only case metadata sent to the model. |
| `cases/<case-id>/private/` | Expected answer and case-specific grounding result | Private oracle and audit evidence. |
| `evaluations/<model>/` | Closed-answer responses, pass/fail results, and aggregate summary | Trusted evaluator output. |
| `frame.json` | Frame identity, source, artifact names, counts, models, and grounding thresholds | Dataset metadata. |
| `frames.jsonl` | One `frame.json` record per completed batch frame | Batch metadata. |
| `ground_truth.jsonl` | One private result record per intent across the batch | Private aggregate ground truth. |
| `manifest.json` | Counts of frames, accepted questions, and rejections | Batch summary. |

## Extension Points

To add a recording source, create an adapter that returns `CalibratedFrame` with a rectified image, matching camera intrinsics, and a point cloud or depth-derived point cloud in the camera frame. Then expose it through the CLI without changing the question, grounding, or evaluation privacy boundary.

To add a question type, extend `QuestionKind`, validate it in `generation/question_agent.py`, add its deterministic answer rule in `generation/ground_truth_generator.py`, expose it in explicit CLI query expansion, and add accepted and rejected test cases. The tool trace and grounded evidence should remain sufficient to independently audit the answer.
