---
title: "VQA Benchmark"
---

# Point-Cloud-Grounded VQA Benchmark

DimOS generates visual question-answering (VQA) cases from frozen robot recordings and evaluates an image-only model against private, point-cloud-grounded answers. The benchmark is designed so the model being scored cannot see calibration, point clouds, segmentation, measurements, tool calls, or expected answers.

## Pipeline Flow

```text
Frozen robot recording
        |
        v
Rectified RGB image + calibrated visible point cloud
        |
        +-------------------- constrained generation --------------------+
        | Image-only object author -> deterministic question families     |
        | -> private MoonDream/EdgeTAM/LiDAR grounding                    |
        | -> geometry quality-gate validation -> accepted or rejected     |
        +-----------------------------------------------------------------+
        |
        +---------------------- agentic generation -----------------------+
        | Image-only free-form author -> frozen public question/contract  |
        | -> private local oracle tool calls                              |
        | -> geometry quality-gate validation                             |
        | -> private semantic-evidence validation -> accepted or rejected |
        +-----------------------------------------------------------------+
        |
        v
Accepted public cases: image + question + answer contract
        |
        v
Image-only evaluator -> answer-format validation and scoring -> results + viewer
```

The private generation evidence, validation decisions, and expected answers are retained for audit but are never provided to the evaluator.

## Trust Boundary

The benchmark has three independent roles.

| Role | Sees | Does not see |
|---|---|---|
| Question author | Rectified RGB image | Point cloud, calibration, tool output, answer |
| Private oracle | Frozen frame, point cloud, calibration, local tools, frozen question | Evaluated-model response |
| Evaluated model | Public image, question, and answer contract | Oracle, tools, point cloud, private evidence, answer |

The question is frozen before the private oracle runs. This prevents selecting questions after inspecting private geometry. The oracle either produces cited evidence or rejects the question. Rejection is expected and is preferable to fabricated ground truth.

## Input Frame

The Go2 adapter loads a synchronized RGB frame, LiDAR observation, and odometry pose from a Memory2 recording. It rectifies the camera image and transforms LiDAR into the camera optical frame.

Each generated frame directory has three image artifacts:

| Artifact | Purpose |
|---|---|
| `original_image.jpg` | Raw Go2 fisheye image retained for sensor and calibration audit. |
| `image.jpg` | Rectified image used by the question author, private perception, and evaluated model. |
| `grounding_overlay.jpg` | Private audit view with segmentation masks, detector boxes, and point prompts over the rectified image. |

## Generation Modes

### Constrained

Constrained generation starts with an image-only OpenAI proposer that lists salient objects. The generator expands each object into fixed, deterministic question families:

- presence;
- horizontal direction;
- distance threshold;
- nearest left-versus-right comparison.

Additional deterministic question families will be added as their private evidence programs mature.

MoonDream detects or point-localizes the object, EdgeTAM produces a mask, and visible LiDAR points inside the mask establish range and direction. This mode is useful for stable, high-volume baseline data.

### Agentic

Agentic generation uses a free-form image-only question author followed by a private tool-calling oracle. It does not call the constrained `QuestionIntent` answer programs.

The author is prompted to prefer point-cloud-verifiable questions: object height, metric comparisons, object count, spatial relations, and distance questions. It cannot access private measurements.

The oracle receives the frozen question and can call only direct, read-only local tools over the frozen frame:

| Tool | Evidence returned |
|---|---|
| `ground_semantic_object(query)` | Grounded objects, range, side, point support, and evidence IDs. |
| `estimate_ground_plane()` | Open3D RANSAC ground-plane fit, inliers, residual, and quality flags. |
| `measure_object_height(query)` | Visible point-cloud height above the fitted ground plane, unit, tolerance, provenance, and quality flags. |
| `measure_object_height_bucket(query)` | The same private height measurement mapped deterministically to a public height bucket. |

Additional read-only oracle tools will be added as new point-cloud and map evidence capabilities become available.

MoonDream, EdgeTAM, LiDAR projection, and Open3D run privately. These are direct Python tools exposed through LangChain schemas, not MCP or robot RPC tools. The oracle has a bounded tool-call budget and cannot use shell, Python execution, network access, or mutable robot skills.

After structural validation checks answer format and evidence citations, a separate private no-tools validator decides whether the cited evidence supports the frozen question and answer. For example, range evidence alone cannot validate a chair-height answer.

### Adding a Tool

New point-cloud or map capabilities should be added as typed local oracle tools rather than as new VQA intent pipelines. A tool must provide:

- a read-only argument schema;
- a versioned structured result;
- evidence and provenance IDs;
- units and tolerance for numeric values;
- quality flags and explicit rejection conditions.

The semantic validator receives only the frozen question, proposed answer, and cited tool output. The evaluated model never receives any of them.

## Geometry Quality Gates

Geometry quality gates are a private validation step: after the oracle grounds or measures a question, they decide whether the point-cloud evidence is reliable enough to accept it as benchmark ground truth. A failed gate rejects the question; it never becomes a public case.

Ground-plane estimation uses Open3D `segment_plane` on visible points in the lower image band. It rejects insufficient support, insufficient inliers, and high residuals.

Object height is a visible-point-cloud estimate, not a full CAD dimension. The height tool requires one unambiguous grounded mask, sufficient object point support, an accepted ground plane, and elevated points above that plane. It records its uncertainty and rejects sparse, partial, or ambiguous measurements.

## Public Answer Contracts

Public cases are either choice or numeric. Metric questions use public choices by default so an image-only model can make a visual estimate rather than reproduce an exact measurement.

| Contract | Evaluated-model final line | Scoring |
|---|---|---|
| Choice | `ANSWER: left` | Must exactly match a public allowed choice. |
| Numeric | `ANSWER: 0.86 m` | Must be finite, use the required unit, and fall within the public tolerance. |

Height questions use the fixed choices `under 0.5 m`, `0.5-1.0 m`, `1.0-1.5 m`, and `over 1.5 m`. The private oracle measures the height and maps it deterministically to one of those choices. Numeric contracts remain available for specialized measurement evaluation; their expected values, measurement provenance, and validator decisions remain private.

## Generate a Dataset

Generate a constrained five-frame sample:

```bash
OPENAI_API_KEY="$OPENAI_API_KEY" dimos vqa generate \
  --recording go2_short \
  --start-index 0 --stop-index 100 --stride 20 \
  --question-mode constrained \
  --output ~/.local/state/dimos/datasets/vqa/go2-short-constrained
```

Generate an agentic sample:

```bash
OPENAI_API_KEY="$OPENAI_API_KEY" dimos vqa generate \
  --recording go2_short \
  --start-index 0 --stop-index 100 --stride 20 \
  --question-mode agentic \
  --output ~/.local/state/dimos/datasets/vqa/go2-short-agentic
```

Generation is resumable. A directory containing `frame.json` is complete and skipped on a later run; aggregate indexes are rebuilt after all requested frames are considered.

Constrained mode uses the image-only object author automatically unless one or more explicit `--query` values are supplied. `--propose-questions` remains a compatible alias for forcing image-authored constrained questions.

## Dataset Layout

Frame directories are authoritative artifacts for images, public cases, and private evidence. Dataset-root indexes support query and analysis workloads.

```text
dataset/
  frame-000040/
    image.jpg
    original_image.jpg
    grounding_overlay.jpg
    frame.json
    cases/<case-id>/
      case.json
      image.jpg
      private/oracle.json
      private/ground_truth.json
  frames.jsonl
  public_cases.jsonl
  private_oracles.jsonl
  frames.parquet
  public_cases.parquet
  private_oracles.parquet
  ground_truth.jsonl
  manifest.json
```

`public_cases` contains only question contracts, frame identity, and relative public paths. `private_oracles` contains expected answers, tolerance provenance, validator revision, and private relative paths. Keep private indexes under the same access controls as `private/` case artifacts.

## Evaluation

Run the image-only LangChain evaluator:

```bash
OPENAI_API_KEY="$OPENAI_API_KEY" dimos vqa evaluate \
  --dataset ~/.local/state/dimos/datasets/vqa/go2-short-agentic
```

Evaluation is serial by default (`--workers 1`) so the local review viewer follows one active case at a time. `--workers N` enables bounded parallel independent cases when live sequential review is not needed.

The evaluator starts a local browser viewer showing the public case image on the left, private grounding overlay on the right, and case results in the sidebar. It writes a versioned evaluation run directory:

```text
evaluations/<model>/
  run-manifest.json
  events.jsonl
  checkpoints/
  results.json
  summary.json
```

Each terminal case result is checkpointed atomically. If interrupted, rerun with `--resume`; the run validates dataset identity and configuration before skipping completed checkpoints. `results.json` and `summary.json` are published only after the run completes.

## Requirements

Generation needs a CUDA-capable PyTorch build supported by the local GPU because EdgeTAM performs private segmentation. Agentic and image-proposed generation need `OPENAI_API_KEY`. Evaluation needs `OPENAI_API_KEY` for the configured LangChain vision model.

Open3D provides ground-plane segmentation. PyArrow writes Parquet aggregate indexes. Both are part of the DimOS dependency stack.
