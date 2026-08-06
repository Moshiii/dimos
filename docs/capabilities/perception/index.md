---
title: "Perception"
---

## Detections

See [Single-Frame Perception VQA](/docs/capabilities/perception/single-frame-vqa.md) for the recording-to-dataset pipeline that proposes image questions, grounds answers with private 3D perception, and evaluates image-only answers.

See [Single-Frame VQA Architecture](/docs/capabilities/perception/single-frame-vqa-architecture.md) for its data flow, source layout, deterministic question execution, and dataset records.

## Experimental WorldBelief

The experimental xArm6 WorldBelief stack records RGB-D observations and processes
them on demand to maintain object identities across scans and process restarts.
Its implementation lives in
`dimos/experimental/world_belief/` while the shared detector,
embedding, and 3D object primitives remain in their standard packages.

Run the hardware blueprint:

```bash
dimos --xarm6-ip <ROBOT_IP> run xarm6-worldbelief
```

Then request a scan or recall from another terminal:

```bash
dimos mcp call scan -a prompt='["mug", "coke can"]'
dimos mcp call recall -a text="mug"
```

This stack is experimental and may change without compatibility guarantees.
