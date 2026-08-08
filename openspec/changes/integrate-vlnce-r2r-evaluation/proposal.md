## Why

DimOS needs a recognized semantic navigation benchmark that runs through the existing `dimos eval run <case>` contract and measures an agent acting through the normal navigation stack. VLN-CE R2R provides official language-following episodes and scoring, while its public `17DRP5sb8fy` scene and matching R2R training episodes let us build and verify the integration without waiting for full Matterport3D access.

## What Changes

- Add a case-bound VLN-CE R2R source that pins one official episode, scene, dataset revision, runtime image, and digests; one case always runs one episode.
- Run a fresh, pinned OCI benchmark container per attempt. The container owns Habitat reset, private episode data, timeout, reference trajectory, official scoring, and a terminal result artifact.
- Add a benchmark-specific DimOS connection module and Protobuf/gRPC bidirectional stream over a Unix-domain socket for public RGB, depth-derived geometry, odometry, an inspectable geometry-only occupancy map, planar velocity commands, and explicit route submission.
- Expose a documented `submit_route()` agent skill that maps only to the benchmark's terminal `STOP`; ordinary navigation completion and zero velocity do not submit an episode.
- Keep all semantic, goal, progress, reference-path, success, and scoring oracles outside the DimOS runtime. Preserve every official VLN-CE metric unchanged and map only official `SUCCESS` to normalized pass or fail.
- Add a non-gated development case using the public `17DRP5sb8fy` asset and a matching official R2R training episode, plus deterministic harness verification and a real-agent end-to-end smoke.
- Support headless execution and optional Rerun visualization without changing case identity or evaluation semantics.
- Document that the public geometry condition and training-scene smoke result are not directly comparable with the standard VLN-CE leaderboard. Full validation splits remain optional user-provided data under Matterport's terms.

## Capabilities

### New Capabilities

- `vlnce-r2r-evaluation-backend`: Defines the one-shot VLN-CE R2R container, public DimOS bridge, benchmark-owned privacy and lifecycle boundary, official scoring, public development case, and verification behavior.

### Modified Capabilities

- `agent-evaluation-case-model`: Adds a strict case shape for one external benchmark episode whose OCI runtime and benchmark-native validator are selected by the case rather than a CLI runtime option.
- `realtime-simulator-agent-evaluation`: Extends canonical `dimos eval run` dispatch, lifecycle, evidence, outcomes, and visualization to case-bound one-shot external benchmark runtimes with terminal agent submission and benchmark-native scoring.

## Impact

- Affects the generic agent-evaluation case models, compiler, preflight, dispatch, attempt lifecycle, evidence manifest, CLI rendering, and outcome normalization.
- Adds a VLN-CE benchmark package containing an OCI build definition, pinned upstream metadata, episode/asset preparation tooling, Protobuf schema, container gateway, DimOS connection module, benchmark-only submission skill, blueprint, case data, and tests.
- Adds build/runtime dependencies on an OCI engine, gRPC/Protobuf, the pinned VLN-CE/Habitat environment inside the image, and GPU-capable headless rendering where required. Benchmark assets remain external mounts and are never embedded in the image or repository.
- Preserves existing frozen QA, PiMSim/DimSim real-time cases, and the global viewer configuration.
