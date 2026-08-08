## Context

The generic evaluator already models a case as source, task, interaction, and private validator contracts and dispatches both frozen QA and live simulator cases through `dimos eval run <case>`. Live cases currently assume that DimOS directly starts a compatible simulator provider and polls a private goal predicate. VLN-CE has an old, tightly pinned Python and Habitat environment, owns richer stateful metrics, and must remain the sole authority for episode truth. Installing it into the DimOS environment or translating its score into a DimOS checker would weaken both reproducibility and benchmark fidelity.

R2R uses Matterport3D scenes. Full validation requires licensed assets, but Habitat publishes the complete `17DRP5sb8fy` MP3D example and official R2R data contains matching training episodes. That overlap supports a real, non-gated development evaluation. It proves the full integration but does not produce a leaderboard-comparable result.

The trust boundary is strict: the evaluated agent can inspect the entire DimOS runtime. Therefore, no goal position, reference path, semantic annotation, progress measure, or score may enter the DimOS process tree. The benchmark container owns those values and exports only a terminal result after the episode ends.

## Goals / Non-Goals

**Goals:**

- Make one command compile the case, prepare pinned public dependencies when absent, start every runtime, execute one R2R episode, collect official metrics, clean up, and report the result:

  ```bash
  uv run dimos eval run \
    dimos/benchmark/vlnce_r2r/cases/17DRP5sb8fy-smoke/case.json \
    --output=/tmp/dimos-vlnce-r2r-smoke
  ```

- Exercise the configured DimOS agent through the normal Go2 perception, mapping, navigation, and CodePolicy/Porcelain evaluation path.
- Give the agent official RGB, depth-derived public geometry, odometry, and a complete geometry-only navigation map while requiring it to infer every semantic fact from observations.
- Preserve the official VLN-CE episode, STOP semantics, and native metric implementations.
- Keep benchmark-private state outside DimOS and retain enough evidence to audit lifecycle, public observations, actions, submission, native metrics, and cleanup.
- Run headlessly by default and honor the existing optional Rerun viewer settings.

**Non-Goals:**

- Reproduce the published VLN-CE sensor, action, or leaderboard condition exactly. The public navmesh map and planar velocity surface define a declared DimOS geometry condition.
- Evaluate mapping quality, Go2 locomotion physics, or factual QA.
- Put Habitat, VLN-CE, or their legacy dependencies into the main DimOS Python environment.
- Add generic external-worker infrastructure or change the existing backend-neutral simulator runtime protocol.
- Redistribute Matterport assets, hide their license terms, or make full validation assets an implementation prerequisite.
- Run a suite from one case. A scheduler may later run many independent case files.

## Decisions

### 1. Extend the canonical case instead of adding another command

An external benchmark case adds an `external_benchmark_episode` source and a benchmark-native result validator. The source pins the benchmark, upstream revisions, OCI image recipe and digest, episode dataset identity and digest, split, episode ID, scene ID, public asset preparation recipe, blueprint, protocol revision, and expected result schema. The separate task contract contains the copied public instruction and its digest. The interaction retains the existing bounded live Pi and standalone CodePolicy model.

The single-case CLI accepts no benchmark or runtime selector. Case identity determines all runtime choices; agent condition, credentials, output location, and viewer remain attempt settings. This preserves the source/task/interaction/validator pattern and prevents a command-line option from silently changing what is evaluated.

Alternative considered: add `dimos vlnce eval`. Rejected because it would duplicate attempt ownership, evidence, agent configuration, outcome semantics, and visualization behavior.

### 2. Make `dimos eval run` the complete local lifecycle owner

The command performs these phases:

1. Compile and verify the case and its private references.
2. Under a preparation lock, fetch checksum-pinned public development assets and build or resolve the digest-pinned OCI image when the cache is absent.
3. Reserve a fresh attempt directory and create private case, public socket, and terminal-output mounts.
4. Start one benchmark container and wait for its reset and public gateway readiness.
5. Build the case-bound DimOS blueprint containing `VlnceConnection`, the geometry/navigation stack, benchmark submission skill, and no internal LLM agent.
6. Wait for coherent initial observations, map, odometry, motion control, live Memory2, and Porcelain readiness.
7. Start one external Pi session with standalone live CodePolicy, deliver the verified instruction and submission guidance, and begin the episode deadline.
8. On submission, timeout, interruption, or infrastructure failure, stop agent actions and motion, collect the atomic native result when applicable, stop all owned processes, validate artifacts, and print the normalized outcome.

Preparation artifacts are content-addressed and reusable; attempts remain fresh and non-overwriting. An offline cache miss or unavailable OCI/GPU prerequisite fails before the agent starts with a concrete preparation error. No manual `podman run`, simulator startup, blueprint startup, or data placement command is part of normal use.

Alternative considered: require separate `prepare`, `podman run`, and `dimos run` commands. Rejected because the requested product contract is a self-contained evaluation command and split ownership makes cleanup and evidence correlation unreliable.

### 3. Isolate VLN-CE in one OCI container per episode

The repository provides a reproducible OCI build pinned to the selected VLN-CE commit, Habitat-Sim/Lab 0.1.7-compatible environment, base image digest, Python dependencies, gateway source, and generated Protobuf code. Scene and episode assets are read-only runtime mounts, not image layers. The final image digest and mounted asset digests enter attempt evidence.

The container receives the complete private episode binding at launch, resets Habitat, verifies every pinned identity and the copied instruction, runs exactly one episode, owns the benchmark deadline and official measures, writes one atomic result to a supervisor-only mount, and exits. It never serves a private simulator or scoring API.

Alternative considered: a second Python virtual environment launched as an external worker. Rejected because Habitat's native dependencies are harder to reproduce and isolate, while an OCI image works with Docker or Podman and keeps the legacy stack out of DimOS.

### 4. Use one benchmark-specific public gRPC stream over a Unix-domain socket

`VlnceConnection` is a DimOS module inside the blueprint. It connects as a client to a Protobuf/gRPC bidirectional stream exposed on an attempt-private Unix-domain socket. A versioned handshake binds attempt, case, episode, protocol revision, coordinate frames, observation encodings, control limits, and capabilities before any action is accepted. Monotonic sequence numbers correlate coherent observation epochs and commands; stale, duplicate, malformed, or incompatible messages fail the attempt.

The public container-to-DimOS direction carries only:

- timestamped RGB and depth frames with camera calibration;
- benchmark-agent pose as odometry and declared frame transforms;
- one static, geometry-only occupancy grid projected from the complete Habitat navmesh;
- public lifecycle status needed for readiness and terminal transport closure.

The DimOS-to-container direction carries only bounded planar velocity commands, lifecycle begin/cancel signals, and one terminal route submission. Large observation payloads use typed byte fields and declared encodings rather than JSON arrays. No goal, path, semantics, metric, or progress field exists in the public schema. The socket and terminal result mount have separate permissions; DimOS cannot read the result mount.

Alternative considered: TCP. Rejected because the two process trees share one host and need no network exposure. Alternative considered: LCM across the container boundary. Rejected because a benchmark-specific, request-correlated compatibility handshake and isolated per-attempt endpoint are clearer here.

### 5. Adapt public observations into native DimOS streams

The connection publishes RGB and camera information, depth and camera information, depth projected into camera-frustum `PointCloud2`, odometry/TF, and a latched `OccupancyGrid`. Existing DimOS modules consume these streams without importing benchmark types. The complete navmesh-derived grid is deliberately public to provide the minimal collision-free point-to-point prior required by the current navigation stack. It contains traversability only: no room labels, object labels, goal marker, reference route, or progress overlay.

The benchmark's coordinate conversion is explicit and tested at the bridge. Rerun observes only the same public streams and agent actions available to DimOS.

Alternative considered: synthesize 2-D lidar. Rejected because VLN-CE does not define a lidar sensor and depth-to-point-cloud conversion preserves an official observation. Alternative considered: let DimOS reconstruct the global map. Rejected because mapping quality is not the capability under evaluation.

### 6. Preserve the declared embodiment while using the DimOS control surface

Habitat retains VLN-CE's 1.5 m-high, 0.1 m-radius cylinder and 1.25 m camera height. The bridge accepts bounded `Twist.linear.x`, `Twist.linear.y`, and `Twist.angular.z`, integrates them at a fixed declared control period, and applies collision-aware Habitat pathfinder motion. It never exposes teleport or pose reset to the agent. Every accepted pose is appended to the trajectory consumed by the official measures.

This differs from VLN-CE's standard discrete action set and is recorded as the DimOS geometry condition. The integration must not report its results as directly comparable with the standard leaderboard.

Alternative considered: translate every velocity request into the nearest discrete VLN action. Rejected because quantization would bypass the existing controller and make its behavior difficult to reason about.

### 7. Make terminal submission explicit and irreversible

The blueprint includes a benchmark-scoped `submit_route()` skill/RPC. Its required docstring and the episode prompt state that calling it means “I have finished the route; submit VLN-CE STOP and end this evaluation.” It accepts no goal or score arguments, may succeed at most once, returns only submission acknowledgement, and reveals no metric or success feedback. Ordinary navigation goal completion, zero velocity, Pi text, and motion cancellation never imply STOP.

Alternative considered: automatically stop when a DimOS navigation goal completes. Rejected because the agent may use several waypoints and R2R evaluates its decision about where the instruction ends.

### 8. Treat the official terminal artifact as the score authority

The container writes `vlnce-result.v1.json` atomically after official measure finalization. It binds attempt, case, episode, scene, runtime/protocol revisions, terminal reason, trajectory identity, and every native metric. The supervisor validates identity, schema, digest, and single-write behavior, retains the artifact unchanged, and maps only official `SUCCESS` to `task_result`.

A submitted or timed-out healthy episode is `attempt_status=completed`, even when `SUCCESS=0`. Missing, malformed, inconsistent, or unavailable native results after a path that requires scoring produce `attempt_status=failed` and `task_result=not_evaluated`. The CLI never recomputes navigation error, oracle success, SPL, or nDTW.

### 9. Verify the harness independently from agent capability

Tests use three layers:

- Contract and protocol tests reject invalid cases, private-field leakage, incompatible handshakes, bad sequences, illegal commands, instruction mismatches, and malformed results.
- A fake OCI runtime tests complete CLI startup, readiness, timeout, cancellation, cleanup, and evidence behavior without Habitat.
- The real public-scene gate downloads the official `17DRP5sb8fy` asset, selects a matching official R2R/VLN-CE training episode, and runs both a deterministic test-only controller that proves pass/reject scoring paths and a normal DimOS-agent attempt.

The deterministic controller may use benchmark truth only inside a dedicated verification path outside the evaluated DimOS runtime. The real agent is not required to pass; acceptance requires a healthy completed episode with a valid official result, inspectable public evidence, and clean teardown.

## Risks / Trade-offs

- **Legacy Habitat builds may fail on current drivers or GPUs** → Pin the full image, add a headless renderer smoke during preparation, and fail before agent startup with captured diagnostics.
- **First-run image and asset preparation is slow or requires network access** → Cache by digest under a lock, show explicit preparation progress, and document an offline cache check while preserving the one-command interface.
- **A public full navmesh materially changes the benchmark condition** → Name the condition in the case and result, retain official metrics without claiming leaderboard comparability, and never add semantic or goal overlays.
- **Velocity integration changes standard R2R dynamics** → Pin control period, limits, collision method, and embodiment metadata; cover transforms and trajectory recording with deterministic tests.
- **The UDS stream can back up on image traffic** → Bound queues, prefer the newest complete observation epoch, report dropped public frames, and treat command or trajectory loss as infrastructure failure.
- **Agent-readable DimOS code could reveal accidental private fields** → Generate a public-only Protobuf schema, use separate mounts and permissions, test the runtime process environment and filesystem view, and prohibit score transport over the socket.
- **The public example scene or upstream download URLs may change** → Pin checksums and upstream revisions; fail closed on mismatch rather than silently accepting replacement data.
- **A real LLM run can be flaky** → Gate integration correctness on deterministic harness success and valid real-agent completion, while retaining the real task result as measured evidence.

## Migration Plan

1. Add the new case and result models without changing existing case tags or drivers.
2. Add fake-runtime lifecycle and evidence tests, then wire external dispatch behind the new case kind.
3. Add the pinned OCI build, public asset preparation, protocol, connection module, blueprint, and submission skill.
4. Land the public single-scene case and real integration gates.
5. Keep frozen QA and PiMSim/DimSim tests as regression gates throughout.

Rollback removes the new case kind and benchmark package; existing cases and commands remain unchanged because dispatch is type-based. Cached images and assets are regenerable data and may be removed separately by the existing cache policy.

## Open Questions

No product decision blocks implementation. During implementation, the preparation manifest will select and pin one exact official converted R2R episode for `17DRP5sb8fy`; its IDs, instruction, source URL, and digests become immutable case data before the end-to-end gate is accepted.
