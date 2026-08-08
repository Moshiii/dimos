## 1. Pin the Upstream Development Inputs

- [x] 1.1 Select one official converted R2R/VLN-CE training episode for `17DRP5sb8fy` and record its split, episode ID, scene ID, exact instruction, source revision, URLs, and digests in a reviewable manifest.
- [x] 1.2 Verify that Habitat's public `mp3d_example_scene` contains every scene and navigation asset required by the selected episode, and add a checksum-pinned fixture test for that layout.
- [x] 1.3 Pin the VLN-CE, Habitat-Sim/Lab, base-image, Python dependency, and protocol code-generation revisions used by the runtime.
- [x] 1.4 Document the upstream licenses and mark scene/episode assets as external read-only data that must not enter Git, package artifacts, or OCI layers.

## 2. Extend the Canonical Case Contract

- [x] 2.1 Add strict external-benchmark source, native-result validator, task submission guidance, and automatic preparation models to `dimos.benchmark.agent_eval.case`.
- [x] 2.2 Include every external source, task, protocol, blueprint, preparation, image, asset, and result-schema field in canonical case identity and public/private projections.
- [x] 2.3 Add compiler and preflight validation for one-episode binding, safe private references, exact instruction/digest agreement, image and asset digests, and compatible interaction/validator combinations.
- [x] 2.4 Add case-model tests that accept the pinned R2R shape and reject incomplete sources, mismatched instructions, runtime overrides, unsafe paths, malformed digests, and invalid validator bindings.
- [x] 2.5 Preserve and rerun the frozen QA and simulator-scene case-model tests to prove backward-compatible dispatch and fingerprints.

## 3. Implement Automatic Public Preparation

- [x] 3.1 Add a case-bound preparation recipe that downloads the public MP3D example, official R2R/VLN-CE episode data, and required public configuration files into content-addressed caches.
- [x] 3.2 Verify every download and extracted file against the pinned manifest, use atomic cache publication, and fail closed on missing or substituted content.
- [x] 3.3 Add an exclusive preparation lock and verified-cache fast path so concurrent or repeated runs neither corrupt nor rebuild the same inputs.
- [x] 3.4 Add an OCI image resolver/builder that produces or locates the pinned runtime image by digest through the supported local OCI engine without a separate user command.
- [x] 3.5 Add preparation tests for cold cache, warm/offline cache, checksum failure, interrupted publication, concurrent preparation, missing OCI engine, and failed image build.

## 4. Build the One-Shot VLN-CE OCI Runtime

- [x] 4.1 Add the pinned OCI build definition and lock material for the legacy VLN-CE/Habitat headless environment without installing its dependencies into the DimOS environment.
- [x] 4.2 Add an image smoke command that imports VLN-CE/Habitat, initializes headless rendering, loads `17DRP5sb8fy`, and validates the configured agent cylinder and camera.
- [x] 4.3 Implement private case loading that verifies attempt, case, dataset, split, episode, scene, instruction, runtime, protocol, and result-schema identities before reset.
- [x] 4.4 Implement the one-episode container entrypoint, signal handling, bounded shutdown, diagnostic logging, and atomic terminal-result publication to the supervisor-only mount.
- [x] 4.5 Assert through image and runtime tests that scene and episode assets are read-only mounts, the public socket mount cannot read the result mount, and no credentials or private case data appear in image layers.

## 5. Define and Conform the Public UDS Protocol

- [x] 5.1 Define the benchmark-specific Protobuf service and bidirectional stream for handshake, lifecycle readiness, coherent observations, public map, bounded planar control, submission, cancellation, acknowledgements, and public errors.
- [x] 5.2 Generate and package compatible client/server bindings while keeping goal, reference path, semantics, progress, success, and score fields absent from the public schema.
- [x] 5.3 Implement attempt, case, episode, revision, frames, encodings, capability, and control-limit negotiation before accepting actions.
- [x] 5.4 Implement monotonic observation/command correlation, bounded queues, complete-epoch publication, drop diagnostics, and rejection of stale, duplicate, malformed, or out-of-state traffic.
- [x] 5.5 Add cross-endpoint conformance tests over a real Unix-domain socket, including compatibility rejection, reconnect/closure behavior, backpressure, sequence violations, and an automated private-field leakage audit.

## 6. Implement Habitat Observations, Motion, and Native Scoring

- [x] 6.1 Configure official RGB/depth observations and camera calibration, publish pose/frames, and project the complete Habitat navmesh into a static geometry-only occupancy grid.
- [x] 6.2 Add validation proving that the public occupancy map contains traversability only and has no semantic, goal, reference-route, or progress-dependent decoration.
- [x] 6.3 Implement fixed-period integration of bounded `linear.x`, `linear.y`, and `angular.z` commands through collision-aware Habitat motion for the pinned cylinder, without any public teleport or pose reset.
- [x] 6.4 Record every accepted motion result into the trajectory used by official measures, with transform, collision, timing, and trajectory-loss tests.
- [x] 6.5 Implement exactly-once route submission as VLN-CE `STOP`, reject later commands, and distinguish submission from zero velocity, navigation completion, cancellation, and timeout.
- [x] 6.6 Finalize the official configured VLN-CE navigation error, oracle success, success, SPL, nDTW, and related measures on submission or healthy timeout without reimplementing their formulas.
- [x] 6.7 Serialize one schema-validated `vlnce-result.v1.json` atomically with correlated identities, terminal reason, trajectory identity, official metrics, and runtime provenance.
- [x] 6.8 Add real-container tests for reset, collision-aware movement, no-teleport enforcement, submit-near success, submit-far failure, healthy timeout failure, scorer failure, and result atomicity.

## 7. Add the DimOS Connection and Evaluation Blueprint

- [x] 7.1 Implement `VlnceConnection` as a typed DimOS module that owns the public gRPC client and translates protocol frames into native RGB, depth, `CameraInfo`, `PointCloud2`, odometry, TF, and latched `OccupancyGrid` streams.
- [x] 7.2 Translate native DimOS planar `Twist` output into correlated bounded protocol commands and expose readiness, begin, cancel, and submission RPCs without exposing reset or direct pose mutation.
- [x] 7.3 Add the benchmark-scoped `submit_route()` skill with its required type annotations and docstring, exactly-once behavior, submission-only acknowledgement, and no success feedback.
- [x] 7.4 Compose the case-bound Go2 evaluation blueprint from `VlnceConnection`, the existing public geometry/navigation/memory stack, submission skill, Porcelain support, and optional Rerun bridge, with no internal LLM agent or private benchmark module.
- [x] 7.5 Add the episode system/task prompt text that supplies the exact verified instruction and explains when irreversible `submit_route()` should be called.
- [x] 7.6 Add module, skill-schema, stream/remapping, transform, depth-to-point-cloud, map, control, prompt, and blueprint tests; regenerate the built-in blueprint registry if the blueprint is registered globally.

## 8. Extend the Real-Time Evaluation Supervisor

- [x] 8.1 Dispatch the external-benchmark case kind from the existing `dimos eval run` command while preserving frozen and simulator-scene dispatch.
- [x] 8.2 Implement the external attempt lifecycle: preparation, fresh output reservation, mount creation, OCI startup, gateway/reset readiness, blueprint startup, handshake, public-stream readiness, and agent dispatch.
- [x] 8.3 Start one external Pi session and standalone live CodePolicy workspace only after readiness, expose the ready DimOS Porcelain app and live memory, and retain the existing agent/authentication condition behavior.
- [x] 8.4 Arbitrate terminal submission, benchmark timeout, user interruption, runtime loss, protocol failure, agent failure, and presentation failure without treating Pi text or navigation-goal completion as success.
- [x] 8.5 Validate and retain the native result unchanged, map only official `SUCCESS` to normalized pass/fail, and distinguish completed task failure from failed/not-evaluated infrastructure.
- [x] 8.6 Implement bounded cleanup for Pi, CodePolicy, motion, bridge, blueprint, OCI container, sockets, mounts, locks, and partial startup paths while preserving primary evidence.
- [x] 8.7 Add fake-runtime supervisor tests for the complete happy path, agent failure, submission, timeout, missing/malformed/foreign result, startup failures at every phase, interruption, stubborn container, cleanup failure, and non-overwriting output.

## 9. Retain Evidence, Reporting, and Visualization

- [x] 9.1 Extend attempt manifests and lifecycle events with preparation receipts, image and asset digests, OCI invocation, public handshake, observation metadata, controls, submission, native-result reference, cleanup, and correlated revisions.
- [x] 9.2 Keep private live benchmark data out of progress, public evidence, CodePolicy results, process environments, and Rerun while retaining the terminal native artifact in private attempt evidence.
- [x] 9.3 Update text and JSON CLI output to show one-command preparation/startup progress, normalized result, all terminal official metrics, condition label, attempt ID, duration, and artifact path.
- [x] 9.4 Label every development result as the `17DRP5sb8fy` training-scene DimOS geometry condition and explicitly non-comparable with standard VLN-CE validation/test leaderboard results.
- [x] 9.5 Wire the existing headless default and optional Rerun/Rerun Web settings to the public blueprint streams, and test that viewer choices do not alter case identity, native scoring, or cleanup.

## 10. Prove the End-to-End User Contract

- [x] 10.1 Add a deterministic test-only controller outside the evaluated DimOS runtime that drives the public scene to one official successful STOP and one official failing STOP, verifying the native score paths.
- [x] 10.2 Run the checked-in case through the normal configured DimOS agent and verify a healthy completed official result, inspectable evidence, and complete teardown regardless of task pass or fail.
- [x] 10.3 Add a self-hosted integration test that starts from verified caches and runs the exact documented `dimos eval run <case> --output=<path>` command without any prior simulator, container, or blueprint startup.
- [x] 10.4 Run the focused unit/integration suites, blueprint registry check, formatting, lint, and type checking for all changed packages.
- [x] 10.5 Document the one-command development run, first-run preparation behavior, cache/offline behavior, optional visualization, result interpretation, license boundary, and later read-only mounting of user-authorized full MP3D splits.
