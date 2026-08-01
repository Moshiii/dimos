## 1. Proposal Provider Contract

- [x] 1.1 Add a provider-neutral proposal input containing the segmented cloud and latest detected center and size
- [x] 1.2 Change `GraspGenSpec` and GraspGenX to accept the common proposal input without changing learned inference output
- [x] 1.3 Add a lightweight heuristic `GraspGenSpec` module that preserves the current single-candidate pose calculation and score
- [x] 1.4 Add unit tests for heuristic occlusion, tall-object, distance-orientation, frame, timestamp, and score behavior

## 2. Provider-Only Pick Pipeline

- [x] 2.1 Make the `PickAndPlaceModule` proposal provider dependency required and remove `heuristic_grasp_fallback`
- [x] 2.2 Remove the internal heuristic proposal branch and provider-source transaction/result fields
- [x] 2.3 Build the common proposal input from the validated object cloud and selected detection geometry
- [x] 2.4 Migrate pipeline tests, demos, and fake providers to the new proposal request
- [x] 2.5 Add blueprint-injection tests for missing, unique, and ambiguous proposal providers

## 3. Shared Grasp Visualization Builders

- [x] 3.1 Add backend-neutral builders for `grasp/object-cloud` and state-colored `grasp/proposals` layers
- [x] 3.2 Move reusable gripper wireframe construction out of demo-only code without importing Viser into the builders
- [x] 3.3 Add immutable candidate visual states and verify pending gray, current yellow, rejected red, and selected green geometry
- [x] 3.4 Migrate the standalone banana visualization demo and its tests to the shared builders

## 4. Live Pipeline Visualization

- [x] 4.1 Add optional grasp visualization geometry to `PickAndPlaceModuleConfig`
- [x] 4.2 Publish the validated object cloud directly through `self._world_monitor.visualization`
- [x] 4.3 Publish pending, current, and rejected candidate replacements during ranked connected-plan evaluation
- [x] 4.4 Replace the proposal layer with only the green selected candidate before execution, or retain all-red evaluated candidates on exhaustion
- [x] 4.5 Contain layer construction and submission failures so they cannot change or delay pick behavior
- [x] 4.6 Add fake-visualizer tests for disabled visualization, state transitions, selected-only replacement, all-failed retention, stable layer IDs, and failure isolation

## 5. xArm Blueprint Composition

- [x] 5.1 Factor provider-neutral real and simulation perception cores without changing their camera, registration, planning, or execution wiring
- [x] 5.2 Compose the heuristic provider into `xarm_perception` and `xarm_perception_sim`
- [x] 5.3 Compose only GraspGenX into `xarm_graspgenx` and switch its manipulation visualization backend from Meshcat to Viser
- [x] 5.4 Supply heuristic, learned, and visualization consumers from the shared xArm grasp geometry source
- [x] 5.5 Update worker counts, blueprint assertions, and the generated blueprint registry when required

## 6. Verification and Documentation

- [x] 6.1 Run focused provider, pick-pipeline, visualization, and xArm blueprint tests
- [x] 6.2 Run formatting, lint, type, import-safety, and OpenSpec validation checks for changed files
- [x] 6.3 Run the banana Viser demo and a mocked end-to-end pick pipeline to inspect final layer behavior
- [x] 6.4 Update manipulation documentation for explicit provider selection and automatic grasp visualization
- [ ] 6.5 Confirm the active grasp-pipeline capability is synchronized before archiving this dependent change
