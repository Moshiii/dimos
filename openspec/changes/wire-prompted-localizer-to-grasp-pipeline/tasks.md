## 1. Reusable Prompted Localization

- [x] 1.1 Extract the CLIP retrieval, Moondream detection, EdgeTAM segmentation, depth projection, and largest-3D-cloud selection flow from `tool_localize.py` into a reusable in-memory localization core.
- [x] 1.2 Update `tool_localize.py` to call the extracted core while preserving its Rerun and optional point-cloud file outputs as debug-only behavior.
- [x] 1.3 Add `PromptedObjectLocalizerSpec` and a configured `PromptedObjectLocalizerModule` that loads its inference runtime in a dedicated worker and queries the latest 30 seconds of the active Memory2 recording on every RPC call.
- [x] 1.4 Add focused tests for latest-window selection, largest reconstructed cloud selection, no-result behavior, and equivalence between runtime and debug-tool core processing.

## 2. Prompted Target Pick Orchestration

- [x] 2.1 Refactor the existing pick implementation so the proposal-through-execution path can consume an already resolved `PointCloud2` while preserving the current object-scene path.
- [x] 2.2 Add the explicitly wired prompted pick-and-place module variant that calls `PromptedObjectLocalizerSpec.localize(object_name)` and passes a valid cloud directly to the shared grasp pipeline.
- [x] 2.3 Implement early localization-failure handling that returns a clear pick failure before GraspGenX, planning, arm, or gripper calls and never falls back to another provider.
- [x] 2.4 Preserve the prompted cloud as grasp input and visualization only, with no object-ID lookup, target suppression, pre-scan motion, or collision-scene registration.
- [x] 2.5 Add orchestration tests covering successful cloud handoff, unchanged candidate evaluation, visualization publication, no-motion failure paths, explicit provider selection, and unchanged object-scene picking behavior.

## 3. xArm6 Runtime Composition

- [x] 3.1 Add the real `xarm6-prompted-pick` blueprint with camera recording, active Memory2 storage, the dedicated localizer worker, prompted pick orchestration, GraspGenX, motion control, manipulation visualization, and `McpServer` without an LLM agent.
- [x] 3.2 Configure the blueprint with `xarm6_hardware(gripper=True)`, `make_xarm6_model_config(add_gripper=True)`, `XARM_GRIPPER_SWEEP`, and `XARM_GRASP_FRAME_TO_TCP`.
- [x] 3.3 Add blueprint structure/wiring tests that verify RPC injection, worker placement, skill exposure, gripper configuration, absence of provider fallback and agent modules, and no changes to `xarm6-worldbelief`.
- [x] 3.4 Regenerate the built-in blueprint registry and verify the generated file is current.

## 4. End-to-End Validation and Usage

- [x] 4.1 Add a recorded-data smoke test that localizes a known prompted object into a non-empty world-frame cloud and feeds it into the grasp pipeline without a file handoff.
- [x] 4.2 Run the focused perception, manipulation, blueprint-generation, formatting, and type-check suites appropriate to the touched code.
- [x] 4.3 Document how to launch `xarm6-prompted-pick`, allow the recording window to populate, inspect manipulation visualization, and invoke `dimos mcp call pick --arg object_name="white and red marker"`.
- [x] 4.4 Add and test an `execute_pick` configuration flag that returns a clearly marked plan-only success after final candidate planning without arm or gripper execution.
- [ ] 4.5 Perform a guarded real-hardware smoke test confirming localization occurs before any motion and the existing grasp evaluation visualization appears during the pick.
