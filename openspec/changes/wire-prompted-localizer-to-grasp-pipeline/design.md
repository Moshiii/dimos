## Context

Two working paths exist today:

1. `dimos/perception/memory/tool_localize.py` searches a Memory2 recording with CLIP, prompts Moondream for detections, segments with EdgeTAM, projects aligned depth into the world frame, and selects the detection whose reconstructed cloud has the most points.
2. The manipulation pipeline accepts a `PointCloud2`, generates GraspGenX proposals, filters them with collision-aware IK, plans the reachable candidates, selects a grasp, visualizes progress, and executes the selected plan.

The missing piece is an in-memory runtime boundary between these paths. The integration must remain small: perception owns segmentation, manipulation owns grasping and motion, and the localized target cloud is not automatically promoted into the planner's collision scene.

The public entry point remains `PickAndPlaceModule.pick(object_name)`. Existing deployments resolve `object_name` through `ObjectSceneRegistration`; the new xArm6 deployment instead resolves it through prompted localization. The implementation must preserve those explicit blueprint choices without silently trying another provider.

## Goals / Non-Goals

**Goals:**

- Make prompted localization callable as a typed internal RPC from a dedicated worker.
- Share one localization core between the runtime module and `tool_localize.py`.
- Feed the localized world-frame `PointCloud2` directly into the existing grasp pipeline in memory.
- Preserve the existing proposal, IK-first evaluation, planning, selection, visualization, and execution behavior.
- Provide a real xArm6 blueprint with recording, localization, manipulation, visualization, gripper control, and direct MCP access.
- Fail before grasp generation and motion if localization cannot produce a target cloud.
- Keep the behavior testable at core, RPC, orchestration, blueprint, and recorded-data smoke-test levels.

**Non-Goals:**

- Moving the robot or camera to pre-scan the scene.
- Adding provider fallback, caching, stable object identities, localization confidence, or a generalized localization result model.
- Treating the target cloud or other perceived clouds as collision geometry.
- Changing how candidates are ranked, planned, selected, visualized, or executed.
- Adding a new localization-facing agent skill or an LLM agent to the combined blueprint.
- Changing the existing `xarm6-worldbelief` or object-scene-based manipulation blueprints.

## Decisions

### 1. Extract a reusable prompted-localization core

Factor the localization sequence out of `tool_localize.py` into reusable code that accepts a text query, recording/stream configuration, and time window and returns the same best reconstructed result used today. The command-line tool and runtime module call this core, so debug behavior does not drift from robot behavior.

The runtime-facing method returns only `PointCloud2 | None`. Debug-only artifacts such as Rerun recordings or PLY/PCD files remain concerns of the standalone tool and are not part of the runtime handoff.

Alternative considered: invoke `tool_localize.py` as a subprocess and exchange files. This was rejected because it adds serialization, temporary-file lifecycle, and error-reporting complexity to a handoff that already has a native message type.

### 2. Use a narrow typed RPC on a dedicated worker

Introduce:

```python
class PromptedObjectLocalizerSpec(Spec, Protocol):
    def localize(self, query: str) -> PointCloud2 | None: ...
```

`PromptedObjectLocalizerModule` implements this RPC and owns the heavy CLIP, Moondream, and EdgeTAM runtime state. It reads the configured active Memory2 recording and searches the latest 30-second window on every call. It preserves “most reconstructed 3D points wins” when multiple detections are available.

A dedicated worker isolates model memory and initialization from manipulation control. A fresh query on every pick avoids introducing cache invalidation or object-lifetime semantics.

Alternative considered: expose localization as another `@skill`. This was rejected because localization is an internal step of the user-facing pick transaction, not an independently required agent capability.

### 3. Keep target-source selection explicit in module composition

Refactor only the shared portion of pick execution—proposal through execution—so it can consume an already resolved target cloud. The existing `PickAndPlaceModule` path continues to resolve targets through `ObjectSceneRegistration`. A prompted-pick module variant wires `PromptedObjectLocalizerSpec`, resolves the text query to a cloud, and then calls the same shared pipeline.

This module-level seam avoids optional RPC dependencies and makes the selected provider visible in the blueprint. There is no runtime chain that tries prompted localization after object-scene lookup, or vice versa.

Alternative considered: introduce a generalized target-provider/result abstraction. This was rejected for the MVP because the two sources already have different identity and collision semantics, and a new common object model would add policy that the integration does not need.

### 4. Preserve the target cloud's limited role

The prompted cloud is used for:

- `GraspProposalInput.from_pointcloud(...)`;
- the existing target point-cloud visualization; and
- naming/status context for the pick transaction.

It is not registered as an obstacle. Because it has no `DetObject` identity, prompted picking does not perform object-ID lookup or target-obstacle suppression. Existing static and already configured planning obstacles remain unchanged.

This preserves the agreed boundary between visualization/perception data and collision-authoritative data until scene integration is designed separately.

### 5. Abort atomically on localization failure

If `localize(object_name)` returns `None` or an unusable empty cloud, `pick` returns a clear failure result before calling GraspGenX, RoboPlan, the arm controller, or the gripper. Model/runtime exceptions are reported as localization failure rather than triggering another provider.

Once a valid cloud is returned, downstream failure and visualization behavior remain the behavior of the existing grasp pipeline.

### 6. Add one direct-test xArm6 blueprint

Add a real-hardware xArm6 blueprint, provisionally named `xarm6-prompted-pick`, containing:

- the camera and active Memory2 recorder needed by prompted localization;
- `PromptedObjectLocalizerModule` in a dedicated worker;
- the prompted pick-and-place module variant;
- GraspGenX and the existing motion/control coordinator;
- xArm6 hardware with `gripper=True`;
- `make_xarm6_model_config(add_gripper=True)`;
- shared `XARM_GRIPPER_SWEEP` and `XARM_GRASP_FRAME_TO_TCP`;
- manipulation visualization; and
- `McpServer`, without `McpClient` or an LLM agent.

After launch, the integration is exercised directly with:

```bash
dimos mcp call pick --arg object_name="white and red marker"
```

The existing `xarm6-worldbelief` blueprint remains a perception/debug stack rather than being repurposed.

### 7. Support plan-only pick validation

Add `execute_pick: bool = True` to `PickAndPlaceModuleConfig`. Candidate generation and evaluation remain unchanged. After `_select_feasible_grasp` has found and visualized a fully planned candidate, the shared pick path checks the flag:

- `True` preserves the current arm and gripper execution sequence.
- `False` returns a successful plan-only result with the selected candidate metadata and does not call safety-lift execution, gripper commands, trajectory execution, or update `_last_pick_pose`.

The check belongs in the shared post-selection path so both object-scene and prompted target providers have identical test behavior. Keeping execution enabled by default preserves existing deployments; the prompted xArm6 blueprint can disable it through a normal module config override.

Because `pick_and_place` delegates its first phase to `pick`, it must detect the plan-only success metadata and return immediately rather than executing the place phase. Other independently invoked motion skills remain outside this pick-pipeline flag.

Alternative considered: add a separate dry-run skill or stop before candidate planning. A separate skill would duplicate orchestration, while stopping before planning would not validate the motion-planning integration this mode is intended to test.

## Risks / Trade-offs

- **[Localization latency is paid on every pick]** → Keep the window fixed at 30 seconds for this MVP and isolate inference in its own worker; add caching only after measurements justify it.
- **[The newest recording window may contain weak views]** → Preserve the current multi-frame retrieval and largest-3D-cloud selection; defer active scanning to a separate feature.
- **[Largest point count is not always the best object instance]** → Keep behavior aligned with the existing debug tool so failures are reproducible; improve the perception oracle independently.
- **[The target is absent from collision authority]** → Continue using existing configured obstacles and make the boundary explicit in tests and documentation; do not imply that a visualized cloud is collision checked.
- **[Concurrent reads of an active recording may see incomplete recent data]** → Use the recording API's supported read behavior and treat frames that cannot be reconstructed as unavailable candidates.
- **[Prompted and object-scene pick paths could drift]** → Share the proposal-to-execution implementation and test that both paths invoke it without provider fallback.
- **[Large model dependencies can make a default blueprint expensive]** → Add a separate blueprint and dedicated worker rather than modifying existing xArm6 stacks.
- **[A successful plan-only result could be mistaken for a physical pick]** → Include explicit `planning_only=true` metadata and “no motion executed” wording, and do not update held-object state.

## Migration Plan

1. Extract the localization core while retaining `tool_localize.py` outputs and selection behavior.
2. Add the localizer Spec/module and unit tests.
3. Refactor the common point-cloud-to-pick path without changing existing object-scene behavior.
4. Add the prompted module variant and xArm6 blueprint.
5. Regenerate the built-in blueprint registry and document the launch/MCP smoke-test command.
6. Run unit, blueprint, and recorded-data smoke tests before hardware validation.

Rollback consists of removing the new blueprint and prompted module variant. Existing blueprints and the standalone localization tool remain usable throughout.

## Open Questions

None for the MVP. Pre-scan motion, collision-scene ingestion, and richer localization results are deliberately deferred.
