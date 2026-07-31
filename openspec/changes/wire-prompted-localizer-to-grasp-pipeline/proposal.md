## Why

The grasp pipeline can plan and execute a pick from an object point cloud, while the prompted scene-perception workflow can produce that point cloud, but the two currently require manual handoff. A minimal runtime integration is needed so a user can request an object by name and run localization, grasp generation, planning, visualization, and execution through one `pick` skill call.

## What Changes

- Add a prompted-object localizer module that exposes a narrow internal RPC returning the best segmented `PointCloud2` for a text query from the latest 30 seconds of an active Memory2 recording.
- Reuse the localization core used by the standalone `tool_localize.py` debug workflow; keep the existing “most reconstructed 3D points wins” selection behavior.
- Allow `PickAndPlaceModule.pick(object_name)` to use the explicitly wired prompted localizer as its target source and pass the returned cloud directly into the existing grasp proposal, reachability, planning, visualization, and execution pipeline.
- Stop the pick before grasp generation or robot motion when localization returns no object cloud.
- Add a plan-only configuration flag that runs localization, grasp generation, reachability checks, planning, selection, and visualization but skips arm and gripper execution.
- Add a real xArm6 blueprint that wires camera recording, prompted localization, GraspGenX, motion control, gripper support, manipulation visualization, and `McpServer` for direct skill invocation.
- Keep existing object-scene-based manipulation blueprints unchanged. Provider choice remains explicit at blueprint construction; there is no automatic localization fallback, pre-scan motion, new collision-scene ingestion, or LLM agent in this change.

## Capabilities

### New Capabilities

- `prompted-object-picking`: Text-prompted object localization and its minimal integration with the existing grasp-planning and execution skill.

### Modified Capabilities

None.

## Impact

- Affects prompted localization code under `dimos/perception/memory/`, manipulation orchestration under `dimos/manipulation/`, and xArm6 blueprint composition.
- Introduces an internal `PromptedObjectLocalizerSpec` RPC boundary returning `PointCloud2 | None`.
- Reuses the current Memory2, CLIP, Moondream, EdgeTAM, depth projection, GraspGenX, RoboPlan, xArm6 gripper, and manipulation visualization dependencies.
- Adds a directly testable MCP entry point such as `dimos mcp call pick --arg object_name="white and red marker"` when the new blueprint is running.
- Does not change the public `pick` skill signature or the collision semantics of existing manipulation blueprints.
