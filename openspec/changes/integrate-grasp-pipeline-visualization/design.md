## Context

`PickAndPlaceModule` already owns the selected detection, segmented object cloud, ranked candidates, connected feasibility checks, and physical pick transaction. It also inherits access to the initialized manipulation visualization through `self._world_monitor.visualization`, whose `VisualizationSpec` supports backend-neutral display-only layers.

The standalone grasp visualization demo already builds `grasp/object-cloud` and `grasp/proposals` layers, but that construction lives in demo code and displays only a static ranking. The live pick path does not publish those layers. The learned xArm blueprint also selects Meshcat, whose adapter intentionally ignores generic display-only layers.

The pick path currently contains a second concern: when no `GraspGenSpec` is injected, it may construct one heuristic candidate internally. That runtime fallback makes the proposal algorithm depend on configuration inside the orchestrator. The existing heuristic consumes the latest `DetObject.center` and `size`; these values cannot be reconstructed exactly from the registered point cloud because that cloud may accumulate observations while detection geometry remains current.

This change follows the active `add-grasp-pipeline-skill` and `validate-connected-grasp-sequences` changes. Their capability must be synchronized before this delta is archived.

## Goals / Non-Goals

**Goals:**

- Show the actual segmented cloud and grasp candidates automatically while `pick` evaluates and executes them.
- Make the current, rejected, pending, and selected candidate states visually distinct.
- Keep visualization display-only, asynchronous, backend-neutral, and unable to affect pick results.
- Make heuristic and learned proposal generation explicit, interchangeable `GraspGenSpec` providers.
- Preserve the current heuristic algorithm and the existing connected-plan feasibility gate.
- Require exactly one proposal provider in every `PickAndPlaceModule` blueprint.

**Non-Goals:**

- Add collision authority to visualization layers.
- Block planning until an intermediate visualization frame renders.
- Visualize individual pre-grasp, grasp, or retreat path segments.
- Add candidate interaction, manual selection, or editing in Viser.
- Change GraspGenX inference, scoring, or the heuristic grasp formula.
- Add new perception or scene fixtures.

## Decisions

### 1. Publish directly through the existing `VisualizationSpec`

`PickAndPlaceModule` will read `self._world_monitor.visualization` and call `set_layer` directly when a backend exists. A small private publishing helper will catch layer-construction and submission errors so visualization cannot fail `pick`.

Alternative: add a presenter service or forwarding methods to `WorldMonitor`. Rejected because `VisualizationSpec` is already the intended display-only boundary and the pipeline owns all source data.

### 2. Share two plain layer-builder functions

A backend-neutral grasp visualization module will provide one function for `grasp/object-cloud` and one for `grasp/proposals`. The proposal builder will accept candidate visual states plus robot gripper sweep geometry and the grasp-frame-to-TCP transform. The live pipeline and standalone banana demo will use the same builders.

Alternative: import the standalone demo from production code. Rejected because demo lifecycle and fixture dependencies do not belong in the pick module. A presenter class was also rejected because the builders hold no state.

### 3. Treat the proposal layer as a latest-wins selection view

Before feasibility checks, the pipeline publishes all returned candidates as pending gray wireframes. Immediately before each validation and connected-plan check, it replaces the layer with earlier candidates red, the current candidate yellow, and later candidates gray. A rejected candidate becomes red on the next replacement. When a candidate passes the complete pre-grasp, grasp, and retreat sequence check, the pipeline atomically replaces the layer with only that candidate in green.

If every candidate fails, all evaluated candidates remain red. The object cloud and final proposal state remain visible after the skill returns and are replaced by the next pick.

Viser may skip intermediate yellow generations because its layer manager is asynchronous and latest-wins. The final green selection or all-red failure state remains the eventual value. The pipeline will not wait or sleep for rendering.

### 4. Use one common proposal input for every provider

`GraspGenSpec` will accept a provider-neutral proposal input containing:

- the validated segmented `PointCloud2`;
- the latest detected object center;
- the latest detected object size.

GraspGenX will consume the cloud and ignore the extra geometry. The heuristic provider will consume center and size so its current occlusion adjustment, tall-object adjustment, and distance-adaptive orientation remain unchanged. Both providers will return `GraspCandidateArray` in the input frame.

Alternative: derive heuristic geometry from the cloud. Rejected because accumulated clouds do not necessarily match the latest detection geometry. Giving the heuristic provider direct perception access was rejected because it would duplicate object resolution and synchronization.

### 5. Make the heuristic an explicit `GraspGenSpec` module

The internal fallback branch and `heuristic_grasp_fallback` configuration will be removed. A lightweight heuristic proposal module will implement the same Spec as GraspGenX and return the existing single candidate with score `0.0`.

`PickAndPlaceModule` will declare a non-optional `GraspGenSpec` dependency. Blueprint construction will fail when no provider or more than one provider matches. The pipeline result will remain provider-agnostic and will no longer report a fallback source.

Alternative: retain fallback configuration. Rejected because it lets one blueprint change algorithms at runtime and makes failures harder to reproduce.

### 6. Compose providers at the blueprint boundary

The real xArm perception stack will be factored into a provider-neutral private core. `xarm_perception` will add the heuristic provider; `xarm_graspgenx` will add GraspGenX without also starting an unused heuristic worker. The simulation perception stack will likewise add the heuristic provider.

The learned xArm blueprint will select Viser instead of Meshcat because Viser implements generic layer rendering and the hierarchical selector. Both xArm provider variants will receive grasp visualization geometry derived from the same xArm gripper configuration used by GraspGenX.

Alternative: compose both providers and resolve injection with remapping. Rejected because it wastes a worker and obscures which algorithm is active.

## Risks / Trade-offs

- [Fast planning may advance through candidates before Viser renders each yellow state] → keep latest-wins semantics and require only the eventual selected or failed state.
- [A required provider breaks provider-free `PickAndPlaceModule` blueprints] → update every shipped xArm composition and enforce the error at build time.
- [The proposal input signature is an RPC contract change] → migrate GraspGenX, fakes, demos, and tests in the same change.
- [The heuristic provider adds one worker to non-learned stacks] → update worker counts; the module remains lightweight and avoids loading GraspGenX.
- [Shared gripper geometry can drift between inference and visualization] → construct both configurations from the existing xArm geometry source and assert blueprint wiring.
- [A visualization conversion bug could otherwise abort selection] → contain construction and submission exceptions in the pipeline's best-effort helper.

## Migration Plan

1. Synchronize the active grasp-pipeline capability so this change can modify it.
2. Introduce the common proposal input and migrate GraspGenX plus all test providers.
3. Extract the heuristic provider and make `PickAndPlaceModule` require one provider.
4. Extract shared visualization builders and migrate the standalone demo.
5. Add live candidate-state publication to the selection loop.
6. Refactor xArm blueprints, select Viser for the learned stack, and update worker counts.
7. Run provider, pipeline, visualization, blueprint, and OpenSpec verification.

Rollback restores the old `GraspGenSpec` signature, internal heuristic branch, and previous blueprint composition as one unit. Visualization layers themselves require no data migration.

## Open Questions

None.
