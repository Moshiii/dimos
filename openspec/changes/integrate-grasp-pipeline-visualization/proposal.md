## Why

The grasp pipeline produces the object cloud, ranked proposals, and feasibility result, but operators cannot inspect those values while a `pick` skill runs. The pipeline also embeds a heuristic fallback inside its orchestration, which makes the active proposal algorithm harder to identify and debug.

## What Changes

- Publish the segmented object cloud and grasp-candidate evaluation state automatically through the manipulation module's existing `VisualizationSpec`.
- Show rejected candidates in red, the candidate under evaluation in yellow, pending candidates in gray, and only the selected candidate in green after connected planning succeeds.
- Keep visualization asynchronous, display-only, and unable to fail or delay the pick operation.
- Extract the existing grasp layer construction into plain backend-neutral functions shared by the standalone demo and the live pipeline.
- **BREAKING**: replace the point-cloud-only `GraspGenSpec` request with a common proposal input that also carries the latest detected center and size.
- **BREAKING**: require exactly one `GraspGenSpec` provider for every `PickAndPlaceModule` blueprint instead of selecting an internal heuristic fallback at runtime.
- Move the current heuristic grasp algorithm unchanged into a standalone `GraspGenSpec` provider.
- Compose non-learned xArm perception stacks with the heuristic provider and the learned xArm stack with GraspGenX.
- Use Viser in the learned xArm grasp blueprint so generic grasp layers appear in its hierarchical layer selector.

## Capabilities

### New Capabilities

- `grasp-pipeline-visualization`: Automatic object-cloud and candidate-state visualization during grasp selection and execution.

### Modified Capabilities

- `grasp-pipeline-skill`: Replace heuristic fallback semantics with one required, blueprint-selected proposal provider and extend the provider input with current object geometry.

## Impact

- Affects `PickAndPlaceModule`, `GraspGenSpec`, GraspGenX, xArm blueprint composition, and the existing heuristic grasp implementation.
- Adds a lightweight heuristic proposal module and a provider-neutral proposal input model.
- Reuses `VisualizationLayer` and Viser without coupling the pick pipeline to a visualization backend.
- Changes xArm worker counts and makes provider ambiguity or absence a blueprint-build error.
- Depends on the active `add-grasp-pipeline-skill` capability being synchronized before this change is archived.
