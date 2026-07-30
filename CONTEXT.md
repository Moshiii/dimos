# Grasp Planning

This context defines the language at the boundary between upstream perception, grasp generation, and robot motion planning.

## Language

**Segmented Object Cloud**:
A target-only 3D point cloud supplied by upstream perception in the manipulation planning frame.
_Avoid_: Scene cloud, detection cloud, raw camera cloud

**Feasible Grasp Sequence**:
A connected, collision-free trajectory from the robot's current state through any required safety lift, pre-grasp, grasp, and retreat. Each segment begins at the preceding segment's endpoint. Validation is a no-motion dry run; execution replans each segment from fresh measured state.
_Avoid_: Reachable grasp, feasible pose, independent IK success

**Safety Lift**:
An optional shared preparation segment planned before evaluating grasp candidates. If required and unplannable, the pick aborts once in `PREPARE`; the failure is not attributed to every candidate.
_Avoid_: Pre-grasp failure, candidate rejection

**Retreat Feasibility (MVP)**:
A connected grasp-to-retreat plan that collision-checks the robot and gripper against the non-target scene. The selected target remains excluded; attached-object geometry and held-object clearance are not modeled.
_Avoid_: Payload-safe retreat, attached-object validation

**Live-Scene Validation (MVP)**:
Each segment of a feasible grasp sequence is checked against the latest available planning scene. The MVP does not snapshot the scene or freeze non-target obstacle updates across the sequence. Execution replans against freshly measured state and scene data.
_Avoid_: Atomic scene validation, frozen-scene guarantee

**Gripper Geometry During Validation (MVP)**:
Arm-path collision checks use the gripper configuration currently represented in the planning scene. Dry-run validation does not model the open-to-closed gripper transition or claim separate clearance guarantees for each finger configuration.
_Avoid_: Coordinated arm-gripper plan, validated finger sweep

**Candidate Rejection Reason (MVP)**:
Candidate rejection is reported by failed sequence stage: `pre_grasp_infeasible`, `grasp_infeasible`, or `retreat_infeasible`. Detailed IK and planner outcomes remain diagnostic logs rather than public skill-result categories.
_Avoid_: Backend-specific public failure codes

**Pipeline Demo**:
A no-hardware contributor command that runs a recorded Segmented Object Cloud through real grasp proposal and connected motion validation, then saves candidate outcomes and all planned segments. It stops before trajectory execution.
_Avoid_: Grasp-only demo, hardware pick demo
