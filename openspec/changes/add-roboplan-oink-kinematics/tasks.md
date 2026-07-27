## 1. Backend Configuration and Selection

- [x] 1.1 Add discriminator-only `RoboPlanKinematicsConfig`, include it in the
  kinematics configuration union and legacy name conversion, and update
  supported backend names.
- [x] 1.2 Make omitted module kinematics configuration distinguishable from an
  explicit Pink configuration while preserving `kinematics_name` as an explicit
  compatibility override.
- [x] 1.3 Update factory validation and construction so omitted configuration
  selects RoboPlan kinematics for a RoboPlan world and Pink otherwise, explicit
  configurations win, and incompatible RoboPlan-native selections fail fast.
- [x] 1.4 Add or update factory/config tests covering both automatic defaults,
  explicit Pink on RoboPlan, the legacy override, same-world native
  kinematics/planner identity, and incompatible combinations.

## 2. RoboPlan-Native IK

- [x] 2.1 Import bundled `roboplan.optimal_ik` with the RoboPlan backend and add
  focused internal request/result helpers without adding a dependency probe or
  fallback backend.
- [x] 2.2 Implement `RoboPlanWorld.solve` as the single-pose convenience wrapper,
  including finalized-world validation and the exactly-one-pose-targetable-group
  requirement.
- [x] 2.3 Implement planning-group selection and public/native joint, group, and
  tip-frame mapping for world-frame single-target, multi-target, auxiliary,
  disjoint same-robot, and composite multi-robot requests.
- [x] 2.4 Implement partial-seed fallback, selection-ordered results,
  auxiliary-only success, and retry seeds that randomize only pose-targeted
  joints within limits.
- [x] 2.5 Implement one request-local OInK instance using RoboPlan numerical
  defaults and `PositionLimit`, reused across attempts with a 100-step
  per-attempt cap.
- [x] 2.6 Implement per-step forward-kinematics convergence checks requiring
  every target to meet both tolerances and reporting maximum position and
  orientation errors.
- [x] 2.7 Implement optional composite endpoint collision validation without a
  self-collision barrier, including collision precedence and closest-candidate
  `NO_SOLUTION` ranking with failure `joint_state=None`.
- [x] 2.8 Hold the RoboPlan scene lock for the complete request and restore the
  pre-request scene configuration in `finally` for success, ordinary failure,
  and solver exceptions.

## 3. Solver Tests

- [x] 3.1 Add unit tests for single-target convergence, multi-target worst-error
  aggregation, iteration caps, and the single-group convenience wrapper.
- [x] 3.2 Add unit tests for partial seeds, restart randomization, auxiliary-only
  requests, selection ordering, and unsupported frames or group selections.
- [x] 3.3 Add unit tests for disjoint same-robot and multi-robot composite
  requests, including collision checks that apply all selected robot states
  together.
- [x] 3.4 Add unit tests for collision-disabled success, collision precedence,
  closest `NO_SOLUTION` reporting, request-local OInK reuse, and scene
  restoration after success, failure, and exceptions.
- [x] 3.5 Add integration coverage in
  `dimos/manipulation/test_roboplan_oink_integration.py`
  against the pinned RoboPlan package for a reachable world-frame target and
  verify the shared scene remains unchanged after solving.

## 4. Documentation

- [x] 4.1 Update `docs/capabilities/manipulation/index.md` with automatic
  backend selection, explicit overrides, valid combinations, bundled OInK, and
  the world-frame and endpoint-collision safety boundaries.
- [x] 4.2 Update `dimos/manipulation/planning/README.md` so factory defaults and
  package guidance describe RoboPlan-native kinematics.
- [x] 4.3 Review and retain
  `docs/adr/0001-use-roboplan-world-for-native-kinematics.md`,
  `docs/adr/0002-select-world-native-kinematics-by-default.md`, and `CONTEXT.md`
  as the final architectural record.

## 5. Verification and Manual QA

- [x] 5.1 Run
  `uv run pytest dimos/manipulation/test_planning_factory.py dimos/manipulation/test_roboplan_world.py dimos/manipulation/planning/monitor/test_world_monitor.py`
  plus the new focused OInK test module.
- [x] 5.2 Run Ruff formatting/checks and the repository's focused mypy command
  for all changed manipulation Python files.
- [x] 5.3 Run
  `uv run doclinks --dry-run docs/capabilities/manipulation/index.md`,
  `uv run doclinks --dry-run dimos/manipulation/planning/README.md`, and
  `uv run md-babel-py run docs/capabilities/manipulation/index.md`.
- [x] 5.4 Run `openspec validate add-roboplan-oink-kinematics`.
- [ ] 5.5 Through the existing manipulation inverse-kinematics client in a
  RoboPlan simulation, manually verify a reachable world-frame target, a
  collision-rejected endpoint, an explicit Pink override, and unchanged world
  state for the next planning request.
  Deferred by user; automated coverage is complete, but client-level simulation
  QA has not been run.
