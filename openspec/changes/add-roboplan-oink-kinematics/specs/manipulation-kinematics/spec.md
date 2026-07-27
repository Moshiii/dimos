## ADDED Requirements

### Requirement: World-sensitive default backend

When a caller omits kinematics configuration, DimOS SHALL select the native
RoboPlan inverse-kinematics backend for a RoboPlan planning world and SHALL
select Pink for any other supported planning world. Backend selection SHALL
occur once during stack construction.

#### Scenario: RoboPlan world uses native kinematics

- **GIVEN** a manipulation stack configured with a RoboPlan world
- **AND** no explicit kinematics backend
- **WHEN** the stack is constructed
- **THEN** inverse-kinematics requests use the RoboPlan-native backend

#### Scenario: Other worlds retain Pink

- **GIVEN** a manipulation stack configured with a non-RoboPlan world
- **AND** no explicit kinematics backend
- **WHEN** the stack is constructed
- **THEN** inverse-kinematics requests use Pink

#### Scenario: Explicit backend overrides the default

- **GIVEN** a RoboPlan planning world
- **AND** Pink is explicitly selected through typed configuration or the legacy
  kinematics name
- **WHEN** the stack is constructed
- **THEN** inverse-kinematics requests use Pink

#### Scenario: Incompatible native selection fails fast

- **GIVEN** RoboPlan-native kinematics is explicitly selected with a
  non-RoboPlan world
- **WHEN** the stack is constructed
- **THEN** construction fails with an actionable incompatibility error
- **AND** DimOS does not silently select Pink

### Requirement: Planning-group-scoped target support

The RoboPlan-native backend SHALL solve every non-overlapping combination of
pose-target and auxiliary planning groups represented by the planning world,
including multiple targets, disjoint groups on one robot, and composite groups
spanning robots. Successful results SHALL contain only selected joints in
planning-group selection order.

#### Scenario: Multiple pose targets converge

- **GIVEN** two or more represented, non-overlapping planning groups with
  world-frame pose targets
- **WHEN** all target frames can reach their requested poses
- **THEN** the solve succeeds only after every target satisfies both requested
  tolerances
- **AND** the result contains the selected joints in selection order

#### Scenario: Composite multi-robot target

- **GIVEN** represented pose-target groups belonging to multiple robots
- **WHEN** inverse kinematics is requested for the composite selection
- **THEN** DimOS solves the selected robot groups as one request
- **AND** evaluates the final composite robot configuration together

#### Scenario: Auxiliary joints participate without a pose target

- **GIVEN** pose-target groups and represented auxiliary groups
- **WHEN** inverse kinematics is requested
- **THEN** auxiliary joints are included in the selected result
- **AND** their seeded positions are retained across restart attempts

#### Scenario: Auxiliary-only selection

- **GIVEN** one or more represented auxiliary groups and no pose targets
- **WHEN** inverse kinematics is requested
- **THEN** DimOS returns the seeded or current selected configuration without
  numerical solving

#### Scenario: Unsupported group selection

- **GIVEN** overlapping groups or a selection not represented by the planning
  world
- **WHEN** inverse kinematics is requested
- **THEN** the result has status `UNSUPPORTED`
- **AND** has no joint state

### Requirement: Seed and restart behavior

The backend SHALL use the supplied seed for the first attempt, fill omitted
named joints from current world state, and limit subsequent randomization to
joints belonging to pose-targeted groups.

#### Scenario: Partial named seed

- **GIVEN** a seed containing values for only some selected robot joints
- **WHEN** inverse kinematics begins
- **THEN** supplied values override those joints
- **AND** missing joint values come from the current planning-world state

#### Scenario: Restart after a failed attempt

- **GIVEN** a request permitting more than one attempt
- **AND** the first attempt does not produce an acceptable solution
- **WHEN** the next attempt begins
- **THEN** pose-targeted joints are restarted within their joint limits
- **AND** auxiliary-group joints retain their seeded positions

### Requirement: Independent convergence and error reporting

DimOS SHALL determine convergence from forward-kinematics errors and the
caller-provided position and orientation tolerances rather than treating one
native solver step as a completed solve. For multiple targets, result errors
SHALL be the maximum position error and maximum orientation error across all
targets.

#### Scenario: All targets satisfy tolerances

- **GIVEN** one or more pose targets
- **WHEN** every target's position and orientation errors are within the
  requested tolerances
- **THEN** the candidate is considered converged
- **AND** the result reports the worst position and orientation errors

#### Scenario: One target remains outside tolerance

- **GIVEN** multiple pose targets
- **AND** at least one target remains outside either requested tolerance
- **WHEN** an attempt reaches its numerical iteration limit
- **THEN** that candidate is not reported as a successful solution

### Requirement: Endpoint collision policy

When collision checking is requested, the backend SHALL validate converged
endpoint candidates using the planning world's current collision state. It
SHALL evaluate all selected robots together and SHALL NOT treat intermediate
numerical search states as an executable collision-free path.

#### Scenario: Collision-free endpoint

- **GIVEN** collision checking is enabled
- **AND** a candidate converges within all requested tolerances
- **WHEN** the composite endpoint is collision-free
- **THEN** the solve returns `SUCCESS` with the selected joint state

#### Scenario: Converged endpoint collides

- **GIVEN** collision checking is enabled
- **AND** a candidate converges within all requested tolerances
- **WHEN** the composite endpoint is in collision
- **THEN** that candidate is rejected
- **AND** another attempt may be tried

#### Scenario: Collision checking is disabled

- **GIVEN** collision checking is disabled
- **WHEN** a candidate converges within all requested tolerances
- **THEN** the solve may return it without an endpoint collision query

### Requirement: Deterministic failure reporting

After exhausting the allowed attempts, the backend SHALL report `COLLISION` if
any candidate converged but was rejected for collision; otherwise it SHALL
report `NO_SOLUTION` based on the closest candidate. Failed results SHALL have
no joint state.

#### Scenario: Collision takes precedence

- **GIVEN** all attempts are exhausted without an accepted solution
- **AND** at least one candidate converged but collided
- **WHEN** the result is returned
- **THEN** its status is `COLLISION`
- **AND** its joint state is absent

#### Scenario: Closest non-converged candidate

- **GIVEN** all attempts are exhausted
- **AND** no candidate both converged and was rejected for collision
- **WHEN** the result is returned
- **THEN** its status is `NO_SOLUTION`
- **AND** its reported errors come from the candidate minimizing the maximum
  normalized position-or-orientation error
- **AND** its joint state is absent

### Requirement: Target-frame boundary

The initial RoboPlan-native backend SHALL accept targets expressed in the
`world` frame and SHALL reject other target frames as unsupported.

#### Scenario: World-frame target

- **GIVEN** a represented pose-target group
- **AND** a target whose frame identifier is `world`
- **WHEN** inverse kinematics is requested
- **THEN** the backend evaluates the target in the planning world frame

#### Scenario: Non-world target

- **GIVEN** a target whose frame identifier is not `world`
- **WHEN** RoboPlan-native inverse kinematics is requested
- **THEN** the result has status `UNSUPPORTED`
- **AND** has no joint state

### Requirement: Planning-world state isolation

An inverse-kinematics request SHALL NOT leave seed, iteration, restart, or
candidate joint configurations applied to the shared planning world after the
request returns or raises an error.

#### Scenario: Successful solve restores world state

- **GIVEN** a finalized RoboPlan planning world
- **WHEN** an inverse-kinematics request succeeds
- **THEN** subsequent world and planning operations observe the same shared
  scene configuration that existed before the request

#### Scenario: Failed solve restores world state

- **GIVEN** a finalized RoboPlan planning world
- **WHEN** an inverse-kinematics request fails or encounters a solver error
- **THEN** subsequent world and planning operations observe the same shared
  scene configuration that existed before the request

### Requirement: Bundled native dependency

DimOS SHALL treat native inverse kinematics as part of the RoboPlan
distribution and SHALL fail fast if the selected RoboPlan installation cannot
provide it.

#### Scenario: Incomplete RoboPlan installation

- **GIVEN** a RoboPlan stack with native kinematics selected
- **AND** the installed RoboPlan distribution cannot load its inverse-kinematics
  functionality
- **WHEN** the stack is constructed
- **THEN** construction fails with an actionable RoboPlan dependency error
- **AND** DimOS does not silently fall back to another backend
