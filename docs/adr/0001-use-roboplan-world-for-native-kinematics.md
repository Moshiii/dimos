# Use RoboPlanWorld for native kinematics

`RoboPlanWorld` implements `KinematicsSpec` for the RoboPlan kinematics backend
instead of introducing a separate stateless OInK adapter. OInK requires the
RoboPlan scene, model, planning groups, joint mappings, collision state, and
scene lock already owned by the world; a separate adapter would expose or
forward those internals without owning meaningful state or behavior. Each IK
request constructs and releases its own OInK session, so the kinematics role
retains no request-specific solver state between calls.

The request holds the world's scene lock for its entire solve. It snapshots the
shared scene configuration before applying the seed and restores that
configuration in a `finally` block. Planning and other scene users therefore
observe IK as one atomic operation, even when the solve fails.

One OInK instance is reused across all numerical iterations and restart attempts
within the request. It uses RoboPlan's `FrameTaskOptions`, `PositionLimit`, and
regularization defaults, with at most 100 OInK steps per attempt. DimOS checks
forward-kinematics errors after each step rather than treating an OInK step as a
complete solve. Every pose target must satisfy the requested position and
orientation tolerances. The result reports the maximum position error and
maximum orientation error across all targets.

## Consequences

The planning factory returns the same `RoboPlanWorld` instance for the world,
native planner, and RoboPlan kinematics roles. The initial
`RoboPlanKinematicsConfig` contains only the `backend="roboplan"`
discriminator; OInK tuning remains internal until hardware testing identifies
a setting that callers must control. No `configure_kinematics()` lifecycle is
introduced. Targets, tasks, constraints, barriers, warm-start state, and QP
workspaces are request-local.

Version one accepts only targets expressed in the `world` frame and does not add
a self-collision barrier. When collision checking is requested, DimOS validates
only converged endpoint candidates against the shared RoboPlan scene; numerical
IK iterations are search states, not an executable path.

The first attempt uses the supplied seed, with missing named joints filled from
the current world state. Later attempts randomize only joints belonging to
pose-targeted groups; auxiliary-group joints remain at their seeded positions.
The backend supports any non-overlapping pose-target and auxiliary-group
selection represented by `RoboPlanModel.groups`, including multiple targets,
multiple disjoint groups on one robot, and composite groups spanning robots.
Other selections return `IKStatus.UNSUPPORTED`. The returned joint state
contains only selected joints in selection order.

After all attempts, a converged candidate rejected for collision takes
precedence and produces `IKStatus.COLLISION`; otherwise the closest candidate
produces `IKStatus.NO_SOLUTION`. Closeness is ranked by the maximum normalized
error,
`max(position_error / position_tolerance, orientation_error /
orientation_tolerance)`. Failed results have no joint state. Persistent OInK
caching and collision barriers remain future performance and behavior work to
be justified by benchmarks.
