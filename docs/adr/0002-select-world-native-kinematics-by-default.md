# Select world-native kinematics by default

When kinematics configuration is omitted, stack construction selects the
world-native kinematics backend when one exists: `RoboPlanWorld` uses RoboPlan
OInK, while other `WorldSpec` implementations use Pink. Explicit kinematics
configuration still overrides this policy, allowing callers to use Pink with
`RoboPlanWorld`. The selection occurs once at startup and never switches
solvers at runtime.

## Consequences

The unresolved kinematics configuration must remain distinguishable from an
explicit `PinkKinematicsConfig`; configuration therefore uses `None` or an
equivalent `auto` value until the planning factory resolves it against the
selected world backend. Existing legacy `kinematics_name` values remain
explicit overrides. The `roboplan` distribution is treated as one atomic
dependency that includes `roboplan.optimal_ik`; there is no separate capability
probe or install path. A normal RoboPlan import failure fails stack construction,
and the system never silently falls back to Pink.
