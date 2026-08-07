# Customize Pink through named task stacks

Robot-specific IK subclasses customize Pink by composing an ordered `dict[str, pink.tasks.Task]`, while `PinkIK` retains the fixed planning, streaming, solve, integration, mapping, and safety algorithms. The backend reserves and validates one `frame/<frame_name>` entry for every commanded frame; subclasses may call `super()` and tune, replace, or add named auxiliary tasks without requiring a universal tuning schema.

Each control-task instance exclusively owns its injected `PinkIK` instance and persistent task stack. The dictionary structure and task identities are fixed after control-context creation; each tick mutates only targets or explicit temporal fields through before/after-solve hooks. Plain `PinkIK` preserves its current per-tick measured-posture target, while subclasses may declare a fixed nominal posture. Stateful tasks are never shared, and any temporal reset semantics belong to the subclass that introduces that state rather than to the coordinator or generic IK lifecycle.

Robot-specific control-task subclasses, class import paths in serialized configuration, and a global schema containing every possible Pink task parameter were rejected. A reusable intermediate subclass such as `DualArmPinkIK` should exist only when multiple robots share concrete task declarations, not merely to label a two-frame solve.
