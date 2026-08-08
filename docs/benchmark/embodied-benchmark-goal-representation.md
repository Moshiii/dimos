# How embodied benchmarks represent goals and checking

Established embodied benchmarks do not usually serialize executable goal-checking logic. They use one of two contracts:

1. Serialize episode facts and configuration that select a trusted, code-defined measure.
2. Serialize a restricted declarative goal language, then evaluate it with a trusted interpreter.

Habitat and RoboTHOR use the first pattern. BEHAVIOR uses the second for task goals, while keeping termination mechanics in code. None of the systems surveyed loads arbitrary checker code from each episode.

## Comparison

| Framework | Serialized per episode or task | Code-defined | Episode termination |
| --- | --- | --- | --- |
| Habitat-Lab / Habitat Challenge | Scene and start state; navigation goal data such as positions, object IDs/categories, and valid viewpoints; configuration names enabled measures and supplies thresholds | Registered `Measure` implementations such as `DistanceToGoal`, `Success`, `SPL`, and `Collisions` | Task action, a code-defined success/failure condition, or configured step/time limit |
| RoboTHOR ObjectNav | Scene, target `object_type`, start pose, and training-only shortest path data | Challenge runner applies visibility, `Stop`, trajectory, success, and SPL rules | `Stop` or `max_steps` |
| BEHAVIOR / BDDL | Objects, initial predicates, and a declarative logical `:goal`; task config selects activity definition/instance and termination parameters | BDDL compiler and simulator predicate implementations evaluate the goal; termination classes distinguish success from failure | Predicate goal success or another code-defined condition such as timeout |

## Habitat-Lab and Habitat Challenge

Habitat separates an episode specification from its measures. A `NavigationEpisode` carries `scene_id`, start pose, a list of goals, and optional shortest paths. ObjectNav extends this with an `object_category`; each `ObjectGoal` can carry an object ID, category, room metadata, position, and valid view points. These are data, not executable predicates. See the official [`NavigationEpisode` and `NavigationGoal` definitions](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/tasks/nav/nav.py#L60-L96) and [`ObjectGoalNavEpisode` and `ObjectGoal`](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/tasks/nav/object_nav_task.py#L30-L89).

A task configuration selects registered measures and supplies their parameters. For example, Habitat's `SuccessMeasurementConfig` names the code-defined `Success` measure and sets `success_distance`; `TaskConfig.success_measure` names which enabled measure represents task success. [`end_on_success`](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/config/default_structured_configs.py#L1327-L1366) controls whether that measure terminates the episode. The actual rule remains Python: `Success.update_metric` checks whether `Stop` was called and whether `DistanceToGoal` falls below the configured threshold. `SPL` and collision accumulation are also stateful Python measures. See the official [`Success`, `SPL`, and `Collisions` implementations](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/tasks/nav/nav.py#L461-L620).

A simplified Habitat-shaped contract is therefore:

```yaml
episode:
  scene_id: apartment.glb
  start_position: [x, y, z]
  goals: [{position: [x, y, z]}]
task:
  measurements:
    distance_to_goal: {type: DistanceToGoal, distance_to: VIEW_POINTS}
    success: {type: Success, success_distance: 1.0}
  success_measure: success
  end_on_success: true
```

The YAML selects registered code and provides data. It does not encode the `Stop && distance < threshold` algorithm. The Habitat Challenge adds benchmark policy on top: ObjectNav success requires the agent to call `STOP` at a position within the distance bound from a target instance with oracle visibility. The challenge publishes that rule as benchmark code and documentation, not in every episode. See the official [Habitat Challenge ObjectNav evaluation definition](https://github.com/facebookresearch/habitat-challenge#evaluation).

## RoboTHOR ObjectNav

RoboTHOR uses an even thinner episode schema. Its JSON episodes contain an ID, scene, target `object_type`, initial pose, and training-only `shortest_path` / `shortest_path_length`. The [official episode example](https://github.com/allenai/robothor-challenge#dataset) contains no checker type or expression.

The challenge runner owns checking. It runs until `Stop` or `max_steps`, records the trajectory, and marks a non-test episode successful when the agent stopped and the target object was visible. The controller's configured `visibilityDistance` supplies the one-metre proximity semantics. See the official [`inference_worker` implementation](https://github.com/allenai/robothor-challenge/blob/main/robothor_challenge/challenge.py#L110-L201), [challenge configuration](https://github.com/allenai/robothor-challenge/blob/main/challenge_config.yaml), and [success definition](https://github.com/allenai/robothor-challenge#agent).

RoboTHOR therefore serializes the inputs to one benchmark-wide checker. It does not serialize goal-checking logic or select among multiple checker implementations per episode.

## BEHAVIOR and BDDL

BEHAVIOR serializes more of the goal than navigation benchmarks. Each BDDL task definition declares objects, initial predicates, and a logical `:goal`. Goals can use predicate composition and quantification, for example an `and` block containing `forpairs` expressions over `inside` predicates. The official [BEHAVIOR task-definition guide](https://behavior.stanford.edu/behavior_components/behavior_tasks.html#task-structure) documents the schema and a complete example.

This is declarative goal data, not arbitrary executable code. `BehaviorTask` compiles the selected BDDL activity and installs two code-defined termination conditions: a timeout and `PredicateGoal`. On each step, `PredicateGoal` calls the compiled task's `check_goal`, which resolves predicate names through simulator-owned implementations. See the official [`BehaviorTask._create_termination_conditions`](https://behavior.stanford.edu/reference/tasks/behavior_task.html#omnigibson.tasks.behavior_task.BehaviorTask), [`PredicateGoal`](https://behavior.stanford.edu/reference/termination_conditions/predicate_goal.html), and [termination-condition base classes](https://behavior.stanford.edu/reference/termination_conditions/termination_condition_base.html).

BEHAVIOR shows when a serialized goal language pays off: a large task library shares a stable predicate vocabulary and generic logical composition. It still keeps predicate semantics, step-by-step evaluation, and the distinction between success and failure in trusted code. Its BDDL examples mainly describe world-state goals; stateful navigation constraints such as ordered visits, path budgets, or "never enter" conditions would require suitable code-defined predicates or termination measures rather than falling out of the state-goal syntax automatically.

## Implication for DimOS realtime simulation evaluation

Serializing the checker itself is unnecessary. The closest established pattern for DimOS is Habitat's hybrid:

- The case serializes goal facts, thresholds, budgets, and a stable checker identifier.
- DimOS owns a registry of trusted checker implementations.
- A checker consumes complete evaluator-frame deltas and returns `CONTINUE`, `PASS`, or `FAIL`.
- The case may select a small number of reusable checker types, but it does not contain Python, callbacks, or a general temporal-expression language.

For example:

```json
{
  "goal": {
    "type": "visit_sequence",
    "targets": ["television-id", "bed-id"],
    "distance_m": 1.5
  }
}
```

Here, JSON provides the inputs needed to instantiate `visit_sequence`; repository code defines target entry, ordering, failure behavior, evidence, and episode termination. A dedicated schema per category is useful only when it validates meaningful parameters or stabilizes the case contract. It is not a prerequisite for implementing goal checking. A plain checker ID plus validated parameters follows common benchmark practice and can evolve into typed models as each verified category lands.

Use a richer declarative language only if DimOS later develops a large task library whose authors must compose new goal logic without code changes. BEHAVIOR demonstrates that this choice also commits the project to a controlled predicate vocabulary, compiler/interpreter, validation tools, and precise simulator-backed predicate semantics.
