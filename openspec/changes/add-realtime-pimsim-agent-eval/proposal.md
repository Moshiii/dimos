## Why

`dimos eval run <case>` currently executes only frozen-memory question answering, while the PiMSim `gotobed` behavior is exercised through a separate pytest system harness. Real-time simulated agent tasks need the same authored case contract, external Pi/CodePolicy execution, private validation, evidence, and normalized outcomes as static QA.

## What Changes

- Extend the canonical evaluation case with a simulator-scene source that binds the scene, PiMSim provider, robot, and DimOS blueprint as part of case identity.
- Add a bounded live CodePolicy interaction that exposes updating Memory2 and the DimOS Porcelain `app` to a fresh external Pi session.
- Add evaluator-owned periodic goal validation that ends an attempt when its private predicate succeeds or the case timeout expires.
- Extend `dimos eval run <case>` to dispatch both frozen QA and real-time simulator cases without a separate runtime-selection CLI option.
- Merge the DimOS-side functionality from `cc/chore/pimsim-integration` needed to discover and launch PiMSim through its existing provider and scene-control entry points; PiMSim itself remains unchanged.
- Add a checked-in Go2 apartment `gotobed` case whose private validator checks that the robot is within two metres of the semantic bed bounds.
- Retain real-time lifecycle, agent activity, private goal observations, score, cleanup, and normalized outcome evidence under the existing attempt artifact model.

## Capabilities

### New Capabilities

- `realtime-simulator-agent-evaluation`: Case-driven simulator and DimOS startup, external live Pi/CodePolicy interaction, evaluator-owned goal polling, termination, cleanup, evidence, and `dimos eval run` behavior.
- `pimsim-go-to-bed-evaluation`: The concrete PiMSim-backed Go2 apartment case, semantic bed-proximity predicate, and end-to-end acceptance behavior.

### Modified Capabilities

- `agent-evaluation-case-model`: Add case-bound simulator-scene sources, bounded live interactions, and private periodically evaluated goals while preserving the four-part source/task/interaction/validator contract.

## Impact

- Affects `dimos/benchmark/agent_eval/`, the `dimos eval run` command, case models, attempt control, progress rendering, and evidence retention.
- Integrates the DimOS simulation-provider and scene-control seams from `cc/chore/pimsim-integration` and adds PiMSim-backed runtime assembly without modifying the external PiMSim repository.
- Adds a real-time simulation case package and focused model, lifecycle, validator, CLI, and PiMSim smoke tests.
- Preserves existing frozen QA case files and commands; agent/authentication and presentation options remain operational CLI configuration.
