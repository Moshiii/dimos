## Credentialed acceptance

Executed on 2026-08-05:

```bash
uv run dimos eval run \
  dimos/benchmark/realtime_sim/cases/go2-apartment-go-to-bed/case.json \
  --output=/tmp/dimos-eval-go-to-bed-codex-4
```

Final attempt:

- Path: `/tmp/dimos-eval-go-to-bed-codex-4/attempt_3f839a43d88f4fc2ad107497a780d86b`
- Attempt status: `completed`
- Task result: `passed`
- Terminal reason: `goal_reached`
- Tool calls: 3
- Duration: 86.3 seconds, including source preparation
- Private goal observations: 35
- Final private distance: 1.79 metres (two-metre goal threshold)

The run used the evaluator-owned `PimSimGo2` runtime, completed the 22-point
apartment memory preparation, started external Pi and standalone CodePolicy,
exercised Porcelain navigation, detected the private goal while the agent turn
was active, cancelled navigation, and cleaned up the owned runtime. Required
startup, preparation, agent, CodePolicy, private observation, private score,
manifest, lifecycle, cleanup, and outcome artifacts were present.

Privacy verification found no private semantic query, bounds, threshold, or
distance fields in `case.public.v1.json`, `outcome.v1.json`, or `events.jsonl`.
The final distance remains only in `goal-observations.private.v1.json`.

An earlier finalized attempt at
`/tmp/dimos-eval-go-to-bed-codex-3/attempt_3a84900a05cf4d93b116682c60398156`
did not reach the goal before its deadline. It finalized as
`completed`/`failed` with reason `episode_timeout`, confirming that a clean task
timeout is not misclassified as an infrastructure failure.

Environment preparation performed for the run:

- installed local editable PiMSim and `pimsim-dimos` packages;
- fetched the tracked `misc/DimSim/scenes/apartment/**` Git LFS assets;
- downloaded the MuJoCo Menagerie once;
- cooked and cached the PiMSim apartment scene package.

Compatibility work stayed in DimOS. PiMSim itself was not modified. DimOS now
accepts PiMSim's `mesh` browser visual target and multi-visual `visual_specs`
scene-cooking call.
