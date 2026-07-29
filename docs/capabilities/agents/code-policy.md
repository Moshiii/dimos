---
title: "Code Policies"
---

A code policy is Python authored by an agent and executed inside a running DimOS
blueprint. It is useful when one tool call needs to inspect observations, process
them with arbitrary Python, branch or retry, and coordinate several robot RPCs.
The first integration is the trusted, simulation-only
`xarm-perception-sim-agent` blueprint.

## Execution model

The blueprint exposes one synchronous MCP skill:

```text
python_exec(code: str, timeout_s: float = 110.0) -> str
```

Each call submits one complete Python program. The result is a bounded plain-text
REPL transcript containing stdout, stderr, the final expression, or a traceback.
The MCP call waits for completion, for at most 110 seconds.

The Python process is created lazily and retains imports, functions, and variables
between successful calls. An ordinary Python exception also preserves mutations
made before the exception. A timeout, subprocess crash, or protocol failure
discards the process and namespace; the next call starts a fresh one. A timeout
cannot cancel an RPC that has already reached another module, so the result warns
that remote work may still be running.

Only two DimOS handles are preloaded:

- `app` is a connected `Dimos` client. Use `app.skills.<name>(...)` for deployed
  skills or `app.get_module("<instance>")` for module RPCs.
- `memory` is a read-only-in-practice second `SqliteStore` attached to the
  recorder's active WAL database. It sees observations committed after the
  session attached as well as earlier history.

For example, an agent can submit:

```python
latest_joint_state = memory.streams.coordinator_joint_state.last()
print(latest_joint_state.ts, latest_joint_state.data.position)

objects = memory.streams.objects.last().data
if objects:
    app.skills.pick(object_name=objects[0].name)
else:
    app.skills.scan_objects()
```

Use snapshot queries such as `.last()`, `.after(timestamp).limit(n).to_list()`,
or `.time_range(start, end)` for history. Do not collect an unbounded `.live()`
stream.

## Tool choice

The simulation agent is prompted to prefer `python_exec` for observation
processing, loops, branching, retries, and multi-RPC behavior. Direct skills
remain available and are simpler for one atomic action such as `open_gripper` or
`go_home`. This is soft routing in v1; there is no capability isolation layer.

## Observability

Every submission and its lifecycle are written to the normal structured DimOS
log, including source, kernel generation, duration, bounded output, error type,
timeout, and reset events:

```bash
uv run dimos log -f
uv run dimos log --json
```

The transcript also returns directly to the calling agent. No separate session
journal is maintained.

## Trust boundary

The v1 kernel is not a sandbox. Submitted code can import Python packages, call
RPCs, and access the worker's process environment and filesystem permissions.
Enable it only for trusted agents and trusted code. It is intentionally included
only in the xArm perception simulation agent, not the real-hardware blueprint.

Run the end-to-end example with:

```bash
./examples/code_policy_xarm_sim.sh
```
