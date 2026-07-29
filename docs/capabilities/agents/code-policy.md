---
title: "Code Policies"
---

A code policy is Python authored by an agent and executed inside a running DimOS
blueprint. It is useful when one tool call needs to inspect observations, process
them with arbitrary Python, branch or retry, and coordinate several robot RPCs.
The first integration is the trusted, simulation-only
`xarm-perception-sim-agent` blueprint.

Install the agent runtime dependencies before using it:

```bash
uv sync --extra agents
```

## Execution model

The blueprint exposes one synchronous MCP skill:

```text
python_exec(code: str, timeout_s: float = 110.0) -> str
```

Each call submits one complete Python program. The result is a bounded plain-text
REPL transcript containing stdout, stderr, the final expression, or a traceback.
The MCP call waits for completion, for at most 110 seconds.

The Code Policy Module lazily creates a standard Jupyter/IPython kernel and
retains imports, functions, and variables between calls. An ordinary Python
exception also preserves mutations made before the exception. A timeout first
interrupts the active cell and preserves the namespace when the kernel recovers.
Only an unresponsive or dead kernel is restarted with a fresh namespace. A
timeout cannot cancel an RPC that has already reached another module, so the
result warns that remote work may still be running.

Only two DimOS handles are preloaded:

- `app` is a connected `Dimos` client. Use `app.skills.<name>(...)` for deployed
  skills or `app.get_module("<instance>")` for module RPCs.
- `memory` is a read-only-in-practice second `SqliteStore` attached to the
  explicitly configured recorder WAL database. It sees observations committed
  after the session attached as well as earlier history from the current
  blueprint run. The simulation demo overwrites this database when a new run
  starts.

The demo recorder intentionally captures only representative direct simulator
outputs that are enabled by default: `coordinator_joint_state` and
`color_image`. It is an integration fixture for the Memory2 path, not a
general-purpose manipulation or perception recorder.

For example, an agent can submit:

```python skip
latest_joint_state = memory.streams.coordinator_joint_state.last()
print(latest_joint_state.ts, latest_joint_state.data.position)

latest_image = memory.streams.color_image.last()
print(latest_image.ts, latest_image.data.shape)

app.skills.get_robot_state()
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
log, including source, Jupyter execution identity, duration, bounded output,
timeout, interrupt, and restart events:

```bash
uv run dimos log -f
uv run dimos log --json
```

The transcript also returns directly to the calling agent. No separate session
journal is maintained.

## Trust boundary

The v1 kernel is not a sandbox. Submitted code can import Python packages, call
RPCs, and access the kernel process's environment and filesystem permissions.
Enable it only for trusted agents and trusted code. It is intentionally included
only in the xArm perception simulation agent, not the real-hardware blueprint.

Run the simulation with:

```bash
uv run dimos --simulation --viewer none run xarm-perception-sim-agent \
  -o pickandplacemodule.visualization.backend=none
```
