---
status: accepted
---

# Task registration owns coordinator context access

The control coordinator owns one stateless `ControlTaskContext` for its lifetime. Registering a `BaseControlTask` creates a private registration lease that grants transparent access through `task.context`; removing the task or deleting the coordinator destroys the lease, while stopping and restarting the coordinator preserves it. Access outside an active registration raises `RuntimeError`. This keeps context access aligned with coordinator ownership without requiring tasks to manage bind and unbind calls or optional context checks.

## Considered options

- Manual bind and unbind methods mixed context access with lifecycle bookkeeping and made cleanup visible throughout coordinator code.
- Invocation-scoped injection made context unavailable outside a command call even though the context belongs to the coordinator, not the invocation.
- Adding context to the structural `ControlTask` protocol would burden tick-driven tasks that need only the state passed to `compute()`.
