# Agent Code Policies

Agent-authored code that inspects robot observations and invokes robot capabilities
through DimOS.

## Language

**Code Policy**:
Executable code written directly by an agent to decide or perform robot behavior. A
code policy is analogous to a learned policy, but represents behavior as source code
rather than learned parameters.
_Avoid_: Blueprint, module, service

**Policy Execution**:
An agent-initiated foreground run of one submitted code-policy snippet that may
synchronously observe, reason, and call RPCs. It ends when the Policy Kernel
returns to idle; detached background work is not part of the Policy Execution.
_Avoid_: Stream trigger, automatic policy loop

**Policy Kernel**:
A persistent IPython kernel process attached to one running DimOS system in which
an agent inspects observations, defines code-policy functions, and explicitly
executes them.
_Avoid_: Policy, blueprint, notebook

**Code Policy Module**:
The DimOS module that owns the lifecycle of a Policy Kernel and exposes Policy
Execution to an agent.
_Avoid_: Policy Kernel, Python kernel module, REPL

**Policy Kernel Namespace**:
The persistent Python namespace in a Policy Kernel. Its DimOS-specific bindings
are `memory`, a native memory2 store attached to the active recording, and `app`,
a native `Dimos.connect()` handle attached to the running blueprint; agent
imports, functions, and variables persist across executions.
_Avoid_: Dataframe environment, observation snapshot, utility bundle

**Observation History**:
The timestamped observations available to code-policy logic for current-state and
historical queries.
_Avoid_: Dataframe, synchronized snapshot
