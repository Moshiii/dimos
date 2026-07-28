# DimOS Control

This context describes how coordinators, control tasks, and control observations relate during a robot control run.

## Language

**Control observation**:
An immutable, point-in-time view of coordinator state produced by one completed control tick.
_Avoid_: Task state, live state

**Control task context**:
A coordinator-owned access surface through which a registered control task obtains coordinator services. It exists for the coordinator's lifetime and carries no task-owned state.
_Avoid_: Runtime, view

**Task registration**:
The exclusive ownership relationship between one control coordinator and one control task. Registration grants context access; stopping and restarting the coordinator preserves registration, while removing the task ends it.
_Avoid_: Binding, attachment
