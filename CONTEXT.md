# DimOS Control

The control context defines how time-varying operator and autonomy intentions become arbitrated robot joint commands.

## Language

**Teleop IK Task**:
A control task that interprets an engaged operator's end-effector pose delta relative to a measured robot pose and produces arbitrated joint commands.
_Avoid_: Teleop controller, Cartesian teleop solver

**Cartesian IK Task**:
A control task that interprets an absolute end-effector pose target and produces arbitrated joint commands from the current measured robot state.
_Avoid_: Cartesian controller, IK solver

**Control IK**:
The measured-state inverse-kinematics calculation used by a control task on each coordinator tick.
_Avoid_: Planning IK, teleop IK

**Engagement Baseline**:
The measured end-effector pose captured when operator control begins, against which subsequent teleop pose deltas are interpreted. It is discarded on disengage, timeout, stop, clear, or E-STOP.
_Avoid_: Initial pose, home pose

**End-Effector Frame**:
The named robot-model frame whose pose a Cartesian control task targets. It is not inferred from a numeric kinematic-joint index.
_Avoid_: End-effector joint, EE joint ID
