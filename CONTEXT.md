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

**Teleop Joint Velocity Limit**:
One maximum angular rate applied to every arm joint while operator control is active, without exceeding any joint's robot-model limit. Control IK uniformly scales the complete joint-velocity solution to satisfy the most restrictive joint while preserving motion direction. It does not constrain end-effector linear or angular speed.
_Avoid_: Teleop speed limit, TCP speed limit

**Joint Delta Guard**:
A per-control-tick bound on the difference between a measured joint angle and its candidate command. It rejects an anomalous candidate after optimization rather than shaping normal robot motion.
_Avoid_: Joint velocity limit, speed limit

**Joint Motion Damping**:
A soft optimization preference for lower joint angular velocity among valid teleop solutions. It does not pull the arm toward a posture or replace the Teleop Joint Velocity Limit.
_Avoid_: Posture preference, velocity limit

**Balanced Pose Tracking**:
End-effector pose tracking in which translation and orientation have equal optimization cost. Neither component is silently discarded when the complete pose cannot be reached in one control tick.
_Avoid_: Translation-priority IK, position-only IK
