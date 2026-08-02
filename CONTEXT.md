# DimOS Robotics

Language for robot hardware capabilities and their representation across DimOS.

## Language

**Gripper opening**:
A calibrated normalized gripper aperture where `0.0` is fully closed and `1.0` is fully open, used consistently for commands and feedback.
_Avoid_: Gripper position, gripper angle, gripper distance

**Hardware topology**:
The inherent arrangement and identity of a robot's buses, actuator groups, and capabilities. It defines what kind of robot something is and does not vary between runs.
_Avoid_: Runtime configuration, deployment configuration

**Runtime configuration**:
Deployment and control-policy values that may vary between runs without changing the robot's hardware topology.
_Avoid_: Hardware topology, robot definition

**Whole-body hardware**:
A physical robot controlled as one ordered set of named joints, with tasks selecting arm, gripper, or other subsets by joint name.
_Avoid_: Manipulator collection, adapter bundle

**Residual torque**:
Task-requested joint torque added above the gravity compensation computed from the robot model. Zero residual torque requests gravity support without additional task effort.
_Avoid_: Raw motor torque, total torque

**Hardware identifier**:
The name of the physical owner of connection, lifecycle, state reads, and command writes.
_Avoid_: Arm name, joint prefix

**Joint namespace**:
The logical prefix that groups joints for planning and task ownership independently of which physical hardware owns them.
_Avoid_: Hardware identifier
