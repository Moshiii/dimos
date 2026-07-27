# DimOS Robotics Context

DimOS describes robot capabilities and planning backends using terminology that
separates public robotics roles from the engines that implement them.

## Language

**RoboPlan kinematics backend**:
A DimOS kinematics backend that satisfies `KinematicsSpec` with RoboPlan OInK and
requires a `RoboPlanWorld` as its planning world.
_Avoid_: Generic OInK backend, WorldSpec-agnostic OInK

**RoboPlan OInK solver**:
RoboPlan's task-based optimal inverse-kinematics solver used inside the RoboPlan
kinematics backend.
_Avoid_: Generic IK solver, RoboPlan world

**Automatic kinematics selection**:
The startup policy that selects the world-native kinematics backend when one
exists and otherwise selects the generic Pink backend.
_Avoid_: Adaptive kinematics, runtime solver switching
