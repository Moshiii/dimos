// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_GEOMETRY_MSGS_H
#define DIMOS_SHIM_GEOMETRY_MSGS_H

#include <std_msgs/Header.h>

namespace geometry_msgs {

struct Quaternion {
    double x = 0, y = 0, z = 0, w = 1;
};

struct Vector3 {
    double x = 0, y = 0, z = 0;
};

struct Point {
    double x = 0, y = 0, z = 0;
};

struct Pose {
    Point position;
    Quaternion orientation;
};

struct PoseStamped {
    std_msgs::Header header;
    Pose pose;
};

struct PoseWithCovariance {
    Pose pose;
    double covariance[36] = {};
};

struct Twist {
    Vector3 linear;
    Vector3 angular;
};

struct TwistWithCovariance {
    Twist twist;
    double covariance[36] = {};
};

}  // namespace geometry_msgs

#endif
