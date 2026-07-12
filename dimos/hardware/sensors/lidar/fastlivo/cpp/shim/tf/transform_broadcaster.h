// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_TF_TRANSFORM_BROADCASTER_H
#define DIMOS_SHIM_TF_TRANSFORM_BROADCASTER_H

#include <cmath>
#include <string>

#include <geometry_msgs/Geometry.h>
#include <ros/ros.h>

namespace tf {

class Vector3 {
public:
    Vector3() = default;
    Vector3(double x, double y, double z) : x_(x), y_(y), z_(z) {}
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }

private:
    double x_ = 0, y_ = 0, z_ = 0;
};

class Quaternion {
public:
    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }
    void setZ(double z) { z_ = z; }
    void setW(double w) { w_ = w; }
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }
    double w() const { return w_; }

private:
    double x_ = 0, y_ = 0, z_ = 0, w_ = 1;
};

class Transform {
public:
    void setOrigin(const Vector3& origin) { origin_ = origin; }
    void setRotation(const Quaternion& rotation) { rotation_ = rotation; }
    const Vector3& getOrigin() const { return origin_; }
    const Quaternion& getRotation() const { return rotation_; }

private:
    Vector3 origin_;
    Quaternion rotation_;
};

class StampedTransform : public Transform {
public:
    StampedTransform(const Transform& t, const ros::Time& stamp, const std::string& parent, const std::string& child)
        : Transform(t), stamp_(stamp), frame_id_(parent), child_frame_id_(child) {}

    ros::Time stamp_;
    std::string frame_id_;
    std::string child_frame_id_;
};

// TF is republished on the Python side (module.py derives it from the
// odometry stream, like the pointlio module) — the C++ broadcaster is inert.
class TransformBroadcaster {
public:
    void sendTransform(const StampedTransform& /*transform*/) {}
};

inline geometry_msgs::Quaternion createQuaternionMsgFromRollPitchYaw(double roll, double pitch, double yaw) {
    // ZYX convention, same as tf::Quaternion::setRPY.
    const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

}  // namespace tf

#endif
