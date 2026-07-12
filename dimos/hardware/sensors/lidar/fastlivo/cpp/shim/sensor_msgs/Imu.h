// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_SENSOR_MSGS_IMU_H
#define DIMOS_SHIM_SENSOR_MSGS_IMU_H

#include <boost/shared_ptr.hpp>

#include <geometry_msgs/Geometry.h>
#include <std_msgs/Header.h>

namespace sensor_msgs {

struct Imu {
    std_msgs::Header header;
    geometry_msgs::Quaternion orientation;
    double orientation_covariance[9] = {};
    geometry_msgs::Vector3 angular_velocity;
    double angular_velocity_covariance[9] = {};
    geometry_msgs::Vector3 linear_acceleration;
    double linear_acceleration_covariance[9] = {};

    typedef boost::shared_ptr<Imu> Ptr;
    typedef boost::shared_ptr<Imu const> ConstPtr;
};

typedef boost::shared_ptr<Imu> ImuPtr;
typedef boost::shared_ptr<Imu const> ImuConstPtr;

}  // namespace sensor_msgs

#endif
