// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_LIVOX_ROS_DRIVER_CUSTOMMSG_H
#define DIMOS_SHIM_LIVOX_ROS_DRIVER_CUSTOMMSG_H

#include <cstdint>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <std_msgs/Header.h>

namespace livox_ros_driver {

struct CustomPoint {
    uint32_t offset_time = 0;  // ns relative to CustomMsg timebase
    float x = 0, y = 0, z = 0;
    uint8_t reflectivity = 0;
    uint8_t tag = 0;
    uint8_t line = 0;
};

struct CustomMsg {
    std_msgs::Header header;
    uint64_t timebase = 0;  // ns epoch of the first point
    uint32_t point_num = 0;
    uint8_t lidar_id = 0;
    uint8_t rsvd[3] = {};
    std::vector<CustomPoint> points;

    typedef boost::shared_ptr<CustomMsg> Ptr;
    typedef boost::shared_ptr<CustomMsg const> ConstPtr;
};

}  // namespace livox_ros_driver

#endif
