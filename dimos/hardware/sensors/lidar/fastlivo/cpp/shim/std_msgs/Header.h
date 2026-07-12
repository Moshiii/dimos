// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_STD_MSGS_HEADER_H
#define DIMOS_SHIM_STD_MSGS_HEADER_H

#include <cstdint>
#include <string>

#include <ros/ros.h>

namespace std_msgs {

struct Header {
    uint32_t seq = 0;
    ros::Time stamp;
    std::string frame_id;
};

}  // namespace std_msgs

#endif
