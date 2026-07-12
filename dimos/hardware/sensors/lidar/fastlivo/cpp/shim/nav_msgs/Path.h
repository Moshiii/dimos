// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_NAV_MSGS_PATH_H
#define DIMOS_SHIM_NAV_MSGS_PATH_H

#include <vector>

#include <geometry_msgs/Geometry.h>
#include <std_msgs/Header.h>

namespace nav_msgs {

struct Path {
    std_msgs::Header header;
    std::vector<geometry_msgs::PoseStamped> poses;
};

}  // namespace nav_msgs

#endif
