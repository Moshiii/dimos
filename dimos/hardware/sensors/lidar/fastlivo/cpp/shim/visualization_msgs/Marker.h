// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_VISUALIZATION_MSGS_MARKER_H
#define DIMOS_SHIM_VISUALIZATION_MSGS_MARKER_H

#include <cstdint>
#include <string>

#include <geometry_msgs/Geometry.h>
#include <std_msgs/Header.h>

namespace visualization_msgs {

struct Marker {
    static constexpr int32_t CYLINDER = 3;
    static constexpr int32_t ADD = 0;

    struct Scale {
        double x = 0, y = 0, z = 0;
    };
    struct Color {
        float r = 0, g = 0, b = 0, a = 0;
    };

    std_msgs::Header header;
    std::string ns;
    int32_t id = 0;
    int32_t type = 0;
    int32_t action = 0;
    geometry_msgs::Pose pose;
    Scale scale;
    Color color;
    ros::Duration lifetime;
};

}  // namespace visualization_msgs

#endif
