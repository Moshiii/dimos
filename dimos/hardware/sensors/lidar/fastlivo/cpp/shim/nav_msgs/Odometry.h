// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_NAV_MSGS_ODOMETRY_H
#define DIMOS_SHIM_NAV_MSGS_ODOMETRY_H

#include <string>

#include <boost/shared_ptr.hpp>

#include <geometry_msgs/Geometry.h>
#include <std_msgs/Header.h>

namespace nav_msgs {

struct Odometry {
    std_msgs::Header header;
    std::string child_frame_id;
    geometry_msgs::PoseWithCovariance pose;
    geometry_msgs::TwistWithCovariance twist;

    typedef boost::shared_ptr<Odometry> Ptr;
    typedef boost::shared_ptr<Odometry const> ConstPtr;
};

}  // namespace nav_msgs

#endif
