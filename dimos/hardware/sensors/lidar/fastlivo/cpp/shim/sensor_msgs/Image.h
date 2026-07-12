// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Unlike ROS, the shim Image carries a decoded cv::Mat directly — the LCM
// glue does the wire→Mat conversion, and cv_bridge::toCvCopy just unwraps.
#ifndef DIMOS_SHIM_SENSOR_MSGS_IMAGE_H
#define DIMOS_SHIM_SENSOR_MSGS_IMAGE_H

#include <string>

#include <boost/shared_ptr.hpp>
#include <opencv2/core.hpp>

#include <std_msgs/Header.h>

namespace sensor_msgs {

struct Image {
    std_msgs::Header header;
    std::string encoding = "bgr8";  // encoding of `mat` (always bgr8 from the glue)
    cv::Mat mat;

    typedef boost::shared_ptr<Image> Ptr;
    typedef boost::shared_ptr<Image const> ConstPtr;
};

typedef boost::shared_ptr<Image> ImagePtr;
typedef boost::shared_ptr<Image const> ImageConstPtr;

}  // namespace sensor_msgs

#endif
