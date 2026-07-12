// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_CV_BRIDGE_H
#define DIMOS_SHIM_CV_BRIDGE_H

#include <stdexcept>
#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/Image.h>

namespace sensor_msgs {
namespace image_encodings {
const std::string BGR8 = "bgr8";
const std::string RGB8 = "rgb8";
const std::string MONO8 = "mono8";
}  // namespace image_encodings
}  // namespace sensor_msgs

namespace cv_bridge {

class CvImage {
public:
    std_msgs::Header header;
    std::string encoding;
    cv::Mat image;

    sensor_msgs::ImagePtr toImageMsg() const {
        auto msg = boost::make_shared<sensor_msgs::Image>();
        msg->header = header;
        msg->encoding = encoding;
        msg->mat = image;
        return msg;
    }
};

typedef boost::shared_ptr<CvImage> CvImagePtr;

// The shim Image already holds a decoded cv::Mat (bgr8 from the LCM glue),
// so "copy" is a Mat clone plus an encoding check.
inline CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& msg, const std::string& encoding) {
    auto out = boost::make_shared<CvImage>();
    out->header = msg->header;
    out->encoding = encoding;
    if (encoding == msg->encoding) {
        out->image = msg->mat.clone();
    } else if (encoding == "bgr8" && msg->encoding == "rgb8") {
        cv::cvtColor(msg->mat, out->image, cv::COLOR_RGB2BGR);
    } else if (encoding == "rgb8" && msg->encoding == "bgr8") {
        cv::cvtColor(msg->mat, out->image, cv::COLOR_BGR2RGB);
    } else {
        throw std::runtime_error("cv_bridge shim: unsupported conversion " + msg->encoding + " -> " + encoding);
    }
    return out;
}

inline CvImagePtr toCvCopy(const sensor_msgs::Image& msg, const std::string& encoding) {
    return toCvCopy(boost::make_shared<sensor_msgs::Image>(msg), encoding);
}

}  // namespace cv_bridge

#endif
