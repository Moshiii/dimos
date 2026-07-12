// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_IMAGE_TRANSPORT_H
#define DIMOS_SHIM_IMAGE_TRANSPORT_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace image_transport {

class Publisher {
public:
    Publisher() = default;
    explicit Publisher(std::string topic) : topic_(std::move(topic)) {}

    void publish(const sensor_msgs::ImageConstPtr& msg) const {
        auto& hooks = dimos_shim::publish_hooks();
        auto it = hooks.find(topic_);
        if (it != hooks.end()) { it->second(static_cast<const void*>(msg.get())); }
    }

private:
    std::string topic_;
};

class ImageTransport {
public:
    explicit ImageTransport(ros::NodeHandle& /*nh*/) {}

    Publisher advertise(const std::string& topic, uint32_t /*queue*/) { return Publisher(topic); }
};

}  // namespace image_transport

#endif
