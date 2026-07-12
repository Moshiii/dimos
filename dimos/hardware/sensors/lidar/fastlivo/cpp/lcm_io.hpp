// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// LCM-facing half of the fastlivo module. Implementation (lcm_io.cpp) is the
// only TU that includes dimos-lcm generated headers; this interface stays
// neutral (see bridge.hpp).

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "bridge.hpp"

namespace fastlivo_glue {

class LcmIo {
public:
    LcmIo();
    ~LcmIo();

    bool good() const;

    // Channel strings are the full dimos LCM channel ("/topic#pkg.Type").
    void subscribe_cloud(const std::string& channel, std::function<void(const BridgeCloud&)> cb);
    void subscribe_imu(const std::string& channel, std::function<void(const BridgeImu&)> cb);
    void subscribe_image(const std::string& channel, std::function<void(const BridgeImage&)> cb);
    void subscribe_camera_info(const std::string& channel, std::function<void(const BridgeCamInfo&)> cb);

    void publish_odometry(const std::string& channel, const BridgeOdom& odom,
                          const std::string& frame_id, const std::string& child_frame_id);

    // Dispatch pending messages; returns after `timeout_ms` if none arrive.
    void handle_timeout(int timeout_ms);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fastlivo_glue
