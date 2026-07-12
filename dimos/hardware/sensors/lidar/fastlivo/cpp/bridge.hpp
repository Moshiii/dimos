// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Neutral data types passed between the LCM I/O translation unit (which
// includes dimos-lcm generated headers) and the FAST-LIVO2 glue translation
// unit (which includes the fake-ROS shim headers). The two sides define
// conflicting `sensor_msgs::`/`nav_msgs::` types, so they may never share a
// TU; everything crosses through these PODs instead.

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace fastlivo_glue {

struct BridgePoint {
    float x = 0, y = 0, z = 0;
    float intensity = 0;        // wire semantic: reflectivity/255, range [0,1]
    uint32_t offset_ns = 0;     // ns relative to the cloud stamp
    uint8_t tag = 0;
    uint8_t line = 0;
};

struct BridgeCloud {
    double stamp = 0;
    bool has_offset_time = false;
    std::vector<BridgePoint> points;
};

struct BridgeImu {
    double stamp = 0;
    double wx = 0, wy = 0, wz = 0;  // rad/s
    double ax = 0, ay = 0, az = 0;  // m/s^2 (FAST-LIVO2 self-normalizes at init)
};

struct BridgeImage {
    double stamp = 0;
    int32_t width = 0, height = 0, step = 0;
    std::string encoding;  // dimos wire encoding: rgb8 / bgr8 / mono8
    std::vector<uint8_t> data;
};

struct BridgeCamInfo {
    std::string distortion_model;
    int32_t width = 0, height = 0;
    double K[9] = {};
    std::vector<double> D;
};

struct BridgeOdom {
    double stamp = 0;
    double px = 0, py = 0, pz = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
    double vx = 0, vy = 0, vz = 0;
    double wx = 0, wy = 0, wz = 0;
    double pose_covariance[36] = {};
};

// Same "--key value" CLI convention as dimos::NativeModule (that header can't
// be included here: it pulls in generated std_msgs types that collide with
// the shim's).
class Args {
public:
    Args(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);
            if (arg.size() > 2 && arg[0] == '-' && arg[1] == '-' && i + 1 < argc) {
                kv_[arg.substr(2)] = argv[++i];
            }
        }
    }

    bool has(const std::string& key) const { return kv_.count(key) > 0; }

    std::string get(const std::string& key, const std::string& default_val = "") const {
        auto it = kv_.find(key);
        return it != kv_.end() ? it->second : default_val;
    }

    std::string require(const std::string& key) const {
        auto it = kv_.find(key);
        if (it == kv_.end()) {
            fprintf(stderr, "Error: missing required arg --%s\n", key.c_str());
            exit(1);
        }
        return it->second;
    }

    bool get_bool(const std::string& key, bool default_val) const {
        auto it = kv_.find(key);
        if (it == kv_.end()) { return default_val; }
        return it->second == "1" || it->second == "true" || it->second == "True";
    }

    double get_double(const std::string& key, double default_val) const {
        auto it = kv_.find(key);
        if (it == kv_.end()) { return default_val; }
        return std::stod(it->second);
    }

    const std::map<std::string, std::string>& all() const { return kv_; }

private:
    std::map<std::string, std::string> kv_;
};

}  // namespace fastlivo_glue
