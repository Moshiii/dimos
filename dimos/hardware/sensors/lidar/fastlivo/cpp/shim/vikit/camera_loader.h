// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Replaces vikit_ros's camera_loader (which reads ROS params) with one that
// reads the dimos_shim param map. main.cpp fills cam_* params from the first
// CameraInfo message on LCM, so any camera the recording used works without
// code changes: plumb_bob → Pinhole, equidistant → Equidistant.
#ifndef DIMOS_SHIM_VIKIT_CAMERA_LOADER_H
#define DIMOS_SHIM_VIKIT_CAMERA_LOADER_H

#include <string>

#include <ros/ros.h>
#include <vikit/abstract_camera.h>
#include <vikit/equidistant_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/polynomial_camera.h>

namespace vk {
namespace camera_loader {

inline double cam_param(const std::string& name, double default_value) {
    const auto& p = dimos_shim::params();
    auto it = p.find(name);
    double out = default_value;
    if (it != p.end()) { dimos_shim::parse_value<double>(it->second, out); }
    return out;
}

inline bool loadFromRosNs(const std::string& /*ns*/, vk::AbstractCamera*& cam) {
    const auto& p = dimos_shim::params();
    auto model_it = p.find("cam_model");
    if (model_it == p.end()) {
        fprintf(stderr, "[fastlivo] cam_model param missing (no CameraInfo received?)\n");
        return false;
    }
    const std::string& cam_model = model_it->second;

    const double width = cam_param("cam_width", 0);
    const double height = cam_param("cam_height", 0);
    const double scale = cam_param("scale", 1.0);
    const double fx = cam_param("cam_fx", 0);
    const double fy = cam_param("cam_fy", 0);
    const double cx = cam_param("cam_cx", 0);
    const double cy = cam_param("cam_cy", 0);

    if (width <= 0 || height <= 0 || fx <= 0 || fy <= 0) {
        fprintf(stderr, "[fastlivo] invalid camera intrinsics (w=%g h=%g fx=%g fy=%g)\n", width, height, fx, fy);
        return false;
    }

    if (cam_model == "Pinhole") {
        cam = new vk::PinholeCamera(width, height, scale, fx, fy, cx, cy,
                                    cam_param("cam_d0", 0.0), cam_param("cam_d1", 0.0),
                                    cam_param("cam_d2", 0.0), cam_param("cam_d3", 0.0),
                                    cam_param("cam_d4", 0.0));
    } else if (cam_model == "EquidistantCamera") {
        cam = new vk::EquidistantCamera(width, height, scale, fx, fy, cx, cy,
                                        cam_param("k1", 0.0), cam_param("k2", 0.0),
                                        cam_param("k3", 0.0), cam_param("k4", 0.0));
    } else if (cam_model == "PolynomialCamera") {
        cam = new vk::PolynomialCamera(width, height, fx, fy, cx, cy,
                                       cam_param("cam_skew", 0.0),
                                       cam_param("k2", 0.0), cam_param("k3", 0.0),
                                       cam_param("k4", 0.0), cam_param("k5", 0.0),
                                       cam_param("k6", 0.0), cam_param("k7", 0.0));
    } else {
        fprintf(stderr, "[fastlivo] unsupported cam_model: %s\n", cam_model.c_str());
        return false;
    }
    return true;
}

}  // namespace camera_loader
}  // namespace vk

#endif
