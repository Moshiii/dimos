// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// FAST-LIVO2 native module for the dimos NativeModule framework.
//
// Runs the unmodified upstream FAST-LIVO2 LIVMapper under the fake-ROS shim
// layer (shim/): LCM messages from the livox module (lidar w/ per-point
// offset_time, imu) and any camera (color_image + camera_info) are converted
// to shim ROS messages and fed straight into LIVMapper's own callbacks; its
// odometry publish is hooked back onto LCM. Camera intrinsics come from the
// camera_info stream at startup, so no camera model is hardcoded.
//
// Usage:
//   ./fastlivo_native \
//       --lidar '/lidar#sensor_msgs.PointCloud2' \
//       --imu '/imu#sensor_msgs.Imu' \
//       --color_image '/color_image#sensor_msgs.Image' \
//       --camera_info '/camera_info#sensor_msgs.CameraInfo' \
//       --odometry '/odometry#nav_msgs.Odometry' \
//       --frame_id odom --sensor_frame_id mid360_link \
//       --extrinsic_t=-0.011,-0.02329,0.04412 --rcl ... --pcl ...  # tuning as CLI args

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <boost/make_shared.hpp>
#include <opencv2/imgproc.hpp>

// Fake-ROS shims + unmodified FAST-LIVO2.
#include "LIVMapper.h"

#include "bridge.hpp"
#include "lcm_io.hpp"

using fastlivo_glue::Args;
using fastlivo_glue::BridgeCamInfo;
using fastlivo_glue::BridgeCloud;
using fastlivo_glue::BridgeImage;
using fastlivo_glue::BridgeImu;
using fastlivo_glue::BridgeOdom;
using fastlivo_glue::LcmIo;

namespace {

// CLI arg name -> FAST-LIVO2 ROS param name. Anything not passed falls back
// to the upstream default in readParameters()/loadVoxelConfig().
const std::pair<const char*, const char*> kParamMap[] = {
    {"img_en", "common/img_en"},
    {"lidar_en", "common/lidar_en"},
    {"ros_driver_bug_fix", "common/ros_driver_bug_fix"},
    {"normal_en", "vio/normal_en"},
    {"inverse_composition_en", "vio/inverse_composition_en"},
    {"max_iterations", "vio/max_iterations"},
    {"img_point_cov", "vio/img_point_cov"},
    {"raycast_en", "vio/raycast_en"},
    {"exposure_estimate_en", "vio/exposure_estimate_en"},
    {"inv_expo_cov", "vio/inv_expo_cov"},
    {"grid_size", "vio/grid_size"},
    {"grid_n_height", "vio/grid_n_height"},
    {"patch_pyrimid_level", "vio/patch_pyrimid_level"},
    {"patch_size", "vio/patch_size"},
    {"outlier_threshold", "vio/outlier_threshold"},
    {"exposure_time_init", "time_offset/exposure_time_init"},
    {"img_time_offset", "time_offset/img_time_offset"},
    {"imu_time_offset", "time_offset/imu_time_offset"},
    {"lidar_time_offset", "time_offset/lidar_time_offset"},
    {"imu_rate_odom", "uav/imu_rate_odom"},
    {"gravity_align_en", "uav/gravity_align_en"},
    {"gyr_cov", "imu/gyr_cov"},
    {"acc_cov", "imu/acc_cov"},
    {"imu_int_frame", "imu/imu_int_frame"},
    {"imu_en", "imu/imu_en"},
    {"gravity_est_en", "imu/gravity_est_en"},
    {"ba_bg_est_en", "imu/ba_bg_est_en"},
    {"blind", "preprocess/blind"},
    {"filter_size_surf", "preprocess/filter_size_surf"},
    {"lidar_type", "preprocess/lidar_type"},
    {"scan_line", "preprocess/scan_line"},
    {"point_filter_num", "preprocess/point_filter_num"},
    {"feature_extract_enabled", "preprocess/feature_extract_enabled"},
    {"extrinsic_t", "extrin_calib/extrinsic_T"},
    {"extrinsic_r", "extrin_calib/extrinsic_R"},
    {"pcl", "extrin_calib/Pcl"},
    {"rcl", "extrin_calib/Rcl"},
    {"blind_rgb_points", "publish/blind_rgb_points"},
    {"pub_scan_num", "publish/pub_scan_num"},
    {"pub_effect_point_en", "publish/pub_effect_point_en"},
    {"dense_map_en", "publish/dense_map_en"},
    {"max_layer", "lio/max_layer"},
    {"voxel_size", "lio/voxel_size"},
    {"min_eigen_value", "lio/min_eigen_value"},
    {"sigma_num", "lio/sigma_num"},
    {"beam_err", "lio/beam_err"},
    {"dept_err", "lio/dept_err"},
    {"layer_init_num", "lio/layer_init_num"},
    {"max_points_num", "lio/max_points_num"},
    {"lio_max_iterations", "lio/max_iterations"},
    {"map_sliding_en", "local_map/map_sliding_en"},
    {"half_map_size", "local_map/half_map_size"},
    {"sliding_thresh", "local_map/sliding_thresh"},
    // Direct camera overrides (normally filled from camera_info).
    {"cam_model", "cam_model"},
    {"cam_width", "cam_width"},
    {"cam_height", "cam_height"},
    {"cam_fx", "cam_fx"},
    {"cam_fy", "cam_fy"},
    {"cam_cx", "cam_cx"},
    {"cam_cy", "cam_cy"},
};

std::atomic<bool> g_got_camera_info{false};

void signal_handler(int /*sig*/) {
    dimos_shim::running().store(false);
}

// Map a generic CameraInfo (any camera, per its distortion_model) onto the
// vikit camera params the shim camera_loader reads. CLI-provided cam_* args
// win over the stream.
void apply_camera_info(const BridgeCamInfo& ci, const Args& args) {
    auto& params = dimos_shim::params();
    auto set_unless_cli = [&](const std::string& key, const std::string& value) {
        if (!args.has(key)) { params[key] = value; }
    };

    set_unless_cli("cam_width", std::to_string(ci.width));
    set_unless_cli("cam_height", std::to_string(ci.height));
    set_unless_cli("cam_fx", std::to_string(ci.K[0]));
    set_unless_cli("cam_fy", std::to_string(ci.K[4]));
    set_unless_cli("cam_cx", std::to_string(ci.K[2]));
    set_unless_cli("cam_cy", std::to_string(ci.K[5]));

    auto d = [&](size_t i) { return i < ci.D.size() ? ci.D[i] : 0.0; };
    if (ci.distortion_model == "equidistant" || ci.distortion_model == "fisheye") {
        set_unless_cli("cam_model", "EquidistantCamera");
        set_unless_cli("k1", std::to_string(d(0)));
        set_unless_cli("k2", std::to_string(d(1)));
        set_unless_cli("k3", std::to_string(d(2)));
        set_unless_cli("k4", std::to_string(d(3)));
    } else {
        // plumb_bob / empty / rational_polynomial-with-zero-tail all map to
        // the radial-tangential pinhole model.
        if (ci.distortion_model == "rational_polynomial" && (d(5) != 0 || d(6) != 0 || d(7) != 0)) {
            fprintf(stderr, "[fastlivo] warning: rational_polynomial k4-k6 nonzero, truncating to 5-coeff pinhole\n");
        }
        set_unless_cli("cam_model", "Pinhole");
        for (int i = 0; i < 5; ++i) {
            set_unless_cli("cam_d" + std::to_string(i), std::to_string(d(i)));
        }
    }
    fprintf(stderr, "[fastlivo] camera from camera_info: %s %dx%d fx=%.2f fy=%.2f cx=%.2f cy=%.2f (%s)\n",
            params["cam_model"].c_str(), ci.width, ci.height, ci.K[0], ci.K[4], ci.K[2], ci.K[5],
            ci.distortion_model.empty() ? "no distortion model" : ci.distortion_model.c_str());
}

}  // namespace

int main(int argc, char** argv) {
    Args args(argc, argv);

    const std::string lidar_channel = args.require("lidar");
    const std::string imu_channel = args.require("imu");
    const std::string odometry_channel = args.require("odometry");
    const std::string image_channel = args.get("color_image");
    const std::string camera_info_channel = args.get("camera_info");
    const std::string frame_id = args.require("frame_id");
    const std::string sensor_frame_id = args.require("sensor_frame_id");

    const bool debug = args.get_bool("debug", false);
    dimos_shim::debug_logging() = debug;

    const bool img_en = args.get_bool("img_en", true);
    if (img_en && (image_channel.empty() || camera_info_channel.empty())) {
        fprintf(stderr, "Error: --color_image and --camera_info are required when img_en is true\n");
        return 1;
    }

    // FAST-LIVO2 params: defaults chosen for the dimos deployment, then any
    // CLI arg from kParamMap overrides.
    auto& params = dimos_shim::params();
    params["imu/imu_en"] = "1";
    // Upstream defaults the extrinsic vectors to EMPTY (its yamls always set
    // them) and MAT_FROM_ARRAY on an empty vector silently yields a zero
    // matrix, which maps every lidar point to the origin. Always provide
    // usable defaults: Mid-360 built-in IMU for lidar↔IMU, identity for
    // lidar↔camera (the Python module passes the real rig values).
    params["extrin_calib/extrinsic_T"] = "-0.011,-0.02329,0.04412";
    params["extrin_calib/extrinsic_R"] = "1,0,0,0,1,0,0,0,1";
    params["extrin_calib/Pcl"] = "0,0,0";
    params["extrin_calib/Rcl"] = "1,0,0,0,1,0,0,0,1";
    for (const auto& [cli_name, param_name] : kParamMap) {
        if (args.has(cli_name)) { params[param_name] = args.get(cli_name); }
    }
    params["common/img_en"] = img_en ? "1" : "0";

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    LcmIo io;
    if (!io.good()) {
        fprintf(stderr, "Error: LCM init failed\n");
        return 1;
    }

    // The camera model must be known before LIVMapper is constructed (its
    // ctor builds the VIO manager), so block on the first CameraInfo.
    if (img_en) {
        io.subscribe_camera_info(camera_info_channel, [&](const BridgeCamInfo& ci) {
            if (!g_got_camera_info.load()) {
                apply_camera_info(ci, args);
                g_got_camera_info.store(true);
            }
        });
        fprintf(stderr, "[fastlivo] waiting for camera_info on %s ...\n", camera_info_channel.c_str());
        while (ros::ok() && !g_got_camera_info.load()) {
            io.handle_timeout(200);
        }
        if (!ros::ok()) { return 0; }
    } else {
        // LIVMapper unconditionally loads a camera even in LIO-only mode;
        // give it an inert one.
        params.emplace("cam_model", "Pinhole");
        params.emplace("cam_width", "640");
        params.emplace("cam_height", "480");
        params.emplace("cam_fx", "500");
        params.emplace("cam_fy", "500");
        params.emplace("cam_cx", "320");
        params.emplace("cam_cy", "240");
    }

    ros::NodeHandle nh;
    LIVMapper mapper(nh);
    image_transport::ImageTransport it(nh);
    mapper.initializeSubscribersAndPublishers(nh, it);

    fprintf(stderr,
            "[fastlivo] params: filter_size_surf=%.3f blind=%.3f point_filter=%d lidar_type=%d "
            "scan_line=%d imu_en=%d img_en=%d extT=[%.4f %.4f %.4f] slam_mode=%d\n",
            mapper.filter_size_surf_min, mapper.p_pre->blind, mapper.p_pre->point_filter_num,
            mapper.p_pre->lidar_type, mapper.p_pre->N_SCANS, static_cast<int>(mapper.imu_en),
            mapper.img_en, mapper.extT(0), mapper.extT(1), mapper.extT(2),
            static_cast<int>(mapper.slam_mode_));

    // Hook LIVMapper's odometry publish onto LCM. Upstream stamps with wall
    // now(); re-stamp from the EKF update time so replayed data keeps its own
    // clock domain.
    dimos_shim::publish_hooks()["/aft_mapped_to_init"] = [&](const void* msg_ptr) {
        const auto& odom = *static_cast<const nav_msgs::Odometry*>(msg_ptr);
        static int dbg_count = 0;
        if (debug && dbg_count < 3 && mapper.feats_undistort && !mapper.feats_undistort->empty()) {
            ++dbg_count;
            const auto& pts = mapper.feats_undistort->points;
            float mn[3] = {1e9f, 1e9f, 1e9f}, mx[3] = {-1e9f, -1e9f, -1e9f};
            for (const auto& p : pts) {
                mn[0] = std::min(mn[0], p.x); mx[0] = std::max(mx[0], p.x);
                mn[1] = std::min(mn[1], p.y); mx[1] = std::max(mx[1], p.y);
                mn[2] = std::min(mn[2], p.z); mx[2] = std::max(mx[2], p.z);
            }
            fprintf(stderr,
                    "[fastlivo dbg] undistort: %zu pts x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f] "
                    "curv[%.3f,%.3f] pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) grav=(%.2f,%.2f,%.2f)\n",
                    pts.size(), mn[0], mx[0], mn[1], mx[1], mn[2], mx[2],
                    pts.front().curvature, pts.back().curvature,
                    mapper._state.pos_end(0), mapper._state.pos_end(1), mapper._state.pos_end(2),
                    mapper._state.vel_end(0), mapper._state.vel_end(1), mapper._state.vel_end(2),
                    mapper._state.gravity(0), mapper._state.gravity(1), mapper._state.gravity(2));
        }
        BridgeOdom out;
        out.stamp = mapper.LidarMeasures.last_lio_update_time;
        out.px = odom.pose.pose.position.x;
        out.py = odom.pose.pose.position.y;
        out.pz = odom.pose.pose.position.z;
        out.qx = odom.pose.pose.orientation.x;
        out.qy = odom.pose.pose.orientation.y;
        out.qz = odom.pose.pose.orientation.z;
        out.qw = odom.pose.pose.orientation.w;
        std::memcpy(out.pose_covariance, odom.pose.covariance, sizeof(out.pose_covariance));
        io.publish_odometry(odometry_channel, out, frame_id, sensor_frame_id);
    };

    // LCM → LIVMapper's own ROS callbacks. Dispatch happens inside
    // ros::spinOnce() on the run-loop thread, matching single-threaded ROS
    // spinning, so LIVMapper's buffer locking assumptions hold.
    io.subscribe_cloud(lidar_channel, [&](const BridgeCloud& cloud) {
        auto msg = boost::make_shared<livox_ros_driver::CustomMsg>();
        msg->header.stamp = ros::Time().fromSec(cloud.stamp);
        msg->header.frame_id = sensor_frame_id;
        msg->timebase = static_cast<uint64_t>(cloud.stamp * 1e9);
        msg->point_num = static_cast<uint32_t>(cloud.points.size());
        msg->points.resize(cloud.points.size());
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            const auto& p = cloud.points[i];
            auto& cp = msg->points[i];
            cp.x = p.x;
            cp.y = p.y;
            cp.z = p.z;
            // dimos wire intensity is reflectivity/255; CustomMsg carries raw
            // reflectivity.
            cp.reflectivity = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, p.intensity * 255.0f)));
            cp.offset_time = p.offset_ns;
            cp.tag = p.tag;
            cp.line = p.line;
        }
        if (!cloud.has_offset_time) {
            static bool warned = false;
            if (!warned) {
                fprintf(stderr, "[fastlivo] warning: lidar stream has no offset_time field; "
                                "points get zero offsets (no motion deskew within a frame)\n");
                warned = true;
            }
        }
        mapper.livox_pcl_cbk(msg);
    });

    io.subscribe_imu(imu_channel, [&](const BridgeImu& imu) {
        auto msg = boost::make_shared<sensor_msgs::Imu>();
        msg->header.stamp = ros::Time().fromSec(imu.stamp);
        msg->angular_velocity.x = imu.wx;
        msg->angular_velocity.y = imu.wy;
        msg->angular_velocity.z = imu.wz;
        msg->linear_acceleration.x = imu.ax;
        msg->linear_acceleration.y = imu.ay;
        msg->linear_acceleration.z = imu.az;
        mapper.imu_cbk(msg);
    });

    if (img_en) {
        io.subscribe_image(image_channel, [&](const BridgeImage& img) {
            if (img.data.size() < static_cast<size_t>(img.step) * img.height) {
                fprintf(stderr, "[fastlivo] image data shorter than step*height, dropping\n");
                return;
            }
            const cv::Mat wire(img.height, img.width,
                               img.encoding == "mono8" ? CV_8UC1 : CV_8UC3,
                               const_cast<uint8_t*>(img.data.data()), img.step);
            auto msg = boost::make_shared<sensor_msgs::Image>();
            if (img.encoding == "rgb8") {
                cv::cvtColor(wire, msg->mat, cv::COLOR_RGB2BGR);
            } else if (img.encoding == "bgr8") {
                msg->mat = wire.clone();
            } else if (img.encoding == "mono8") {
                cv::cvtColor(wire, msg->mat, cv::COLOR_GRAY2BGR);
            } else {
                fprintf(stderr, "[fastlivo] unsupported image encoding '%s', dropping\n", img.encoding.c_str());
                return;
            }
            msg->encoding = "bgr8";
            msg->header.stamp = ros::Time().fromSec(img.stamp);
            mapper.img_cbk(msg);
        });
    }

    dimos_shim::spin_once_fn() = [&]() { io.handle_timeout(0); };

    fprintf(stderr, "[fastlivo] running (lidar=%s imu=%s img=%s odom=%s)\n",
            lidar_channel.c_str(), imu_channel.c_str(),
            img_en ? image_channel.c_str() : "(disabled)", odometry_channel.c_str());

    // LIVMapper::run() loops on ros::ok(), calling ros::spinOnce() (our LCM
    // dispatch) then sync_packages + estimation.
    mapper.run();

    fprintf(stderr, "[fastlivo] done\n");
    return 0;
}
