// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// The only TU that touches dimos-lcm generated message types (they collide
// with the fake-ROS shim namespaces used by the FAST-LIVO2 glue TU).

#include "lcm_io.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <vector>

#include <lcm/lcm-cpp.hpp>

#include "nav_msgs/Odometry.hpp"
#include "sensor_msgs/CameraInfo.hpp"
#include "sensor_msgs/Image.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/Imu.hpp"
#include "sensor_msgs/PointField.hpp"

namespace fastlivo_glue {

namespace {

double stamp_to_sec(const std_msgs::Time& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nsec);
}

struct FieldOffsets {
    int x = -1, y = -1, z = -1, intensity = -1, offset_time = -1, tag = -1, line = -1;
};

FieldOffsets find_fields(const sensor_msgs::PointCloud2& pc) {
    FieldOffsets f;
    for (const auto& field : pc.fields) {
        if (field.name == "x") { f.x = field.offset; }
        else if (field.name == "y") { f.y = field.offset; }
        else if (field.name == "z") { f.z = field.offset; }
        else if (field.name == "intensity") { f.intensity = field.offset; }
        else if (field.name == "offset_time") { f.offset_time = field.offset; }
        else if (field.name == "tag") { f.tag = field.offset; }
        else if (field.name == "line") { f.line = field.offset; }
    }
    return f;
}

template <typename T>
T read_at(const std::vector<uint8_t>& data, size_t base, int offset) {
    T v;
    std::memcpy(&v, data.data() + base + offset, sizeof(T));
    return v;
}

}  // namespace

struct LcmIo::Impl {
    lcm::LCM lcm;

    std::function<void(const BridgeCloud&)> cloud_cb;
    std::function<void(const BridgeImu&)> imu_cb;
    std::function<void(const BridgeImage&)> image_cb;
    std::function<void(const BridgeCamInfo&)> camera_info_cb;

    void on_cloud(const lcm::ReceiveBuffer* rbuf, const std::string& /*chan*/) {
        sensor_msgs::PointCloud2 pc;
        if (pc.decode(rbuf->data, 0, rbuf->data_size) < 0) {
            fprintf(stderr, "[fastlivo] PointCloud2 decode failed\n");
            return;
        }
        FieldOffsets f = find_fields(pc);
        if (f.x < 0 || f.y < 0 || f.z < 0) {
            fprintf(stderr, "[fastlivo] PointCloud2 missing x/y/z fields\n");
            return;
        }
        BridgeCloud out;
        out.stamp = stamp_to_sec(pc.header.stamp);
        out.has_offset_time = f.offset_time >= 0;
        const size_t n = static_cast<size_t>(pc.width) * (pc.height > 0 ? pc.height : 1);
        out.points.resize(n);
        const size_t step = pc.point_step;
        for (size_t i = 0; i < n; ++i) {
            const size_t base = i * step;
            BridgePoint& p = out.points[i];
            p.x = read_at<float>(pc.data, base, f.x);
            p.y = read_at<float>(pc.data, base, f.y);
            p.z = read_at<float>(pc.data, base, f.z);
            if (f.intensity >= 0) { p.intensity = read_at<float>(pc.data, base, f.intensity); }
            if (f.offset_time >= 0) { p.offset_ns = read_at<uint32_t>(pc.data, base, f.offset_time); }
            if (f.tag >= 0) { p.tag = pc.data[base + f.tag]; }
            if (f.line >= 0) { p.line = pc.data[base + f.line]; }
        }
        static bool printed_first = false;
        if (!printed_first && !out.points.empty()) {
            printed_first = true;
            float mn[3] = {1e9f, 1e9f, 1e9f}, mx[3] = {-1e9f, -1e9f, -1e9f};
            uint32_t max_off = 0;
            for (const auto& p : out.points) {
                mn[0] = std::min(mn[0], p.x); mx[0] = std::max(mx[0], p.x);
                mn[1] = std::min(mn[1], p.y); mx[1] = std::max(mx[1], p.y);
                mn[2] = std::min(mn[2], p.z); mx[2] = std::max(mx[2], p.z);
                max_off = std::max(max_off, p.offset_ns);
            }
            fprintf(stderr,
                    "[fastlivo] first cloud: %zu pts step=%d x[%.2f,%.2f] y[%.2f,%.2f] "
                    "z[%.2f,%.2f] max_offset=%.1fms stamp=%.6f\n",
                    out.points.size(), pc.point_step, mn[0], mx[0], mn[1], mx[1], mn[2], mx[2],
                    max_off / 1e6, out.stamp);
        }
        if (cloud_cb) { cloud_cb(out); }
    }

    void on_imu(const lcm::ReceiveBuffer* rbuf, const std::string& /*chan*/) {
        sensor_msgs::Imu imu;
        if (imu.decode(rbuf->data, 0, rbuf->data_size) < 0) {
            fprintf(stderr, "[fastlivo] Imu decode failed\n");
            return;
        }
        BridgeImu out;
        out.stamp = stamp_to_sec(imu.header.stamp);
        out.wx = imu.angular_velocity.x;
        out.wy = imu.angular_velocity.y;
        out.wz = imu.angular_velocity.z;
        out.ax = imu.linear_acceleration.x;
        out.ay = imu.linear_acceleration.y;
        out.az = imu.linear_acceleration.z;
        if (imu_cb) { imu_cb(out); }
    }

    void on_image(const lcm::ReceiveBuffer* rbuf, const std::string& /*chan*/) {
        sensor_msgs::Image img;
        if (img.decode(rbuf->data, 0, rbuf->data_size) < 0) {
            fprintf(stderr, "[fastlivo] Image decode failed\n");
            return;
        }
        BridgeImage out;
        out.stamp = stamp_to_sec(img.header.stamp);
        out.width = img.width;
        out.height = img.height;
        out.step = img.step;
        out.encoding = img.encoding;
        out.data.assign(img.data.begin(), img.data.end());
        if (image_cb) { image_cb(out); }
    }

    void on_camera_info(const lcm::ReceiveBuffer* rbuf, const std::string& /*chan*/) {
        sensor_msgs::CameraInfo ci;
        if (ci.decode(rbuf->data, 0, rbuf->data_size) < 0) {
            fprintf(stderr, "[fastlivo] CameraInfo decode failed\n");
            return;
        }
        BridgeCamInfo out;
        out.distortion_model = ci.distortion_model;
        out.width = ci.width;
        out.height = ci.height;
        for (int i = 0; i < 9; ++i) { out.K[i] = ci.K[i]; }
        out.D.assign(ci.D.begin(), ci.D.end());
        if (camera_info_cb) { camera_info_cb(out); }
    }
};

LcmIo::LcmIo() : impl_(new Impl()) {}
LcmIo::~LcmIo() = default;

bool LcmIo::good() const { return impl_->lcm.good(); }

void LcmIo::subscribe_cloud(const std::string& channel, std::function<void(const BridgeCloud&)> cb) {
    impl_->cloud_cb = std::move(cb);
    impl_->lcm.subscribe(channel, &Impl::on_cloud, impl_.get());
}

void LcmIo::subscribe_imu(const std::string& channel, std::function<void(const BridgeImu&)> cb) {
    impl_->imu_cb = std::move(cb);
    impl_->lcm.subscribe(channel, &Impl::on_imu, impl_.get());
}

void LcmIo::subscribe_image(const std::string& channel, std::function<void(const BridgeImage&)> cb) {
    impl_->image_cb = std::move(cb);
    impl_->lcm.subscribe(channel, &Impl::on_image, impl_.get());
}

void LcmIo::subscribe_camera_info(const std::string& channel, std::function<void(const BridgeCamInfo&)> cb) {
    impl_->camera_info_cb = std::move(cb);
    impl_->lcm.subscribe(channel, &Impl::on_camera_info, impl_.get());
}

void LcmIo::publish_odometry(const std::string& channel, const BridgeOdom& odom,
                             const std::string& frame_id, const std::string& child_frame_id) {
    nav_msgs::Odometry msg;
    msg.header.seq = 0;
    msg.header.frame_id = frame_id;
    msg.header.stamp.sec = static_cast<int32_t>(odom.stamp);
    msg.header.stamp.nsec = static_cast<int32_t>((odom.stamp - msg.header.stamp.sec) * 1e9);
    msg.child_frame_id = child_frame_id;
    msg.pose.pose.position.x = odom.px;
    msg.pose.pose.position.y = odom.py;
    msg.pose.pose.position.z = odom.pz;
    msg.pose.pose.orientation.x = odom.qx;
    msg.pose.pose.orientation.y = odom.qy;
    msg.pose.pose.orientation.z = odom.qz;
    msg.pose.pose.orientation.w = odom.qw;
    for (int i = 0; i < 36; ++i) {
        msg.pose.covariance[i] = odom.pose_covariance[i];
    }
    msg.twist.twist.linear.x = odom.vx;
    msg.twist.twist.linear.y = odom.vy;
    msg.twist.twist.linear.z = odom.vz;
    msg.twist.twist.angular.x = odom.wx;
    msg.twist.twist.angular.y = odom.wy;
    msg.twist.twist.angular.z = odom.wz;
    std::memset(msg.twist.covariance, 0, sizeof(msg.twist.covariance));
    impl_->lcm.publish(channel, &msg);
}

void LcmIo::handle_timeout(int timeout_ms) {
    impl_->lcm.handleTimeout(timeout_ms);
}

}  // namespace fastlivo_glue
