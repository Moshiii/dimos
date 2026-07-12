// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_PCL_CONVERSIONS_H
#define DIMOS_SHIM_PCL_CONVERSIONS_H

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <pcl/io/pcd_io.h>  // pcl::PCDWriter (real ROS code gets it via this header chain)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>

namespace pcl {

// Only the non-Livox lidar handlers in preprocess.cpp call fromROSMsg; this
// module always feeds livox_pcl_cbk (built from the dimos PointCloud2's
// offset_time/tag/line fields), so those handlers must never run.
template <typename PointT>
void fromROSMsg(const sensor_msgs::PointCloud2& /*msg*/, pcl::PointCloud<PointT>& /*cloud*/) {
    fprintf(stderr, "[fastlivo] fromROSMsg is not supported in the dimos build "
                    "(use the livox lidar path / preprocess lidar_type AVIA)\n");
    std::abort();
}

namespace shim_detail {

inline void fill_cloud_header(sensor_msgs::PointCloud2& msg, uint32_t num_points, uint32_t point_step) {
    msg.height = 1;
    msg.width = num_points;
    msg.is_bigendian = false;
    msg.is_dense = true;
    msg.point_step = point_step;
    msg.row_step = point_step * num_points;
    msg.data.resize(msg.row_step);
}

inline sensor_msgs::PointField make_field(const char* name, uint32_t offset, uint8_t datatype) {
    sensor_msgs::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = datatype;
    f.count = 1;
    return f;
}

}  // namespace shim_detail

inline void toROSMsg(const pcl::PointCloud<pcl::PointXYZINormal>& cloud, sensor_msgs::PointCloud2& msg) {
    const uint32_t n = static_cast<uint32_t>(cloud.size());
    shim_detail::fill_cloud_header(msg, n, 16);
    msg.fields = {
        shim_detail::make_field("x", 0, sensor_msgs::PointField::FLOAT32),
        shim_detail::make_field("y", 4, sensor_msgs::PointField::FLOAT32),
        shim_detail::make_field("z", 8, sensor_msgs::PointField::FLOAT32),
        shim_detail::make_field("intensity", 12, sensor_msgs::PointField::FLOAT32),
    };
    for (uint32_t i = 0; i < n; ++i) {
        float* dst = reinterpret_cast<float*>(msg.data.data() + i * 16);
        dst[0] = cloud.points[i].x;
        dst[1] = cloud.points[i].y;
        dst[2] = cloud.points[i].z;
        dst[3] = cloud.points[i].intensity;
    }
}

inline void toROSMsg(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, sensor_msgs::PointCloud2& msg) {
    const uint32_t n = static_cast<uint32_t>(cloud.size());
    shim_detail::fill_cloud_header(msg, n, 16);
    msg.fields = {
        shim_detail::make_field("x", 0, sensor_msgs::PointField::FLOAT32),
        shim_detail::make_field("y", 4, sensor_msgs::PointField::FLOAT32),
        shim_detail::make_field("z", 8, sensor_msgs::PointField::FLOAT32),
        shim_detail::make_field("rgb", 12, sensor_msgs::PointField::FLOAT32),
    };
    for (uint32_t i = 0; i < n; ++i) {
        float* dst = reinterpret_cast<float*>(msg.data.data() + i * 16);
        dst[0] = cloud.points[i].x;
        dst[1] = cloud.points[i].y;
        dst[2] = cloud.points[i].z;
        const float rgb = cloud.points[i].rgb;
        std::memcpy(&dst[3], &rgb, sizeof(float));
    }
}

}  // namespace pcl

#endif
