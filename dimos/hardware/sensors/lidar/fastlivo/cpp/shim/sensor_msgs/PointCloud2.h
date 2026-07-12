// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_SENSOR_MSGS_POINTCLOUD2_H
#define DIMOS_SHIM_SENSOR_MSGS_POINTCLOUD2_H

#include <cstdint>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <std_msgs/Header.h>

namespace sensor_msgs {

struct PointField {
    static constexpr uint8_t INT8 = 1;
    static constexpr uint8_t UINT8 = 2;
    static constexpr uint8_t INT16 = 3;
    static constexpr uint8_t UINT16 = 4;
    static constexpr uint8_t INT32 = 5;
    static constexpr uint8_t UINT32 = 6;
    static constexpr uint8_t FLOAT32 = 7;
    static constexpr uint8_t FLOAT64 = 8;

    std::string name;
    uint32_t offset = 0;
    uint8_t datatype = 0;
    uint32_t count = 0;
};

struct PointCloud2 {
    std_msgs::Header header;
    uint32_t height = 0;
    uint32_t width = 0;
    std::vector<PointField> fields;
    bool is_bigendian = false;
    uint32_t point_step = 0;
    uint32_t row_step = 0;
    std::vector<uint8_t> data;
    bool is_dense = false;

    typedef boost::shared_ptr<PointCloud2> Ptr;
    typedef boost::shared_ptr<PointCloud2 const> ConstPtr;
};

}  // namespace sensor_msgs

#endif
