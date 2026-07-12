// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#ifndef DIMOS_SHIM_VISUALIZATION_MSGS_MARKERARRAY_H
#define DIMOS_SHIM_VISUALIZATION_MSGS_MARKERARRAY_H

#include <vector>

#include <visualization_msgs/Marker.h>

namespace visualization_msgs {

struct MarkerArray {
    std::vector<Marker> markers;
};

}  // namespace visualization_msgs

#endif
