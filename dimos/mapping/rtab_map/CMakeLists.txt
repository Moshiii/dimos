# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.14)
project(dimos_rtab_map CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Generated LCM message headers (sensor_msgs::Image, nav_msgs::Odometry, ...).
# Under nix, -DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM points this at the flake input,
# so the sandboxed build needs no network.
FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Header-only dimos native C++ SDK. Overridable so nix can point at the flake
# input instead of the in-repo relative path.
if(NOT DEFINED DIMOS_NATIVE_CPP_DIR)
  set(DIMOS_NATIVE_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../native/cpp)
endif()
add_subdirectory(${DIMOS_NATIVE_CPP_DIR} ${CMAKE_BINARY_DIR}/dimos_native)

# RTAB-Map's own C++ SDK. Unlike cuVSLAM this is open source and packaged in
# nixpkgs, so the flake just consumes pkgs.rtabmap -- there is no vendored blob
# and no license carve-out.
find_package(RTABMap REQUIRED)
find_package(OpenCV REQUIRED)
# PCL is how rtabmap hands back point clouds, and this module builds cloud_map
# out of them, so it is a direct dependency rather than a transitive one.
find_package(PCL REQUIRED COMPONENTS common filters)
message(STATUS "RTAB-Map ${RTABMap_VERSION}")

add_executable(rtabmap_slam src/rtabmap_slam.cpp)
target_include_directories(rtabmap_slam PRIVATE
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(rtabmap_slam PRIVATE
  dimos_native
  ${LCM_LIBRARIES}
  rtabmap::rtabmap
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)
target_link_directories(rtabmap_slam PRIVATE ${LCM_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
install(TARGETS rtabmap_slam RUNTIME DESTINATION bin)
