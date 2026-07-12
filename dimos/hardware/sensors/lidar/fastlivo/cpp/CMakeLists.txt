cmake_minimum_required(VERSION 3.14)
project(fastlivo_native CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# FAST-LIVO2's Log/ paths; opens fail silently if the dir doesn't exist.
add_definitions(-DROOT_DIR="/tmp/fastlivo_")

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/result" CACHE PATH "" FORCE)
endif()

# MP defines (same logic as upstream FAST-LIVO2)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  add_definitions(-DX86_ARCH)
  include(ProcessorCount)
  ProcessorCount(N)
  if(N GREATER 4)
    add_definitions(-DMP_EN -DMP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN -DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DARM_ARCH)
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include(FetchContent)

# Upstream source trees (nix passes store paths; local dev can pass checkouts)
if(NOT FASTLIVO2_DIR)
  message(FATAL_ERROR "Pass -DFASTLIVO2_DIR=<path to FAST-LIVO2 checkout>")
endif()
if(NOT SOPHUS_DIR)
  message(FATAL_ERROR "Pass -DSOPHUS_DIR=<path to old (non-templated) Sophus checkout>")
endif()
if(NOT VIKIT_DIR)
  message(FATAL_ERROR "Pass -DVIKIT_DIR=<path to rpg_vikit checkout>")
endif()

FetchContent_Declare(dimos_lcm
  GIT_REPOSITORY https://github.com/dimensionalOS/dimos-lcm.git
  GIT_TAG main
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(dimos_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common filters io)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# Old Sophus (non-templated, strasdat @ a621ff) — compiled in directly.
add_library(sophus_old STATIC
  ${SOPHUS_DIR}/sophus/so3.cpp
  ${SOPHUS_DIR}/sophus/se3.cpp
)
target_include_directories(sophus_old PUBLIC ${SOPHUS_DIR} ${EIGEN3_INCLUDE_DIR})

# vikit_common subset: camera models + math used by FAST-LIVO2 (catkin bypassed).
add_library(vikit_common STATIC
  ${VIKIT_DIR}/vikit_common/src/pinhole_camera.cpp
  ${VIKIT_DIR}/vikit_common/src/equidistant_camera.cpp
  ${VIKIT_DIR}/vikit_common/src/polynomial_camera.cpp
  ${VIKIT_DIR}/vikit_common/src/math_utils.cpp
  ${VIKIT_DIR}/vikit_common/src/robust_cost.cpp
  ${VIKIT_DIR}/vikit_common/src/vision.cpp
  ${VIKIT_DIR}/vikit_common/src/performance_monitor.cpp
)
target_include_directories(vikit_common PUBLIC
  ${VIKIT_DIR}/vikit_common/include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(vikit_common PUBLIC sophus_old ${OpenCV_LIBRARIES})

# FAST-LIVO2 core + LIVMapper, compiled UNMODIFIED against the fake-ROS shim
# headers in shim/ (listed before the upstream include dir so the shims win).
add_executable(fastlivo_native
  main.cpp
  lcm_io.cpp
  ${FASTLIVO2_DIR}/src/LIVMapper.cpp
  ${FASTLIVO2_DIR}/src/vio.cpp
  ${FASTLIVO2_DIR}/src/frame.cpp
  ${FASTLIVO2_DIR}/src/visual_point.cpp
  ${FASTLIVO2_DIR}/src/voxel_map.cpp
  ${FASTLIVO2_DIR}/src/preprocess.cpp
  ${FASTLIVO2_DIR}/src/IMU_Processing.cpp
)

target_include_directories(fastlivo_native PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/shim
  ${FASTLIVO2_DIR}/include
  ${dimos_lcm_SOURCE_DIR}/generated/cpp_lcm_msgs
  ${LCM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Upstream defines time_list() (non-inline) in common_lib.h; every TU that
# includes it carries an identical copy. Upstream's catkin build hides this by
# splitting TUs across static libs — linking them into one binary needs the
# duplicates allowed.
target_link_options(fastlivo_native PRIVATE -Wl,--allow-multiple-definition)

target_link_libraries(fastlivo_native PRIVATE
  vikit_common
  sophus_old
  ${LCM_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Boost::thread
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(fastlivo_native PRIVATE OpenMP::OpenMP_CXX)
endif()

target_link_directories(fastlivo_native PRIVATE ${LCM_LIBRARY_DIRS})

install(TARGETS fastlivo_native DESTINATION bin)
