#!/bin/bash
# Builds test_camera_delay.cpp with a plain g++ call, deliberately NOT through
# colcon: it can then be compiled and run while a full test is in progress,
# without touching build/ or install/ and without any risk of disturbing the
# nodes under observation. The binary lands in /tmp, not in the workspace.
#
#   ./script/test/test_camera_delay_build.sh && /tmp/test_camera_delay
#   /tmp/test_camera_delay /robot/camera_rear_data 20
#
# It links against the already-installed istrorsx_hw message libraries, so
# "colcon build" must have run at least once -- which it has if anything is
# running at all.
set -e

cd "$(dirname "$0")/../.."
WS="$PWD"
source /opt/ros/jazzy/setup.bash

OUT=/tmp/test_camera_delay

g++ -std=c++17 -O2 -o "$OUT" script/test/test_camera_delay.cpp \
    -I"$WS/install/istrorsx_hw/include/istrorsx_hw" \
    -I/opt/ros/jazzy/include \
    $(for d in /opt/ros/jazzy/include/*/; do echo -I"$d"; done) \
    -L"$WS/install/istrorsx_hw/lib" \
    -L/opt/ros/jazzy/lib \
    -listrorsx_hw__rosidl_typesupport_cpp \
    -listrorsx_hw__rosidl_generator_c \
    -lrclcpp -lrcl -lrcutils -lrmw -lrcpputils -lrosidl_runtime_c \
    -lrosidl_typesupport_cpp -ltracetools \
    -llibstatistics_collector \
    -lstatistics_msgs__rosidl_typesupport_cpp \
    -lrcl_interfaces__rosidl_typesupport_cpp \
    -lbuiltin_interfaces__rosidl_typesupport_cpp \
    -Wl,-rpath,"$WS/install/istrorsx_hw/lib" \
    -Wl,-rpath,/opt/ros/jazzy/lib

echo "built: $OUT"
