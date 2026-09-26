#!/bin/bash
# Builds test_camera_gen_cpp.cpp with a plain g++ call, deliberately NOT
# through colcon -- same reasoning as test_camera_delay_build.sh: it can then
# be compiled and run while a full test is in progress, without touching
# build/ or install/. The binary lands in /tmp, not in the workspace.
#
#   ./script/test/test_camera_gen_cpp_build.sh && /tmp/test_camera_gen_cpp
#
# Links against the already-installed istrorsx_hw message libraries, so
# "colcon build" must have run at least once.
set -e

cd "$(dirname "$0")/../.."
WS="$PWD"
source /opt/ros/jazzy/setup.bash

OUT=/tmp/test_camera_gen_cpp

g++ -std=c++17 -O2 -o "$OUT" script/test/test_camera_gen_cpp.cpp \
    -I"$WS/install/istrorsx_hw/include/istrorsx_hw" \
    -I/opt/ros/jazzy/include \
    $(for d in /opt/ros/jazzy/include/*/; do echo -I"$d"; done) \
    $(pkg-config --cflags opencv4 2>/dev/null) \
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
    $(pkg-config --libs opencv4 2>/dev/null || echo "-lopencv_core -lopencv_imgcodecs") \
    -Wl,-rpath,"$WS/install/istrorsx_hw/lib" \
    -Wl,-rpath,/opt/ros/jazzy/lib

echo "built: $OUT"
