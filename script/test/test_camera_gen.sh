#!/bin/bash
# Synthetic camera generator -- stands in for camera_node where there is no
# RealSense (the cloud dev VM). Publishes CameraFrame.msg at 30 FPS: a still
# from sample/ as the color half, plus a flat, obstacle-free depth frame built
# from vision_depth.cpp's own cdepth_col_ref1 reference. Runs until Ctrl+C.
#
#   ./script/test/test_camera_gen.sh                    # grass, front camera
#   ./script/test/test_camera_gen.sh cesta              # road instead
#   ./script/test/test_camera_gen.sh trava rear         # rear camera
#   ./script/test/test_camera_gen.sh cesta front 5      # 5 FPS
#
# image:  "trava" | "cesta" | any filename in sample/ | a full path
# camera: "front" | "rear"      fps: default 30
#
# image_number is stamped from /robot/image_number, exactly as camera_node
# does -- so run planner_node or script/test/test_image_number_gen.sh in
# another terminal if you need frames numbered (save_node pairs on it).
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec python3 script/test/test_camera_gen.py --ros-args \
    -p "image:=${1:-trava}" \
    -p "camera:=${2:-front}" \
    -p "fps:=${3:-30.0}"
