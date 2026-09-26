#!/bin/bash

# ROS overlay + Fast DDS profile, so this works in a fresh terminal without
# having to remember "source _setup_env.sh" first. Harmless if the shell
# already has it -- re-sourcing the same setup files just reassigns them.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_setup_env.sh"
# LD_PRELOAD works around a symbol collision: librealsense2.so bundles its own
# (unhidden) copy of eprosima::fastcdr/fastrtps for its DDS network-camera
# feature, which collides with ROS 2's own libfastcdr/libfastrtps and corrupts
# rclcpp::Node's first type registration -> segfault in CameraNode's ctor.
# Preloading the ROS 2 versions makes the dynamic linker prefer them. See
# doc/ai/03_progress.md for the librealsense2+rclcpp/DDS interaction history.
export LD_PRELOAD="/opt/ros/jazzy/lib/libfastcdr.so.2:/opt/ros/jazzy/lib/libfastrtps.so.2.14${LD_PRELOAD:+:$LD_PRELOAD}"
# Legacy r.sh's own parameter tail, kept identical to full.launch.py's
# common_args so both entry points produce the same Config. -cb2 leads it and
# has to: config.cpp parses argv in order and drops -cg/-ca silently unless
# useControlBoard2 is already set when it reaches them. Only ctrlboard_node
# acts on -cb2; for every other node it is an inert flag, like -lidar is for
# planner/save.
ros2 run istrorsx_hw camera_node -- -nowait -camdev 0 -cb2 /dev/ttyUSB_CB2 -i DIstrobotics -vf 12 -vf2 12 -vf3 12 -vb -17 -cg 300 -ca 0