#!/bin/bash

# ROS overlay + Fast DDS profile, so this works in a fresh terminal without
# having to remember "source _setup_env.sh" first. Harmless if the shell
# already has it -- re-sourcing the same setup files just reassigns them.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_setup_env.sh"
# -lidar sets conf.useLidar=1 so saveLidarGroup() actually writes lidar.png --
# save_node never opens the device itself, it only reads LidarData.msg, but it
# needs its own -lidar flag regardless of lidar_node's (each node parses its
# own argv into its own Config instance). Path matches lidar_node_run.sh's for
# consistency, though any value would do functionally.
# Legacy r.sh's own parameter tail, kept identical to full.launch.py's
# common_args so both entry points produce the same Config. -cb2 leads it and
# has to: config.cpp parses argv in order and drops -cg/-ca silently unless
# useControlBoard2 is already set when it reaches them. Only ctrlboard_node
# acts on -cb2; for every other node it is an inert flag, like -lidar is for
# planner/save.
ros2 run istrorsx_core save_node -- -nowait -lidar /dev/lidar0 -cb2 /dev/ttyUSB_CB2 -i DIstrobotics -vf 12 -vf2 12 -vf3 12 -vb -17 -cg 300 -ca 0
