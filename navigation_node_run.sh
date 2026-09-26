#!/bin/bash

# ROS overlay + Fast DDS profile, so this works in a fresh terminal without
# having to remember "source _setup_env.sh" first. Harmless if the shell
# already has it -- re-sourcing the same setup files just reassigns them.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_setup_env.sh"
# -path sets conf.navigationPath AND conf.useNavigation=1 -- without it
# navigationTick() skips route planning entirely (see navigation_node.cpp's
# conf.useNavigation checks), so navmap.png/.kml would show no route and
# navp_idx would stay -1. "E1" is just a short demo path; use whatever the
# actual course needs. Note config.cpp only honours -path when a GPS device
# is configured ("requires gps!" warning otherwise).
# Legacy r.sh's own parameter tail, kept identical to full.launch.py's
# common_args so both entry points produce the same Config. -cb2 leads it and
# has to: config.cpp parses argv in order and drops -cg/-ca silently unless
# useControlBoard2 is already set when it reaches them. Only ctrlboard_node
# acts on -cb2; for every other node it is an inert flag, like -lidar is for
# planner/save.
ros2 run istrorsx_core navigation_node -- -nowait -path E1 -cb2 /dev/ttyUSB_CB2 -i DIstrobotics -vf 12 -vf2 12 -vf3 12 -vb -17 -cg 300 -ca 0
