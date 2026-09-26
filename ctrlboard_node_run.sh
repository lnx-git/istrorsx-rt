#!/bin/bash

# ROS overlay + Fast DDS profile, so this works in a fresh terminal without
# having to remember "source _setup_env.sh" first. Harmless if the shell
# already has it -- re-sourcing the same setup files just reassigns them.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_setup_env.sh"
# Legacy r.sh's own parameter tail, kept identical to full.launch.py's
# common_args so both entry points produce the same Config. The tail starts at
# -i here rather than -cb2: this node already passes its own -cb2 above (it is
# the one node that actually opens that port), and that satisfies the ordering
# -cg/-ca depend on -- config.cpp parses argv in order and drops them silently
# unless useControlBoard2 is already set when it reaches them.
ros2 run istrorsx_hw ctrlboard_node -- -nowait -cb /dev/ttyUSB_CB -cb2 /dev/ttyUSB_CB2 -i DIstrobotics -vf 12 -vf2 12 -vf3 12 -vb -17 -cg 300 -ca 0
