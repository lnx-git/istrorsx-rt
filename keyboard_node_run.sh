#!/bin/bash

# ROS overlay + Fast DDS profile, so this works in a fresh terminal without
# having to remember "source _setup_env.sh" first. Harmless if the shell
# already has it -- re-sourcing the same setup files just reassigns them.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_setup_env.sh"
# Terminal-based keyboard input, replacing the standard "joy" package's
# joy_node (see xbox_controller_run.sh) for a real gamepad -- run this
# directly in an interactive terminal (not backgrounded/redirected), it
# reads raw keypresses from its own stdin. See doc/ai/01_architecture.md
# § keyboard_node.
ros2 run istrorsx_core keyboard_node
