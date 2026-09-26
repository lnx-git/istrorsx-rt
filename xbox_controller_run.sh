#!/bin/bash

# ROS overlay + Fast DDS profile, so this works in a fresh terminal without
# having to remember "source _setup_env.sh" first. Harmless if the shell
# already has it -- re-sourcing the same setup files just reassigns them.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_setup_env.sh"
# Standard ROS 2 "joy" package, not one of this project's own nodes -- the
# deliberate, narrow exception to the "no external ROS2 packages" rule (see
# doc/ai/01_architecture.md § drive_node): a real gamepad needs no custom
# node, just joy_node publishing sensor_msgs/msg/Joy on the default /joy
# topic, which drive_node already subscribes to directly.
#
# deadzone is set low (0.05) deliberately -- drive_node applies its own,
# separate thresholds on top of this raw signal:
#   - steering: applyJoystickSteering() maps the raw axis proportionally onto
#     SA_MIN..SA_MAX (center -> SA_STRAIGHT), using this same 0.05 joy_node
#     deadzone as its only threshold, not a second/inconsistent one.
#   - throttle: pollKey()'s own "joystick_throttle_deadzone" ROS param
#     (default 0.5, wider than steering's) gates when 'w'/'s' fire at all,
#     and "joystick_velocity_max_fwd"/"joystick_velocity_max_back" cap how
#     far they can push velocity once they do -- both real-joystick-only (see
#     drive_node.hpp's own comment), overridable via
#     "ros2 run istrorsx_core drive_node --ros-args -p <name>:=<value>" or
#     the matching full.launch.py launch arguments. A real gamepad has both
#     axes on one physical stick and no closed-loop speed feedback to the
#     ESC, so without these, a mostly-lateral steering motion's small
#     forward/back component was enough to launch the robot.
# device_id 0 assumes the Xbox controller is the only/first joystick present
# (/dev/input/js0) -- pass e.g. "ros2 run joy joy_node --ros-args -p device_id:=1"
# directly instead of this script if a second device needs selecting.
ros2 run joy joy_node --ros-args -p deadzone:=0.05
