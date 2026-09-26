#!/bin/bash

# ROS overlay + Fast DDS profile, so this works in a fresh terminal without
# having to remember "source _setup_env.sh" first. Harmless if the shell
# already has it -- re-sourcing the same setup files just reassigns them.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_setup_env.sh"
# Waveshare UPS Power Module (C) battery telemetry -- no legacy CLI flags to
# pass (no Config/-nowait, see doc/ai/01_architecture.md § ups_node).
# Override i2c_bus/i2c_addr/poll_period_ms via --ros-args -p <name>:=<value>
# if this Jetson's wiring differs from the defaults (bus 7, addr 0x41).
ros2 run istrorsx_hw ups_node
