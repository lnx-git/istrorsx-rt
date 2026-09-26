#!/bin/bash
# Synthetic control-board telemetry: publishes ImuData (/robot/imu_data) and
# ServoData (/robot/servo_data) at 50 Hz, for machines with no serial
# hardware. Runs until Ctrl+C.
#
# Telemetry only -- the outbound half of ctrlboard_node. It does NOT consume
# SpeedCommand/LedConfig/DeviceAction, so commands sent to the robot are
# silently ignored. Not a substitute for the board itself.
#
# Start this whenever planner_node is under test without the real board:
# planner_node produces NOTHING without IMU data. Its plannerTick() refuses
# to process while ctrlb_euler_x is unknown ("dont process data if
# ctrlb_euler_x is not known!!"), and only ImuData sets it -- measured here,
# 0 decisions without it, 25 in six seconds with it.
#
#   ./script/test/test_ctrlboard_data_gen.sh              # 50 Hz, robot at rest
#   ./script/test/test_ctrlboard_data_gen.sh 20           # 20 Hz instead
#   ./script/test/test_ctrlboard_data_gen.sh 50 324.12    # park log's heading
#
# rate: Hz, default 50    euler_x: IMU yaw in degrees, default 90
#
# Defaults describe a robot standing still (STATE_STOP, zero encoder, wheels
# straight, motor at rest) -- the truth for a mock fed by a still image. For
# anything else use the parameters directly, e.g.
#   ros2 run ... --ros-args -p state:=2 -p velocity:=343 -p ircv500:=29.0
# or run this script and pass extra -p flags after the two positionals.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

RATE="${1:-50.0}"
EULER_X="${2:-90.0}"
shift 2 2>/dev/null || shift $#

exec python3 script/test/test_ctrlboard_data_gen.py --ros-args \
    -p "rate:=${RATE}" \
    -p "euler_x:=${EULER_X}" \
    "$@"
