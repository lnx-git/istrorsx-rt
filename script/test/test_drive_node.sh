#!/bin/bash
# Manual end-to-end test for drive_node: starts it, publishes a ServoData
# (no obstacle, driving forward) and a DriveCommand (AUTONOMOUS-mode drive
# decision), then checks that /robot/speed_command comes out with the
# expected legacy-formula values:
#   steering_angle = (angle-90)*SA_MULT + SA_STRAIGHT = (90-90)*2+334 = 334
#   speed          = velocity                          = 343
# Nothing like this existed in the legacy single-process app -- there was no
# equivalent of publishing synthetic bus traffic to exercise one thread in
# isolation, since every thread just read whatever the real hardware/other
# threads had last written into the shared DataSet.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting drive_node..."
ros2 run istrorsx_core drive_node -- -nowait > /tmp/test_drive_node.log 2>&1 &
NODE_PID=$!
sleep 2

echo "Publishing ServoData (state=STATE_FWD, no obstacle)..."
ros2 topic pub -1 /robot/servo_data istrorsx_hw/msg/ServoData \
  "{state: 2, heading: 0.0, ircv: 0, ircv500: 0.0, angle: 335, velocity: 335, loadd: 0, cbtime: 0, ulsd1: 999, ulsd2: 999, ulsd3: 999, ulsd4: 999, ulsd5: 999}"

echo "Publishing DriveCommand (angle=90, velocity=343, state=NAV_ANGLE)..."
ros2 topic pub -1 /robot/drive_command istrorsx_core/msg/DriveCommand \
  "{angle: 90, velocity: 343, state: 5, stop: 0, yaw: 0.0}"

echo "Expected: /robot/speed_command messages with steering_angle=334 and speed=343"
echo "(controlTick() publishes SpeedCommand separately per changed field -- expect to see both"
echo " a steering_angle=334 message and a speed=343 message below; it keeps retriggering"
echo " periodically, so Ctrl+C-style truncation after a couple of messages is expected)"
timeout 3 ros2 topic echo /robot/speed_command || true

kill "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true
# "ros2 run"'s own PID ($NODE_PID above) isn't reliably the actual drive_node
# binary's PID -- pkill -f as a robustness fallback so a killed wrapper
# process doesn't leave the real node running behind it.
pkill -f "lib/istrorsx_core/drive_node" 2>/dev/null || true
echo "drive_node log: /tmp/test_drive_node.log"
