#!/bin/bash
# Runs the CameraFrame delivery-jitter probe, building it first if needed.
# Reports the min/max/avg inter-arrival gap every N messages; Ctrl+C to stop.
#
#   ./script/test/test_camera_delay.sh                          # front camera, every 10
#   ./script/test/test_camera_delay.sh /robot/camera_rear_data  # rear camera
#   ./script/test/test_camera_delay.sh /robot/camera_front_data 20
#
# Run it alongside test_camera_gen.sh: the generator says whether a frame was
# SENT on time (its "period" column), this says whether it ARRIVED on time. A
# stall visible in both at the same instant is the publisher; a stall only
# here is delivery.
#
# The probe does nothing per message except take a timestamp, and has no timer
# of its own -- so whatever it measures is delivery, not its own slowness.
set -e

cd "$(dirname "$0")/../.."
BIN=/tmp/test_camera_delay
SRC=script/test/test_camera_delay.cpp

# Rebuild when the binary is missing (a reboot clears /tmp) or older than the
# source. Deliberately built outside colcon, so this never touches build/ or
# install/ and is safe to run while a test is in progress.
if [ ! -x "$BIN" ] || [ "$SRC" -nt "$BIN" ]; then
    echo "building $BIN ..."
    ./script/test/test_camera_delay_build.sh
fi

source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec "$BIN" "$@"
