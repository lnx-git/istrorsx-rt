#!/bin/bash
# One-shot GPS bring-up: configure the module, verify it, start gpsd on it, and record the raw
# NMEA stream to logout/gpspipe.log in the background.
#
#   ./_gpsd_run.sh
#
# Run this once per boot (or after replacing the GPS), before starting the
# robot -- gps_node talks to gpsd, not to the device, and fails at gps.init()
# if gpsd is not already running (doc/ai/04_installation.md).
#
# No _setup_env.sh here on purpose: nothing in this script is a ROS node, so
# the overlay and the Fast DDS profile would be noise.
set -e

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# gpsd holds the serial port open, so it has to go before gpsd_init.py can
# write to the module. Tolerate failure: nothing may be running yet.
echo "=== stopping gpsd ==="
script/gpsd_stop.sh || true

# Aborts the whole script (set -e) if the module does not come back sending
# valid RMC only -- starting gpsd on a misconfigured GPS just hides the fault
# until the robot is already driving.
echo
echo "=== configuring and verifying the GPS module ==="
script/gpsd_init.sh

echo
echo "=== starting gpsd ==="
script/gpsd_start.sh

# Raw NMEA from the module for the whole session, as legacy r.sh did ("gpspipe -r -o out/gpspipe.log").
# Appending (>>) instead of -o keeps "truncate" in script/mvout.sh safe while it keeps writing.
echo
echo "=== recording raw GPS sentences to logout/gpspipe.log ==="
mkdir -p logout
nohup gpspipe -r >> logout/gpspipe.log 2>/dev/null &
disown
echo "gpspipe pid $! -- stopped by script/gpsd_stop.sh"
