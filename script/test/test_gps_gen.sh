#!/bin/bash
# Synthetic GPS generator: publishes a fixed position on /robot/gps_data every
# 1s, mimicking gps_node's ~1Hz fix rate without needing a real GPS fix or
# gpsd running. Runs until Ctrl+C -- a standalone utility for isolated
# testing of GpsData consumers (e.g. navigation_node, save_node's
# saveNavMap()), not a pass/fail test.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec python3 script/test/test_gps_gen.py
