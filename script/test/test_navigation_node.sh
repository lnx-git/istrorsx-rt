#!/bin/bash
# Manual end-to-end test for navigation_node: starts it with a navigation
# path ("-path E1", a point that exists in the compiled-in ISTRO_MAP_BA_SADJK
# map data -- see navig_data.cpp), publishes one GpsData fix near the map's
# reference point, and checks that /robot/navigation_data comes out with a
# located next-point (navp_idx, navp_dist, navp_azimuth) and a sane local XY
# position (x/y, via GeoCalc). Change "-path E1"/the coordinates below to
# exercise a different point or a different compiled-in park.
# Nothing like this existed in the legacy single-process app -- there was no
# equivalent of publishing synthetic bus traffic to exercise one thread in
# isolation, since every thread just read whatever the real hardware/other
# threads had last written into the shared DataSet.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting navigation_node with -path E1..."
ros2 run istrorsx_core navigation_node -- -nowait -path E1 > /tmp/test_navigation_node.log 2>&1 &
NODE_PID=$!
sleep 2

echo "Expected: /robot/navigation_data with navp_idx=3 (point \"E1\"), navp_dist/navp_azimuth computed, ref=1, x/y set"
# Start the echo subscriber BEFORE publishing (not after) -- navigation_node
# publishes its response the instant it processes the GpsData message, so a
# late subscriber can miss it entirely (no latched/retained message on this
# topic's default QoS).
timeout 4 ros2 topic echo /robot/navigation_data --once &
ECHO_PID=$!
sleep 1

echo "Publishing GpsData fix near the ISTRO_MAP_BA_SADJK reference point..."
ros2 topic pub -1 /robot/gps_data istrorsx_hw/msg/GpsData \
  "{fix: true, latitude: 48.1340837, longitude: 17.1138066, speed: 0.0, course: 0.0}"

wait "$ECHO_PID" 2>/dev/null || echo "(no message received within 4s)"

kill "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true
echo "navigation_node log: /tmp/test_navigation_node.log"
