#!/bin/bash
# Verifies that a NavigationPointSet.msg (planner_node's QR-scan result) is
# actually applied to navigationPoint[] in BOTH processes that hold their own
# copy of that array: navigation_node (route matching) and save_node (draws it
# in navmap.png/.kml).
#
# Why this test exists: /robot/navigation_point_set was published by
# planner_node but had NO subscriber at all until 2026-08-30 -- the QR
# coordinate feature was inert end-to-end, and nothing would have caught it
# except a QR-code field test. See doc/ai/02_specifications.md § 4/7.
#
# Each node is started SEPARATELY on purpose: both log through the same
# navig:: logger (shared navig.cpp), so a combined run can't attribute the
# "navigation_point_set()" line to one node or the other.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

POINT_IDX=3
LAT=48.1339999
LON=17.1119999
MSG="{point_idx: $POINT_IDX, point_latitude: $LAT, point_longitude: $LON}"

check_node() {
    local node_name="$1"; shift
    local logmark
    logmark="$(date '+%Y-%m-%d %H:%M:%S')"

    echo "--- $node_name ---"
    ros2 run istrorsx_core "$node_name" -- "$@" > "/tmp/test_nps_${node_name}.log" 2>&1 &
    sleep 3   # discovery settle -- see doc/ai/06_hw_testing_guide.md trap #5

    ros2 topic pub --once /robot/navigation_point_set \
        istrorsx_core/msg/NavigationPointSet "$MSG" > /dev/null 2>&1
    sleep 2

    pkill -f "lib/istrorsx_core/$node_name" 2>/dev/null || true
    sleep 1

    # log4cxx rotates per minute -- grep every file, not just the active one
    # (doc/ai/06_hw_testing_guide.md § 3).
    if grep -h "navigation_point_set" logout/istro_rt20*.log 2>/dev/null \
         | awk -v t="$logmark" '$0 >= t' | grep -q "idx=$POINT_IDX"; then
        echo "  OK   $node_name applied the update"
        return 0
    fi
    echo "  FAIL $node_name did NOT apply the update"
    return 1
}

RESULT=0
check_node navigation_node -nowait -path E3 || RESULT=1
check_node save_node       -nowait          || RESULT=1

echo
echo "RESULT (NavigationPointSet -> navigationPoint[] in both processes):" \
     "$([ $RESULT -eq 0 ] && echo OK || echo FAIL)"
exit $RESULT
