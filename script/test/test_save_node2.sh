#!/bin/bash
# save_node test 2/4: synthetic LidarData -> lidar.png. Only save_node needs
# to run -- "-lidar /dev/null" just sets conf.useLidar=1 (save_node never
# opens the device itself, it only reads LidarData.msg), no planner_node/
# wmodel, see test_save_node2.py's own comment.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting save_node (-lidar /dev/null, so conf.useLidar=1)..."
ros2 run istrorsx_core save_node -- -nowait -lidar /dev/null > /tmp/test_save_node2.log 2>&1 &
NODE_PID=$!
sleep 2

python3 script/test/test_save_node2.py && RESULT=0 || RESULT=$?

kill "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true
pkill -f "lib/istrorsx_core/save_node" 2>/dev/null || true

echo "save_node log: /tmp/test_save_node2.log"
exit $RESULT
