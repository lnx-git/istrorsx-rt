#!/bin/bash
# save_node test 3/4: real sample image -> camera.jpg (no depth, no
# vision_node). Only save_node needs to run -- no planner_node/wmodel, see
# test_save_node3.py's own comment.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting save_node..."
ros2 run istrorsx_core save_node -- -nowait > /tmp/test_save_node3.log 2>&1 &
NODE_PID=$!
sleep 2

python3 script/test/test_save_node3.py && RESULT=0 || RESULT=$?

kill "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true
pkill -f "lib/istrorsx_core/save_node" 2>/dev/null || true

echo "save_node log: /tmp/test_save_node3.log"
exit $RESULT
