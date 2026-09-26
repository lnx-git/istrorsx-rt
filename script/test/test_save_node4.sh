#!/bin/bash
# save_node test 4/4: real sample image through a *real* vision_node (backed
# by a real visionn_server/NN, same CPU/ONNX server test_vision_node.sh
# itself uses) -> a real vision.jpg overlay on disk. No planner_node/wmodel
# -- see test_save_node4.py's own comment.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting visionn_server (CPU/ONNX)..."
(cd script/visionn_segfb0_server && .venv_rosvm/bin/python visionn_server.py) > /tmp/test_save_node4_visionn_server.log 2>&1 &
SERVER_PID=$!
sleep 3

echo "Starting vision_node..."
ros2 run istrorsx_core vision_node -- -nowait > /tmp/test_save_node4_vision_node.log 2>&1 &
VISION_PID=$!
sleep 2

echo "Starting save_node..."
ros2 run istrorsx_core save_node -- -nowait > /tmp/test_save_node4_save_node.log 2>&1 &
SAVE_PID=$!
sleep 2

python3 script/test/test_save_node4.py && RESULT=0 || RESULT=$?

kill "$SAVE_PID" 2>/dev/null || true
wait "$SAVE_PID" 2>/dev/null || true
pkill -f "lib/istrorsx_core/save_node" 2>/dev/null || true

kill "$VISION_PID" 2>/dev/null || true
wait "$VISION_PID" 2>/dev/null || true
pkill -f "lib/istrorsx_core/vision_node" 2>/dev/null || true

kill "$SERVER_PID" 2>/dev/null || true
wait "$SERVER_PID" 2>/dev/null || true

echo "save_node log: /tmp/test_save_node4_save_node.log"
echo "vision_node log: /tmp/test_save_node4_vision_node.log"
echo "visionn_server log: /tmp/test_save_node4_visionn_server.log (also logout/visionn_server.log)"
exit $RESULT
