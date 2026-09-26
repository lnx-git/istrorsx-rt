#!/bin/bash
# vision_node test 2/2: same as test_vision_node.sh (CPU/ONNX visionn_server +
# vision_node + a simulated CameraFrame), but with save_node ALSO running
# alongside vision_node -- see test_vision_node2.py's own comment for why
# (isolating vision_node's own health from save_node's separate handling of
# the same VisionDebugData).
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting visionn_server (CPU/ONNX)..."
(cd script/visionn_segfb0_server && .venv_rosvm/bin/python visionn_server.py) > /tmp/test_vision_node2_visionn_server.log 2>&1 &
SERVER_PID=$!
sleep 3

echo "Starting vision_node..."
ros2 run istrorsx_core vision_node -- -nowait > /tmp/test_vision_node2_vision_node.log 2>&1 &
VISION_PID=$!
sleep 2

echo "Starting save_node TOO (this is the combination test_save_node4 fails under)..."
ros2 run istrorsx_core save_node -- -nowait > /tmp/test_vision_node2_save_node.log 2>&1 &
SAVE_PID=$!
sleep 2

python3 script/test/test_vision_node2.py && RESULT=0 || RESULT=$?

kill "$SAVE_PID" 2>/dev/null || true
wait "$SAVE_PID" 2>/dev/null || true
pkill -f "lib/istrorsx_core/save_node" 2>/dev/null || true

kill "$VISION_PID" 2>/dev/null || true
wait "$VISION_PID" 2>/dev/null || true
pkill -f "lib/istrorsx_core/vision_node" 2>/dev/null || true

kill "$SERVER_PID" 2>/dev/null || true
wait "$SERVER_PID" 2>/dev/null || true

echo "vision_node log: /tmp/test_vision_node2_vision_node.log"
echo "save_node log: /tmp/test_vision_node2_save_node.log"
echo "visionn_server log: /tmp/test_vision_node2_visionn_server.log (also logout/visionn_server.log)"
exit $RESULT
