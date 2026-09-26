#!/bin/bash
# Manual end-to-end test for vision_node: starts the CPU/ONNX visionn_server
# (script/visionn_segfb0_server/ -- vision_node's Vision::init()/VisioNN::init()
# both need it up before the node can even initialize), starts vision_node,
# then runs test_vision_node.py (a real CameraFrame message can't practically
# be built via "ros2 topic pub" YAML -- it needs rclpy) which publishes one
# simulated front-camera frame and checks that /robot/vision_front_data and
# /robot/vision_front_debug_data both come back.
# Nothing like this existed in the legacy single-process app -- there was no
# equivalent of publishing synthetic bus traffic to exercise one thread in
# isolation, since every thread just read whatever the real hardware/other
# threads had last written into the shared DataSet.
set -e

cd "$(dirname "$0")/../.."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting visionn_server (CPU/ONNX)..."
(cd script/visionn_segfb0_server && .venv_rosvm/bin/python visionn_server.py) > /tmp/test_vision_node_server.log 2>&1 &
SERVER_PID=$!
sleep 3

echo "Starting vision_node..."
ros2 run istrorsx_core vision_node -- -nowait > /tmp/test_vision_node.log 2>&1 &
NODE_PID=$!
sleep 2

echo "Publishing a simulated CameraFrame and checking the response..."
# "&& RESULT=0 || RESULT=$?", not a plain "RESULT=$?" after -- under set -e,
# a non-zero exit here would abort the script immediately, before the
# assignment (and before the kill/cleanup below) ever runs.
python3 script/test/test_vision_node.py && RESULT=0 || RESULT=$?

kill "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true
# "ros2 run"'s own PID ($NODE_PID above) isn't reliably the actual vision_node
# binary's PID -- pkill -f as a robustness fallback so a killed wrapper
# process doesn't leave the real node running behind it.
pkill -f "lib/istrorsx_core/vision_node" 2>/dev/null || true
kill "$SERVER_PID" 2>/dev/null || true
wait "$SERVER_PID" 2>/dev/null || true

echo "vision_node log: /tmp/test_vision_node.log"
echo "visionn_server log: /tmp/test_vision_node_server.log (also logout/visionn_server.log)"
exit $RESULT
