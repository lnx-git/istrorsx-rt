#!/bin/bash
# Starts the NN inference server in the background, before _full_launch.sh
# (which runs with launch_visionn_server:=false and expects it already up).
#
#   ./_visionn_server_run.sh
#
# Variant: set VISIONN_DIR below. segfb0-trt (SegFormer-B0, TensorRT) is what
# Robotour 2026 was driven with; roadyolo-trt (YOLO11s-seg) is the alternative.
# Log: logout/visionn_server.log and visionn_server2.log. Stop: $VISIONN_DIR/visionn_server_stop.sh
#
# No _setup_env.sh here on purpose: the server is plain Python, not a ROS node.

VISIONN_DIR=script/visionn_segfb0-trt_server

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# a second instance would fight the first one for the server port
if pgrep -f visionn_server.py >/dev/null; then
    echo "visionn_server.py already running -- stop it first with $VISIONN_DIR/visionn_server_stop.sh"
    ps -ef | grep visionn_server.py | grep -v grep
    exit 0
fi

echo "=== starting $VISIONN_DIR ==="
"$VISIONN_DIR/visionn_server_start.sh"
