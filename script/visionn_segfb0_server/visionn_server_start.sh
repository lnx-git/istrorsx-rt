#!/bin/bash
# Starts visionn_server.py in the background. Paths are resolved relative to
# this script's own location, not hardcoded to any user's home directory --
# this dev VM's /home/ubuntu won't match the final Jetson's /home/istrobotics.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGOUT_DIR="$SCRIPT_DIR/../../logout"

cd "$SCRIPT_DIR"

# VISIONN_MIN_PERIOD=<ms> -> throttle inference: run model_predict() at most
# once per that many milliseconds, answering anything in between with the
# previous mask (see visionn_server.py's own comment for why it answers
# rather than drops).
#
# This is the CPU/ONNX server, so it is throttled by default: one predict
# costs ~330 ms here and vision_node asks at the camera's rate, which pegs a
# core and starves everything else on the box. Server CPU works out at
# roughly predict_ms / VISIONN_MIN_PERIOD, so 1000 ms -> ~33%.
#
# "${VISIONN_MIN_PERIOD:-1000}" so an explicit value from the caller still
# wins:  VISIONN_MIN_PERIOD=500 ./visionn_server_start.sh
#
# Set it to 0 for the original, unthrottled behaviour -- which is what the
# Jetson wants, and what the segfb0-trt server does (it has no throttle at
# all; TensorRT inference is fast enough that this problem does not arise).
#export VISIONN_MIN_PERIOD=0
export VISIONN_MIN_PERIOD="${VISIONN_MIN_PERIOD:-1000}"

# VISIONN_FOREGROUND=1 -> run in the foreground, logging to stdout instead of
# the log file. Used by istrorsx_bringup's full.launch.py: ros2 launch has to
# own the server as a real child process, or it can neither show its output
# nor kill it on shutdown (a backgrounded script exits immediately, leaving
# visionn_server.py orphaned). Manual use is unchanged -- without this
# variable set, the script backgrounds and logs exactly as before.
if [ "$VISIONN_FOREGROUND" = "1" ]; then
    exec "$SCRIPT_DIR/.venv_rosvm/bin/python3" ./visionn_server.py
fi

date -Iseconds >> "$LOGOUT_DIR/visionn_server2.log"
"$SCRIPT_DIR/.venv_rosvm/bin/python3" ./visionn_server.py >> "$LOGOUT_DIR/visionn_server2.log" 2>&1 &
ps -ef | grep visionn_server.py | grep -v grep
