#!/bin/bash
# Starts visionn_server.py in the background. Paths are resolved relative to
# this script's own location, not hardcoded to any user's home directory --
# this dev VM's /home/ubuntu won't match the final Jetson's /home/istrobotics.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGOUT_DIR="$SCRIPT_DIR/../../logout"

cd "$SCRIPT_DIR"

# Reuses ../visionn_segfb0_server/.venv_rosvm instead of a venv of this
# variant's own: both are CPU/ONNX servers and need the same three packages
# (numpy, opencv, onnxruntime), so a second venv would only be another 200 MB
# to keep in step. Same arrangement as the two TensorRT variants, which share
# .venv_trt. On a fresh VM create it once with
# ../visionn_segfb0_server/install/setup_cpu_infer_venv.sh; the fallback to
# plain python3 works wherever those packages are installed system-wide.
ORT_VENV_PY="$SCRIPT_DIR/../visionn_segfb0_server/.venv_rosvm/bin/python3"
if [ -x "$ORT_VENV_PY" ]; then
    PYTHON_BIN="$ORT_VENV_PY"
else
    PYTHON_BIN="python3"
fi

# VISIONN_MIN_PERIOD=<ms> -> throttle inference: run model_predict() at most
# once per that many milliseconds, answering anything in between with the
# previous mask (see visionn_server.py's own comment for why it answers
# rather than drops).
#
# Throttled by default for the same reason as ../visionn_segfb0_server: this is
# CPU inference, one predict costs a few hundred ms here and vision_node asks
# at the camera's rate, which pegs a core. Server CPU works out at roughly
# predict_ms / VISIONN_MIN_PERIOD.
#
# Set it to 0 for unthrottled behaviour -- which is what a bare model
# comparison wants, and what the Jetson variants do (they have no throttle at
# all; TensorRT inference is fast enough that this problem does not arise):
#   VISIONN_MIN_PERIOD=0 ./visionn_server_start.sh
export VISIONN_MIN_PERIOD="${VISIONN_MIN_PERIOD:-1000}"

# VISIONN_FOREGROUND=1 -> run in the foreground, logging to stdout instead of
# the log file. Used by istrorsx_bringup's full.launch.py -- see the identical
# block in ../visionn_segfb0_server/visionn_server_start.sh for why. Manual
# use is unchanged without this variable set.
if [ "$VISIONN_FOREGROUND" = "1" ]; then
    exec "$PYTHON_BIN" ./visionn_server.py
fi

date -Iseconds >> "$LOGOUT_DIR/visionn_server2.log"
"$PYTHON_BIN" ./visionn_server.py >> "$LOGOUT_DIR/visionn_server2.log" 2>&1 &
ps -ef | grep visionn_server.py | grep -v grep
