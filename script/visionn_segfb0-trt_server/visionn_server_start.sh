#!/bin/bash
# Starts visionn_server.py in the background. Paths are resolved relative to
# this script's own location, not hardcoded to any user's home directory --
# this dev VM's /home/ubuntu won't match the final Jetson's /home/istrobotics.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGOUT_DIR="$SCRIPT_DIR/../../logout"

cd "$SCRIPT_DIR"

# Prefer the road-segmentation training project's own .venv_trt (dev/training
# machine) when present, falling back to plain system python3 otherwise (the
# Jetson target, where tensorrt/cuda-python/cv2 are installed system-wide via
# JetPack -- see doc/jetson-tensorrt-inference.md -- and this training
# project isn't deployed at all). Invoking the venv's python3 binary directly
# has the same effect as "source .venv_trt/bin/activate" without needing a
# shell-level activation step, matching how ../visionn_segfb0_server/
# visionn_server_start.sh already picks its own .venv_rosvm/bin/python3.
TRT_VENV_PY="$HOME/pablo_trenovanie_NN-mix/road-segmentation-20260314/.venv_trt/bin/python3"
if [ -x "$TRT_VENV_PY" ]; then
    PYTHON_BIN="$TRT_VENV_PY"
else
    PYTHON_BIN="python3"
fi

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
