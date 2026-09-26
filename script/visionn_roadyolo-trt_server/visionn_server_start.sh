#!/bin/bash
# Starts visionn_server.py in the background. Paths are resolved relative to
# this script's own location, not hardcoded to any user's home directory --
# this dev VM's /home/ubuntu won't match the final Jetson's /home/istrobotics.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGOUT_DIR="$SCRIPT_DIR/../../logout"

cd "$SCRIPT_DIR"

# Reuses the SegFormer-TRT project's own .venv_trt rather than a venv of this
# variant's own -- this is NOT merely a dev-machine fallback here, it is
# currently the only interpreter on this box with cuda-python installed:
# plain system python3 was checked directly on this Jetson (2026-09-12) and
# does NOT have the "cuda" module (JetPack ships tensorrt system-wide, but
# cuda-python is a separate manual `pip install cuda-python==13.2.0` step --
# see doc/ai/01_architecture.md -- and that step was only ever run inside
# .venv_trt, not against the bare system interpreter). Falls back to plain
# python3 only if .venv_trt is ever removed/moved, which would need that pip
# install repeated against the fallback for this variant to actually start.
# Invoking the venv's python3 binary directly has the same effect as "source
# .venv_trt/bin/activate" without a shell-level activation step, matching how
# ../visionn_segfb0_server/visionn_server_start.sh picks its own
# .venv_rosvm/bin/python3 and ../visionn_segfb0-trt_server/ picks this same
# .venv_trt.
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
