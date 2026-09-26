#!/bin/bash
# Runs the C++ camera generator, building it first if needed.
#
#   ./script/test/test_camera_gen_cpp.sh                  # cesta, front, 5 FPS
#   ./script/test/test_camera_gen_cpp.sh trava front 5
#   ./script/test/test_camera_gen_cpp.sh cesta rear 30
#
# Same arguments and same output as test_camera_gen.sh's Python original, so
# the two can be swapped and their per-tick numbers compared directly. Run one,
# then the other, with the same subscribers attached: if only the Python one
# stalls, the interpreter is involved; if both do, it is not.
#
# The build is a plain g++ call outside colcon, so it never touches build/ or
# install/ and is safe to run while a test is in progress.
set -e

cd "$(dirname "$0")/../.."
BIN=/tmp/test_camera_gen_cpp
SRC=script/test/test_camera_gen_cpp.cpp

# Rebuild when the binary is missing (a reboot clears /tmp) or older than the
# source.
if [ ! -x "$BIN" ] || [ "$SRC" -nt "$BIN" ]; then
    echo "building $BIN ..."
    ./script/test/test_camera_gen_cpp_build.sh
fi

source /opt/ros/jazzy/setup.bash
source install/setup.bash

# The workspace root is passed explicitly: the generator reads sample/ and
# parses vision_depth.cpp out of it, and must not depend on the caller's cwd.
exec "$BIN" "${1:-cesta}" "${2:-front}" "${3:-5}" "$PWD"
