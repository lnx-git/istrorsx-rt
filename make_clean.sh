#!/bin/bash
# Removes everything colcon/CMake regenerates on the next "colcon build"
# (build/, install/, log/) plus this project's own runtime log/output dirs
# (logout/, out/ -- re-created empty afterward, since node startup code
# expects them to already exist and does not create them itself, see
# doc/ai/04_installation.md) and telemetry_node's ramdisk/ output (NOT
# re-created: telemetry_node makes it itself on startup). Intended to be run before making a backup of
# the whole workspace, to keep it as small as possible.
#
# Does NOT touch src/ (source code), conf/, or anything under script/
# (e.g. script/visionn_segfb0_server/.venv_rosvm, model files) -- those
# either aren't build output or aren't trivially regenerable (a venv needs
# network access to reinstall; models aren't rebuilt by "colcon build").
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

print_sizes() {
    for d in build install log logout out ramdisk; do
        if [ -d "$d" ]; then
            du -sh "$d"
        else
            echo "0	$d (does not exist)"
        fi
    done
}

echo "Before:"
print_sizes

rm -rf build install log
rm -rf logout out ramdisk
mkdir -p logout out

echo "After:"
print_sizes

echo "Done. Run colcon build again before using any node."
