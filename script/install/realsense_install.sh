#!/bin/bash

# ---------------------------------------------------------------------------
# Original install notes, kept for historical reference only:
#
# From doc/istrobtx_2025/script/install/realsense_install.txt:
#   Navod na isntalaciu pre Ubuntu:
#   https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
#
# From doc/istrobtx_2025/doc/191114_camera_realsense.txt (2019-11-14 email):
#   Na FTPcku v /robotour/robotour2019/MakerFaireBratislava/realsense/ je
#   zdrojak v C-cku, ktory otvori farbnu kameru + zobrazuje a zapisuje obrazok.
#   Navod na instalaciu pre Ubuntu:
#   https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
#
# Neither pointed at a specific apt repo or package set -- just the upstream
# install doc. See doc/obsidian/"Reinstalacia OS 256GB nvme"/"12 Intel
# Realsense.md" for the actual up-to-date repo setup this script follows
# (RealSense was spun off from Intel into its own company -- new apt domain,
# librealsenseai.com, replacing the old librealsense.intel.com).
# ---------------------------------------------------------------------------

# Updated for Ubuntu 24.04 LTS (Noble) / ROS 2 Jazzy (see doc/ai/04_installation.md).
#
# Installs librealsense2 from Intel/RealSense AI's own apt repo (signed,
# GPG-keyed) rather than building from source -- much faster and lighter than
# a from-source build, which matters on this VM (3.7 GiB RAM, no swap; a full
# librealsense2 `make -j$(nproc)` build is a real OOM risk here, see
# doc/ai/03_progress.md's colcon --parallel-workers 1 workaround for a taste
# of how tight memory is).
#
# librealsense2-dev ships a proper CMake config package (unlike libgps, which
# needed pkg-config) -- a future camera_node's CMakeLists.txt can use
# find_package(realsense2 REQUIRED) directly.
set -e

sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
    | sudo gpg --yes --dearmor -o /etc/apt/keyrings/librealsenseai.gpg

echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update
sudo apt-get install -y librealsense2-utils librealsense2-dev librealsense2-dbg

# Verify (requires an attached RealSense camera -- not run automatically here):
#   rs-enumerate-devices
#   realsense-viewer

# ---------------------------------------------------------------------------
# Fallback, only if librealsense2-utils installed but realsense-viewer is
# missing (seen on arm64/Jetson Orin NX per a GitHub issue -- check first
# with `dpkg -L librealsense2-utils | grep realsense-viewer`). Builds from
# source with the RSUSB backend, still without touching the kernel. NOT run
# by this script -- on this VM's limited RAM, prefer running it manually and
# watching for OOM (see the note above) rather than letting it run unattended:
#
#   git clone https://github.com/IntelRealSense/librealsense.git
#   cd librealsense
#   sudo apt-get install -y git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev cmake build-essential
#   mkdir build && cd build
#   cmake .. -DFORCE_RSUSB_BACKEND=true -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release
#   make -j$(nproc)
#   sudo make install
#   sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
#   sudo udevadm control --reload-rules && sudo udevadm trigger
#
# (Add -DBUILD_WITH_CUDA=true instead if a CUDA dev-kit is installed, for GPU-accelerated point clouds.)
# ---------------------------------------------------------------------------
