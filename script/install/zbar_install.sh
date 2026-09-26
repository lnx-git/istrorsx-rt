#!/bin/bash

# ---------------------------------------------------------------------------
# Original install script (from doc/istrobtx_2025/script/install/zbar_install.sh):
#
# sudo apt-get install libzbar-dev libzbar0
# ---------------------------------------------------------------------------

# Still current on Ubuntu 24.04 LTS (Noble) / ROS 2 Jazzy -- no change needed.
# Used by QRScanner (qrscan.cpp, #include <zbar.h>), part of vision_node.
set -e

sudo apt-get update
sudo apt-get install -y libzbar-dev libzbar0
