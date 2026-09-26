#!/bin/bash

# ---------------------------------------------------------------------------
# Original install script (from doc/istrobtx_2025/script/install/geolib_install.sh):
# built GeographicLib 1.46 from source. Kept for historical reference only --
# Ubuntu 24.04 (Noble) ships a packaged version, no need to build from source.
#
# wget https://sourceforge.net/projects/geographiclib/files/distrib/GeographicLib-1.46.tar.gz
# tar xfpz GeographicLib-1.46.tar.gz
# cd GeographicLib-1.46
# mkdir BUILD
# cd BUILD
# cmake ..
# make
# make test
# sudo make install
# ---------------------------------------------------------------------------

# Updated for Ubuntu 24.04 LTS (Noble) / ROS 2 Jazzy (see doc/ai/04_installation.md).
# Provides both a pkg-config module ("geographiclib") and a CMake find-module
# (FindGeographicLib.cmake) -- used by GeoCalc (geocalc.cpp) via pkg-config,
# see CMakeLists.txt.
set -e

sudo apt-get update
sudo apt-get install -y libgeographiclib-dev
