#!/bin/bash

# ---------------------------------------------------------------------------
# Original install notes (from doc/istrobtx_2025/script/install/rplidar_install.txt):
# Kept for historical reference only -- the download URL below is dead and the
# API (RPlidarDriver, IS_OK/IS_FAIL, rplidar_response_measurement_node_t) is
# from SDK v1.6.1 / v1.11.0, superseded by the current SDK's sl:: namespace.
#
# 17.1.2020
#
# Pre Jetson Nano sme pouzili "RPLIDAR SDK" v1.11.0, ale pri kompilacii vypisuje vela depredecated funkcii,
# kvoli zavedeniu noveho datoveho typu pre vysledky lidar merani "rplidar_response_measurement_node_hq_t".
# Interne v SDK je funkcia convert(), ktorou by sa dal urobit prevod.
#
#
# 28.6.2018
#
# 1. instalacia "RPLIDAR SDK" v1.6.1 zo stranky "https://www.slamtec.com/en/Support#rplidar-a3"
#      cd /usr/local/
#      sudo mkdir rplidar
#      cd /usr/local/rplidar/
#      sudo wget http://bucket.download.slamtec.com/5a399ca373918716521c3c48680e63b6b0aaaf22/rplidar_sdk_v1.6.1.zip
#      sudo unzip rplidar_sdk_v1.6.1.zip
#      cd /usr/local/rplidar/sdk/
#      sudo make
#      cd /usr/local/rplidar/sdk/sdk/include/
#      sudo mkdir hal
#      cd /usr/local/rplidar/sdk/sdk/include/hal/
#      sudo cp /usr/local/rplidar/sdk/sdk/src/hal/types.h .
#
# 2. najnovsia verzia "RPLIDAR SDK" vyzadovala urobit tieto zmeny v "lidar.cpp":
#      - zmenit "CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT)" na "CreateDriver(DRIVER_TYPE_SERIALPORT)"
#      - zmenit "startScan()" na "startScan(0, 1)"
# ---------------------------------------------------------------------------

# Updated for Ubuntu 24.04 LTS (Noble) / ROS 2 Jazzy (see doc/ai/04_installation.md).
#
# The current Slamtec SDK (github.com/Slamtec/rplidar_sdk, v2.x) has no apt
# package and ships no pkg-config/.pc file, so it's built from source and
# kept at a fixed system path (/usr/local/rplidar_sdk) instead. The lidar_node
# CMakeLists.txt will reference RPLIDAR_SDK_DIR below directly for its include
# path and static library, the same way the old build pointed at
# /usr/local/rplidar/sdk/.
#
# NOTE: the SDK's API changed significantly since v1.6.1 -- it now lives under
# the "sl::" namespace (sl::createLidarDriver(), SL_IS_OK/SL_IS_FAIL,
# sl_lidar_response_measurement_node_hq_t, channel-based connect()) instead of
# the old free-standing RPlidarDriver/IS_OK/IS_FAIL API. Porting
# lidar_rplidar.cpp will need to follow this new API -- see the migration
# guide (doc/ai/05_migration_guide.md) once that step starts.
set -e

RPLIDAR_SDK_DIR=/usr/local/rplidar_sdk

sudo apt-get update
sudo apt-get install -y git build-essential

if [ -d "$RPLIDAR_SDK_DIR/.git" ]; then
    sudo git -C "$RPLIDAR_SDK_DIR" pull
else
    sudo git clone https://github.com/Slamtec/rplidar_sdk.git "$RPLIDAR_SDK_DIR"
fi

sudo make -C "$RPLIDAR_SDK_DIR/sdk"
