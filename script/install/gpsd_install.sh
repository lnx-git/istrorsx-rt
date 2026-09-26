#!/bin/bash

# ---------------------------------------------------------------------------
# Original install script (from doc/istrobtx_2025/script/install/gpsd_install.sh,
# written for an older Ubuntu release). Kept for historical reference — do not
# run as-is on Ubuntu 24.04 (Noble): "python-gps" is the Python 2 package name
# and no longer exists under that name.
#
# sudo apt-get install gpsd gpsd-clients python-gps
# sudo systemctl stop gpsd.socket
# sudo systemctl disable gpsd.socket
# sudo apt-get install libgps-dev
# ---------------------------------------------------------------------------

# Updated for Ubuntu 24.04 LTS (Noble) / ROS 2 Jazzy (see doc/ai/04_installation.md).
set -e

sudo apt-get update

# gpsd + client tools (gpsd-clients provides gpspipe, which _gpsd_run.sh uses to record the raw
# NMEA stream to logout/gpspipe.log); libgps-dev for the C++ gpsd client (libgpsmm, used by
# gps_node/GpsDevice); python3-gps and python3-serial for the scripts in this
# script/ directory (gpsd_init.py, gps_port.py use pyserial; python3-gps is the
# Python 3 successor to the original script's "python-gps").
sudo apt-get install -y gpsd gpsd-clients libgps-dev python3-gps python3-serial

# gpsd's default systemd socket-activation doesn't know which serial device to
# watch, and conflicts with the manual "gpsd -b <device> -F /var/run/gpsd.sock"
# invocations used by gpsd_start*.sh / gpsd_boot.sh (../gpsd_start.sh etc.) —
# disable it so those scripts have full control over when/how gpsd runs.
sudo systemctl stop gpsd.socket
sudo systemctl disable gpsd.socket
