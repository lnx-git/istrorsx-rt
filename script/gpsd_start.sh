#!/bin/bash
# NOTE (2026-08-23 port to Ubuntu 24.04 / ROS 2 Jazzy): original used `python`,
# which doesn't exist on this distro (only `python3`).
sudo killall gpsd
sudo rm /var/run/gpsd.sock
sudo systemctl stop gpsd.socket
ps -ef|grep gps|grep -v grep
#sudo gpsd -b `python3 script/gps_port.py` -F /var/run/gpsd.sock
sudo gpsd -b /dev/ttyUSB_GPS -F /var/run/gpsd.sock
ps -ef|grep gps|grep -v grep
gpspipe -r -n 20
