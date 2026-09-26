#!/bin/bash
# also ends the raw logout/gpspipe.log recording started by _gpsd_run.sh
killall gpspipe 2>/dev/null
sudo killall gpsd
sudo rm /var/run/gpsd.sock
sudo systemctl stop gpsd.socket
ps -ef|grep gps|grep -v grep
