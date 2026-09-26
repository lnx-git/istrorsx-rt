#!/bin/bash
# SUPERSEDED by gpsd_init.sh, which runs gpsd_init.py instead. Kept because it
# is the shell-only path: it needs nothing but pyserial's miniterm, and it
# ends in an interactive terminal on the GPS, which is the only way to type
# the "$PMTK447*35" check by hand or to send a command this pair of scripts
# doesn't cover. The Python version does the same two configuration writes
# non-interactively and prints the module's replies.
#
# NOTE (2026-08-23 port to Ubuntu 24.04 / ROS 2 Jazzy): original used `python`
# and `miniterm.py`, neither of which exist on this distro (only `python3`,
# and pyserial's Debian package ships the tool as `pyserial-miniterm`).
echo 'PMTK_API_SET_NMEA_OUTPUT - only RMC sentences...'
echo -ne '$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n' > `python3 gps_port.py`
sleep 1
echo 'PMTK_SET_Nav Speed threshold - disable...'
echo -ne '$PMTK386,0*23\r\n' > `python3 gps_port.py`
sleep 1
echo 'please check if GPS is sending only "$GNRMC,"'
echo 'use "$PMTK447*35" + Enter to check navigation speed threshold (expected response: "$PMTK527,0.00*00")'
pyserial-miniterm `python3 gps_port.py` 4800
