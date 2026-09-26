#!/bin/bash
# One-off GPS module configuration: restrict NMEA output to RMC sentences and
# disable the navigation speed threshold. Needed once per fresh GPS module or
# after a factory reset -- NOT part of the normal start sequence (that is
# gpsd_start.sh).
#
#   ./script/gpsd_init.sh
#
# Run this with gpsd STOPPED (script/gpsd_stop.sh) -- gpsd holds the serial
# port open and the writes below would not reach the module.
#
# Thin wrapper around gpsd_init.py, which finds the GPS by USB VID:PID rather
# than by device node, writes the two PMTK commands and prints the module's
# replies. The .py has no shebang and is not executable on purpose: it is
# always run through this script, so there is one entry point and one place
# where the interpreter is named.
#
# The previous shell-only implementation is kept as gpsd_init_old.sh -- see
# its own header for when that one is still the better tool.
set -e

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 gpsd_init.py "$@"
