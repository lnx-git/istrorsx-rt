#!/bin/bash
# Exports an OSM map to KML + C++ navmap data through navigation_node's
# /robot/navmap_export service (port of legacy navmap_test()).
#
#   script/navmap_export.sh navmap-praha-stromovka
#
# Reads conf/<map>.osm, writes out/<map>.kml and out/<map>.cpp. If no
# navigation_node is running, one is started from the workspace root just for
# the export and stopped again -- the legacy "compile, run, quit" step of
# doc/istrobtx_2025/doc/190317_nova_mapa.txt. Its log goes to
# logout/istro_<EVENT_TAG>.log as usual; grep it for "navmap_load" to see what was
# loaded or rejected. See doc/ai/01_architecture.md § navigation_node.
#
# ISTRO_WS overrides the workspace root (default: this script's parent dir).

MAP="${1%.osm}"
if [ -z "$MAP" ]; then
    echo "usage: $0 <map_name>     e.g. $0 navmap-praha-stromovka" >&2
    exit 2
fi

WS="${ISTRO_WS:-$(cd "$(dirname "$0")/.." && pwd)}"
INSTALL="${ISTRO_INSTALL:-$WS/install}"
# log file name follows EVENT_TAG (event_defs.h), which conf/log4cxx.xml must match
TAG=$(grep -o 'define EVENT_TAG "[^"]*"' "$(dirname "$0")/../src/istrobtx/include/event_defs.h" | cut -d'"' -f2)
LOG="$WS/logout/istro_${TAG:-rt2026}.log"
SERVICE=/robot/navmap_export

if [ ! -r "$WS/conf/$MAP.osm" ]; then
    echo "error: $WS/conf/$MAP.osm not found" >&2
    exit 1
fi
mkdir -p "$WS/out" "$WS/logout"

source /opt/ros/jazzy/setup.bash
source "$INSTALL/setup.bash"

have_service() {
    ros2 service list 2>/dev/null | grep -qx "$SERVICE"
}

NODE_PID=""
if have_service; then
    echo "using the running navigation_node"
else
    NODE_BIN="$INSTALL/istrorsx_core/lib/istrorsx_core/navigation_node"
    if [ ! -x "$NODE_BIN" ]; then
        echo "error: $NODE_BIN not found -- build istrorsx_core or set ISTRO_INSTALL" >&2
        exit 1
    fi
    echo "starting navigation_node for the export..."
    ( cd "$WS" && exec "$NODE_BIN" --ros-args -- -nowait ) >/dev/null 2>&1 &
    NODE_PID=$!
    for i in $(seq 1 30); do
        have_service && break
        if ! kill -0 "$NODE_PID" 2>/dev/null; then
            echo "error: navigation_node exited during startup, see $LOG" >&2
            exit 1
        fi
        sleep 1
    done
    if ! have_service; then
        echo "error: $SERVICE did not appear within 30 s" >&2
        kill -INT "$NODE_PID" 2>/dev/null; wait "$NODE_PID" 2>/dev/null
        exit 1
    fi
fi

# a large map blocks navigation_node for a few seconds while it loads
OUT=$(timeout 180 ros2 service call "$SERVICE" istrorsx_core/srv/NavMapExport "{map_name: '$MAP'}" 2>&1)
RES=$?

if [ -n "$NODE_PID" ]; then
    kill -INT "$NODE_PID" 2>/dev/null
    wait "$NODE_PID" 2>/dev/null
fi

RESPONSE=$(echo "$OUT" | grep 'NavMapExport_Response')
if [ $RES -ne 0 ] || [ -z "$RESPONSE" ]; then
    echo "error: service call failed (exit $RES)" >&2
    echo "$OUT" >&2
    exit 1
fi
echo "$RESPONSE" | sed 's/.*NavMapExport_Response(//; s/)$//; s/, \([a-z_]*=\)/\n  \1/g; s/^/  /'

if echo "$RESPONSE" | grep -q 'success=True'; then
    ls -la "$WS/out/$MAP.kml" "$WS/out/$MAP.cpp"
    if ! echo "$RESPONSE" | grep -q 'component_cnt=1,'; then
        echo "WARNING: graph is not connected -- one line per disconnected component:"
        tac "$LOG" | awk '/navmap_load\(\): msg="loading started/ {exit} {print}' | tac \
            | grep -a 'not connected to the main graph' | sed 's/.*handleNavMapExport(): msg="[^"]*", /  /'
    fi
    exit 0
fi
exit 1
