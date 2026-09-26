#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGOUT_DIR="$SCRIPT_DIR/../../logout"

truncate -s 0 "$LOGOUT_DIR/visionn_server.log"
date -Iseconds >> "$LOGOUT_DIR/visionn_server.log"
truncate -s 0 "$LOGOUT_DIR/visionn_server2.log"
date -Iseconds >> "$LOGOUT_DIR/visionn_server2.log"
