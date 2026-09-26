#!/bin/bash
# NOTE: legacy's own restart script called "./visionn_server_stop.sh", but the
# actual legacy file was named "visionn_server__stop.sh" (double underscore)
# -- a pre-existing legacy bug, would have failed with "No such file or
# directory". Fixed here by using one consistent single-underscore naming
# convention (matches this project's own *_run.sh/*_node_run.sh scripts
# elsewhere) and resolving siblings relative to this script's own location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$SCRIPT_DIR/visionn_server_stop.sh"
"$SCRIPT_DIR/visionn_server_start.sh"
