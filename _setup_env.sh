# This file must be SOURCED, not executed:
#   source _setup_env.sh
#   . _setup_env.sh
#
# Running it directly (./_setup_env.sh) has no effect: a script run as its own
# process cannot change the environment of the shell that launched it -- that
# is exactly how `source install/_setup.bash` (and this wrapper around it)
# works. See doc/ai/04_installation.md.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "ERROR: _setup_env.sh must be sourced, not executed. Run: source ${0}" >&2
    exit 1
fi

_SETUP_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "${_SETUP_ENV_DIR}/install/setup.bash"

# Fast DDS shared-memory sizing. CameraFrame is 1735680 B against a 512 KB
# default SHM segment, which blocks the publisher under RELIABLE QoS -- see
# conf/fastdds_large_msg.xml and doc/ai/03_progress.md. Read by each process
# when it creates its DDS participant, so it has to be in the environment of
# every node: sourcing this file once per terminal covers that.
# Set FASTDDS_NO_LARGE_MSG_PROFILE=1 before sourcing to skip it.
if [[ -z "${FASTDDS_NO_LARGE_MSG_PROFILE}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${_SETUP_ENV_DIR}/conf/fastdds_large_msg.xml"
fi

unset _SETUP_ENV_DIR
