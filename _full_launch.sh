#!/bin/bash
# Starts EVERYTHING: gpsd first (needs sudo, deliberately excluded from
# istrorsx_bringup's own full.launch.py -- see that launch file's module
# docstring for why), then ros2 launch istrorsx_bringup full.launch.py
# (every ROS node + visionn_server.py + joy_node).
#
# Usage (can be run from anywhere -- cd's to its own directory, the
# workspace root, first):
#   ./_full_launch.sh                        # gpsd, then launch everything
#   ./_full_launch.sh --no-gpsd              # skip gpsd -- e.g. on a dev VM
#                                             # with no real GPS device:
#                                             # gpsd_start.sh's own final
#                                             # "gpspipe -r -n 20" check waits
#                                             # for real NMEA fixes and would
#                                             # otherwise hang forever
#   ./_full_launch.sh launch_xbox_controller:=false   # extra args pass straight
#                                             # through to full.launch.py's
#                                             # own DeclareLaunchArgument-s
#   ./_full_launch.sh --no-gpsd cb_device:=/dev/ttyUSB0
#   ./_full_launch.sh navigation_path:=E1    # point sequence for -path
#                                             # (default "*1*2*3"; the
#                                             # individual navigation_node_run.sh
#                                             # uses "E1" -- both are demo
#                                             # paths, set what the course needs)
#   ./_full_launch.sh visionn_server_variant:=segfb0-trt   # TensorRT NN server
#                                             # (the Jetson's GPU path;
#                                             # default is "segfb0", CPU/ONNX)
#
# Full argument list: ros2 launch istrorsx_bringup full.launch.py --show-args
# Currently: cb_device, cb2_device, lidar_device, navigation_path,
# launch_xbox_controller, xbox_deadzone, launch_visionn_server,
# visionn_server_variant. A MISSPELLED argument does not fall back to the
# default silently in a useful way -- check --show-args rather than assuming a
# name. (This header itself carried "visionn_variant" for a while, which would
# have left a Jetson run on the CPU/ONNX server while looking correct.)
#
# ---------------------------------------------------------------------------
# Prefer this script over calling "ros2 launch istrorsx_bringup full.launch.py"
# directly. The launch file is complete on its own as far as node arguments go
# -- verified against every *_run.sh -- but two things live here and nowhere
# else:
#
#   * gpsd, which needs sudo and is deliberately outside the launch file.
#   * "source _setup_env.sh", which sets FASTRTPS_DEFAULT_PROFILES_FILE
#     (conf/fastdds_large_msg.xml -- sizes Fast DDS's shared-memory segment for
#     the 1.74 MB CameraFrame). Nodes launched without it silently use the
#     512 KB default; nothing fails, it is just the stock configuration.
#     script/test/test_dds_shm.sh reports which one a running system got.
#
# A bare "ros2 launch" from a shell that already sourced _setup_env.sh is
# equivalent apart from gpsd. From a fresh shell it is not.
# ---------------------------------------------------------------------------
#
# Keyboard control is NOT started by any of this -- keyboard_node needs its
# own tty. Run ./keyboard_node_run.sh in a second terminal (the launch prints
# this reminder too).

set -e

# ---------------------------------------------------------------------------
# Launch parameters -- EDIT HERE, not on the command line.
#
# Deliberately a variable in the script rather than something to remember to
# type: a forgotten argument on a real drive is a wrong course or a wrong
# calibration, and nothing announces it. Anything passed on the command line
# is appended after these, so it still wins for one-off overrides.
#
# Defaults for everything not listed here come from full.launch.py's own
# DeclareLaunchArgument-s, which carry the legacy r.sh tail
# (-i DIstrobotics -vf 12 -vf2 12 -vf3 12 -vb -17 -cg 300 -ca 0).
# Full list: ros2 launch istrorsx_bringup full.launch.py --show-args
# ---------------------------------------------------------------------------
# ---- DRIVE parameters: what the robot does ------------------------------
# Every value the legacy run script set, spelled out so they can be edited
# here during testing. This is doc/istrobtx_2025/script/r.sh's own tail:
#
#   -nowait -i DIstrobotics -path E1 -vf 12 -vf2 12 -vf3 12 -vb -17 -cg 300 -ca 0
#
# (-nowait is not here: it is already on every node in full.launch.py.)
#
#   navigation_path      -path   point sequence, two characters at a time
#   ctrlboard_init_str   -i      startup string for the board: "D" is its
#                                display command, so the default writes
#                                "Istrobotics" on the display. Cosmetic.
#   velocity_fwd/2/3     -vf/2/3 forward speed offsets from VEL_ZERO
#   velocity_back        -vb     reverse speed
#   calib_gps_azimuth    -cg     needs -cb2 earlier in argv, which
#   calib_imu_yaw        -ca     full.launch.py already puts there
DRIVE_ARGS="\
navigation_path:=*1*2*3 \
ctrlboard_init_str:=DIstrobotics \
velocity_fwd:=12 \
velocity_fwd2:=12 \
velocity_fwd3:=12 \
velocity_back:=-17 \
calib_gps_azimuth:=300 \
calib_imu_yaw:=0 \
joystick_velocity_max_fwd:=17 \
"

# ---- SYSTEM parameters: what runs and on which devices ------------------
# Not from r.sh (legacy discovered its serial ports with serial_scan.py
# instead). Spelled out for the same reason as above: these are the values a
# bring-up on new hardware has to change, and a default that is merely
# correct today is not visible when it stops being correct.
#
#   cb_device / cb2_device   control board serial ports (ctrlboard_node)
#   lidar_device             RPLIDAR A1 port; also sets conf.useLidar=1 on
#                            planner/save, which gates lidar.png and the
#                            obstacle path -- see full.launch.py
#   visionn_server_variant   segfb0 = CPU/ONNX (dev VM)
#                            segfb0-trt = TensorRT, the Jetson's GPU path
#   launch_visionn_server    FALSE here on purpose: the NN server is started
#                            separately and left running, which saves its
#                            model-load wait on every relaunch and keeps a
#                            crash-restart of the ROS side from taking the
#                            inference server down with it. It must already be
#                            up before this launch, or vision_node connects to
#                            nothing:
#                              ./_visionn_server_run.sh     # Jetson, roadyolo-trt
#                              ./script/visionn_segfb0_server/visionn_server_start.sh   # dev VM
#                            Set true to have the launch own it instead.
#   launch_xbox_controller   FALSE here on purpose: driving is from the
#                            keyboard, and keyboard_node publishes the same
#                            /joy topic. It is never started by the launch
#                            (needs a real tty) -- run it in its own terminal:
#                              ./keyboard_node_run.sh
#                            Set true when a real Xbox controller is plugged
#                            in; do not run both, they share /joy.
SYSTEM_ARGS="\
cb_device:=/dev/ttyUSB_CB \
cb2_device:=/dev/ttyUSB_CB2 \
lidar_device:=/dev/lidar0 \
visionn_server_variant:=segfb0 \
launch_visionn_server:=false \
launch_xbox_controller:=false \
"

LAUNCH_ARGS="$DRIVE_ARGS $SYSTEM_ARGS"

# ---- gpsd: this script's own setting, NOT a launch argument --------------
# Kept separate deliberately: full.launch.py knows nothing about gpsd (it
# needs sudo, which is why it lives out here), so putting it among the
# arg:=value pairs would mean handing ros2 launch a name it does not know.
#
# FALSE by default, same reasoning as launch_visionn_server and
# launch_xbox_controller: gpsd is a system daemon, start it once and leave it
# up. Starting it here means a sudo password prompt in the field plus a
# kill/restart of the running service, and gpsd_start.sh ends with
# "gpspipe -r -n 20", which BLOCKS until twenty NMEA sentences arrive -- slow
# under poor sky view, forever with no GPS at all.
#
# gps_node needs it running, so start it once per session:
#   ./_gpsd_run.sh     (configures the module, starts gpsd, records logout/gpspipe.log)
# Override for a single run without editing this file:
#   ./_full_launch.sh --gpsd        # force start
#   ./_full_launch.sh --no-gpsd     # force skip
START_GPSD=false

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$_SCRIPT_DIR"

# --gpsd / --no-gpsd override START_GPSD for this run; everything else is
# passed through to the launch untouched.
_ARGS=""
for _a in "$@"; do
    case "$_a" in
        --gpsd)    START_GPSD=true ;;
        --no-gpsd) START_GPSD=false ;;
        *)         _ARGS="$_ARGS $_a" ;;
    esac
done
LAUNCH_ARGS="$LAUNCH_ARGS $_ARGS"

if [[ "$START_GPSD" == "true" ]]; then
    echo "Starting gpsd (needs sudo)..."
    # gpsd_start.sh's own "python3 gps_port.py" call is a bare relative
    # path -- run it from script/, not the workspace root this script
    # itself just cd'd to.
    (cd script && ./gpsd_start.sh)
else
    echo "Not starting gpsd (START_GPSD=$START_GPSD)."
    echo "  gps_node needs it running -- start it once with:  ./_gpsd_run.sh"
fi

source _setup_env.sh

echo "Launching istrorsx_bringup full.launch.py $LAUNCH_ARGS"

# Deliberately NOT "exec ros2 launch ...": ros2 launch shuts its nodes down
# cleanly on SIGINT (Ctrl+C) but not on SIGTERM -- a plain "kill <pid>" of the
# launch leaves every node running as an orphan (verified: 7 of them), and
# leftover nodes then publish alongside the next run's, which is a genuinely
# confusing thing to debug. Running it as a child and translating TERM->INT
# makes "kill" behave like Ctrl+C.
ros2 launch istrorsx_bringup full.launch.py $LAUNCH_ARGS &
_LAUNCH_PID=$!
trap 'kill -INT "$_LAUNCH_PID" 2>/dev/null || true' INT TERM

# Two waits: the first returns as soon as a trapped signal is handled (bash
# interrupts "wait" to run the handler), the second then waits for ros2
# launch to actually finish tearing its nodes down.
wait "$_LAUNCH_PID" 2>/dev/null || true
wait "$_LAUNCH_PID" 2>/dev/null || true
