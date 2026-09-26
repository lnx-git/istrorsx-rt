"""Bring up the whole istrorsx robot at once: all four istrorsx_hw hardware
nodes, all istrorsx_core logic nodes, the external visionn_server.py NN
inference process, and (by default) a real Xbox controller via the standard
joy package.

Replaces the legacy main()/loop()'s own threads.start(...) orchestration --
and, in the near term, this project's own per-node *_node_run.sh scripts,
which this launch file's own defaults are built to match exactly.

Usage (from the workspace root -- see the CWD note below):
    ros2 launch istrorsx_bringup full.launch.py
    ros2 launch istrorsx_bringup full.launch.py cb_device:=/dev/ttyUSB0
    ros2 launch istrorsx_bringup full.launch.py visionn_server_variant:=segfb0-trt
    ros2 launch istrorsx_bringup full.launch.py launch_xbox_controller:=false

Three things this launch file deliberately does NOT do, all flagged during
this package's own design analysis (doc/ai/01_architecture.md):

0. Does not start keyboard_node. It puts the terminal into raw mode and reads
   keypresses straight off stdin -- it even refuses to start without one
   ("keyboard_node needs an interactive terminal on stdin"), and ros2 launch
   gives its children no tty, so launching it here fails immediately no
   matter where in the sequence it's placed (verified, not assumed). Start it
   by hand in a second terminal instead -- the launch prints this reminder at
   startup too:
       ./keyboard_node_run.sh

1. Does not start gpsd. gpsd needs sudo (killing/restarting a system daemon,
   socket cleanup -- see script/gpsd_start.sh) -- awkward and fragile from
   inside a launch file, and it's a once-after-boot system-level step, not
   part of "bring up this ROS graph". Start it manually first:
       script/gpsd_start.sh  (or the -USB0/-USB1/-USB3 variant for the
       actual port in use)

2. Does NOT resolve any of the relative paths every node already assumes
   (conf/log4cxx.xml, out/, logout/, sample/*.jpg, conf/vision_mask.png,
   ramdisk/) via ament_index/package-share lookups -- doing that properly
   would mean touching Config/save_node.cpp/telemetry_node.cpp/vision_node's
   own calibration loading across both istrorsx_hw and istrorsx_core, a much
   bigger refactor than this package on its own. Deliberately deferred (user
   decision): this launch file instead requires the SAME thing every
   *_node_run.sh script already implicitly requires -- being invoked with
   the workspace root as the current working directory:
       cd ~/istrorsx_ws && ros2 launch istrorsx_bringup full.launch.py
   A sanity check below fails fast with a clear message if that's not the
   case, rather than letting each node fail separately with a cryptic
   "file not found".
"""
import glob
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Where a visionn server variant lives, given its short name ("segfb0",
# "segfb0-trt", ...). Each such directory owns its own start script, and that
# script -- not this launch file -- decides how to run the server: the CPU
# variant uses its own .venv_rosvm interpreter, the TensorRT one uses plain
# system python3 (JetPack ships tensorrt/cv2). Deliberately NOT duplicated
# here: adding a future variant should mean dropping in a directory, not
# editing this file.
_VISIONN_DIR_FMT = "script/visionn_{}_server"
_VISIONN_START_SCRIPT = "visionn_server_start.sh"

# LD_PRELOAD workaround for camera_node -- see camera_node_front_run.sh/
# camera_node_rear_run.sh's own comment: librealsense2.so bundles its own
# (unhidden) copy of eprosima::fastcdr/fastrtps for its DDS network-camera
# feature, which collides with ROS 2's own libfastcdr/libfastrtps and
# corrupts rclcpp::Node's first type registration -> segfault in
# CameraNode's ctor. Preloading the ROS 2 versions makes the dynamic linker
# prefer them.
_CAMERA_LD_PRELOAD = (
    "/opt/ros/jazzy/lib/libfastcdr.so.2:/opt/ros/jazzy/lib/libfastrtps.so.2.14"
)

# How long to let visionn_server.py finish loading its model before
# starting vision_node -- a pragmatic fixed delay, NOT a robust readiness
# check (this project's own test scripts hit real discovery-timing
# flakiness with sleep-based waits more than once this session -- see
# doc/ai/03_progress.md). A real fix would poll the TCP port (7001) instead
# of guessing a duration; flagged as a known limitation, not solved here.
_VISIONN_STARTUP_DELAY_SEC = 5.0


def _check_cwd_sane():
    # Fails fast with one clear message instead of every single node
    # failing separately with its own "file not found" once it tries to
    # open conf/log4cxx.xml -- see module docstring's CWD note.
    if not os.path.isfile("conf/log4cxx.xml"):
        raise RuntimeError(
            "istrorsx_bringup's full.launch.py must be run with the "
            "istrorsx_ws workspace root as the current working directory "
            "(conf/log4cxx.xml not found relative to CWD) -- run:\n"
            "    cd ~/istrorsx_ws && ros2 launch istrorsx_bringup full.launch.py"
        )


def _visionn_dir(variant):
    # Resolved (and validated) here rather than passed straight through as a
    # LaunchConfiguration substitution, so a typo'd variant fails immediately
    # with the list of real options instead of ExecuteProcess dying later with
    # a bare "No such file or directory".
    directory = _VISIONN_DIR_FMT.format(variant)
    if not os.path.isfile(os.path.join(directory, _VISIONN_START_SCRIPT)):
        available = sorted(
            d.split("visionn_", 1)[1].rsplit("_server", 1)[0]
            for d in glob.glob(_VISIONN_DIR_FMT.format("*"))
            if os.path.isfile(os.path.join(d, _VISIONN_START_SCRIPT))
        )
        raise RuntimeError(
            f"unknown visionn_server_variant '{variant}': "
            f"{directory}/{_VISIONN_START_SCRIPT} not found.\n"
            f"    available variants: {', '.join(available) or '(none found)'}"
        )
    return directory


def _argv_value(name, default):
    # Reads a "name:=value" launch argument straight from sys.argv. Needed for
    # values used while the launch description is still being BUILT -- picking
    # and validating the visionn server directory, and composing the startup
    # summary below -- where a LaunchConfiguration substitution isn't resolved
    # yet. The same names are still DeclareLaunchArgument-ed normally, so
    # --show-args and the conditions keep working the usual way.
    for arg in sys.argv:
        if arg.startswith(name + ":="):
            return arg.split(":=", 1)[1]
    return default


def generate_launch_description():
    _check_cwd_sane()

    visionn_server_variant = _argv_value("visionn_server_variant", "segfb0")
    visionn_dir = _visionn_dir(visionn_server_variant)

    cb_device_arg = DeclareLaunchArgument(
        "cb_device", default_value="/dev/ttyUSB_CB",
        description="Control board serial device")
    cb2_device_arg = DeclareLaunchArgument(
        "cb2_device", default_value="/dev/ttyUSB_CB2",
        description="Second control board serial device")
    lidar_device_arg = DeclareLaunchArgument(
        "lidar_device", default_value="/dev/lidar0",
        description="RPLIDAR A1 serial device")
    # The navigation point sequence, parsed two characters at a time
    # (navig.cpp's navigation_next_point()). "*1*2*3" is the competition
    # layout -- pickup, destination, service area -- whose coordinates are
    # ANGLE_NONE in navig_data.cpp and get filled in at runtime from scanned
    # QR codes (NavigationPointSet.msg).
    #
    # Passing this flag at all is what sets conf.useNavigation=1; without it
    # navigationTick() plans no route and navp_idx stays -1. That is why the
    # default is a real path rather than "".
    #
    # NOTE when overriding from a shell: quote it. The asterisks are glob
    # characters, and while bash leaves an unmatched pattern alone, a stray
    # matching filename in the CWD would silently rewrite the argument --
    #   ./_full_launch.sh navigation_path:='*1*2'
    navigation_path_arg = DeclareLaunchArgument(
        "navigation_path", default_value="*1*2*3",
        description="navigation point sequence for -path, e.g. '*1*2*3' or 'E1'")

    # The legacy run script's own tail (doc/istrobtx_2025/script/r.sh):
    #   -nowait -i DIstrobotics -path ... -vf 12 -vf2 12 -vf3 12 -vb -17 -cg 300 -ca 0
    # istro_rt2025 was one process, so one command line configured everything.
    # Here each node parses its own argv into its own Config, so these are
    # appended to EVERY node rather than guessed at per node -- a flag a node
    # does not read costs nothing, and getting the split wrong costs a drive.
    ctrlboard_init_str_arg = DeclareLaunchArgument(
        "ctrlboard_init_str", default_value="DIstrobotics",
        description="-i, control board init string, sent at startup. "
                    "\"DIstrobotics\" is the board's D(isplay) command plus "
                    "the text, i.e. it writes Istrobotics on the display. "
                    "Cosmetic -- nothing depends on it")
    velocity_fwd_arg = DeclareLaunchArgument(
        "velocity_fwd", default_value="12",
        description="-vf, forward speed offset from VEL_ZERO")
    velocity_fwd2_arg = DeclareLaunchArgument(
        "velocity_fwd2", default_value="12", description="-vf2")
    velocity_fwd3_arg = DeclareLaunchArgument(
        "velocity_fwd3", default_value="12", description="-vf3")
    velocity_back_arg = DeclareLaunchArgument(
        "velocity_back", default_value="-17", description="-vb, reverse speed")
    calib_gps_azimuth_arg = DeclareLaunchArgument(
        "calib_gps_azimuth", default_value="300",
        description="-cg. Config only honours it when useGPSDevice AND "
                    "(useAHRSystem OR useControlBoard2) are set, and only if "
                    "they were set EARLIER in argv -- which is why -cb2 leads "
                    "the common tail below")
    calib_imu_yaw_arg = DeclareLaunchArgument(
        "calib_imu_yaw", default_value="0", description="-ca, see -cg")

    # Appended to every Config-parsing node. -cb2 comes FIRST: config.cpp
    # parses argv in order and -cg/-ca are silently dropped unless
    # useControlBoard2 (or useAHRSystem) is already set when they are reached.
    # Only istro_main_ctrlboard.cpp acts on useControlBoard2 -- for every other
    # node it is an inert flag, the same way -lidar is for planner/save.
    common_args = [
        "-cb2", LaunchConfiguration("cb2_device"),
        "-i", LaunchConfiguration("ctrlboard_init_str"),
        "-vf", LaunchConfiguration("velocity_fwd"),
        "-vf2", LaunchConfiguration("velocity_fwd2"),
        "-vf3", LaunchConfiguration("velocity_fwd3"),
        "-vb", LaunchConfiguration("velocity_back"),
        "-cg", LaunchConfiguration("calib_gps_azimuth"),
        "-ca", LaunchConfiguration("calib_imu_yaw"),
    ]

    # Named after the input DEVICE, not after the /joy topic: /joy has two
    # possible producers in this project (this Xbox controller, and
    # keyboard_node -- see the hint at the bottom), and a future USB-cabled
    # joystick would be a third. "launch_joy" wouldn't say which one.
    launch_xbox_controller_arg = DeclareLaunchArgument(
        "launch_xbox_controller", default_value="true",
        description="Start the standard joy package's joy_node for a real Xbox controller, "
                    "publishing /joy (same as xbox_controller_run.sh). Set false when driving "
                    "from the keyboard instead -- keyboard_node publishes the same /joy topic.")
    xbox_deadzone_arg = DeclareLaunchArgument(
        "xbox_deadzone", default_value="0.05",
        description="joy_node's own deadzone for the Xbox controller -- kept low deliberately, "
                    "drive_node's own pollKey() already applies its own axis deadzone "
                    "(see xbox_controller_run.sh)")
    # drive_node's own real-joystick-only safety caps -- see
    # joystick_velocity_max_fwd_'s own comment in drive_node.hpp. Distinct
    # from xbox_deadzone_arg above (that's joy_node's raw-signal deadzone;
    # this is drive_node's throttle-axis activation threshold and the
    # velocity ceiling once it does activate).
    joystick_velocity_max_fwd_arg = DeclareLaunchArgument(
        "joystick_velocity_max_fwd", default_value="12",
        description="drive_node: real-joystick-only cap on velocity above VEL_ZERO "
                    "(ctrlboard_defs.h). Does not apply to keyboard_node or AUTONOMOUS/"
                    "progLoop() speeds.")
    joystick_velocity_max_back_arg = DeclareLaunchArgument(
        "joystick_velocity_max_back", default_value="-17",
        description="drive_node: real-joystick-only cap on velocity below VEL_ZERO "
                    "(negative offset). See joystick_velocity_max_fwd.")
    joystick_throttle_deadzone_arg = DeclareLaunchArgument(
        "joystick_throttle_deadzone", default_value="0.5",
        description="drive_node: pollKey()'s throttle-axis activation threshold, wider than "
                    "steering's own 0.3 -- a real gamepad has both axes on the same physical "
                    "stick, so a mostly-lateral steering motion still carries a small "
                    "forward/back component.")
    # ups_node: Waveshare UPS Power Module (C) battery telemetry -- no legacy
    # equivalent, own parameters (not CLI flags like -cb/-lidar, since this
    # node has no Config at all -- see doc/ai/01_architecture.md § ups_node).
    ups_i2c_bus_arg = DeclareLaunchArgument(
        "ups_i2c_bus", default_value="7",
        description="ups_node: I2C bus number for the INA219 (/dev/i2c-<N>).")
    ups_i2c_addr_arg = DeclareLaunchArgument(
        "ups_i2c_addr", default_value="65",
        description="ups_node: I2C address of the INA219 (65 = 0x41, matches "
                    "UPS_Power_Module_C/ina219.py's own __main__).")
    ups_poll_period_ms_arg = DeclareLaunchArgument(
        "ups_poll_period_ms", default_value="2000",
        description="ups_node: how often to read the INA219 and publish UpsData, in ms.")
    # Declared for --show-args discoverability and so "visionn_server_variant:=..."
    # is accepted like any other launch argument; the value itself is read
    # straight from sys.argv above, where it's needed earlier (see there).
    launch_visionn_server_arg = DeclareLaunchArgument(
        "launch_visionn_server", default_value="true",
        description="Start the NN inference server as part of this launch. Set false when it "
                    "is already running (started by hand via "
                    "script/visionn_<variant>_server/visionn_server_start.sh, which backgrounds "
                    "it): vision_node then simply connects to the existing server over TCP "
                    "(port 7001) and this launch neither starts nor stops it. Useful because "
                    "the server takes seconds to load its model -- leaving it up across "
                    "repeated launches makes the restart cycle much faster.")
    visionn_server_variant_arg = DeclareLaunchArgument(
        "visionn_server_variant", default_value="segfb0",
        description="Which NN inference server to start: the directory "
                    "script/visionn_<variant>_server must exist and contain "
                    "visionn_server_start.sh. 'segfb0' = CPU/ONNX (own venv, "
                    "used on the dev VM), 'segfb0-trt' = TensorRT, SegFormer-B0 "
                    "(system python3, the Jetson's GPU path), 'roadyolo-trt' = "
                    "TensorRT, YOLO11s-seg road segmentation (same Jetson GPU "
                    "path as 'segfb0-trt').")

    ctrlboard_node = Node(
        package="istrorsx_hw", executable="ctrlboard_node", name="ctrlboard_node",
        output="screen",
        arguments=["-nowait", "-cb", LaunchConfiguration("cb_device"),
                   "-cb2", LaunchConfiguration("cb2_device")] + common_args,
    )

    # No common_args/-nowait -- this node has no Config, none of those legacy
    # flags apply (see ups_node.hpp's own comment).
    ups_node = Node(
        package="istrorsx_hw", executable="ups_node", name="ups_node",
        output="screen",
        parameters=[{
            "i2c_bus": LaunchConfiguration("ups_i2c_bus"),
            "i2c_addr": LaunchConfiguration("ups_i2c_addr"),
            "poll_period_ms": LaunchConfiguration("ups_poll_period_ms"),
        }],
    )

    gps_node = Node(
        package="istrorsx_hw", executable="gps_node", name="gps_node",
        output="screen",
        arguments=["-nowait"] + common_args,
    )

    lidar_node = Node(
        package="istrorsx_hw", executable="lidar_node", name="lidar_node",
        output="screen",
        arguments=["-nowait", "-lidar", LaunchConfiguration("lidar_device")] + common_args,
    )

    camera_node_front = Node(
        package="istrorsx_hw", executable="camera_node", name="camera_node_front",
        output="screen",
        arguments=["-nowait", "-camdev", "0"] + common_args,
        additional_env={"LD_PRELOAD": _CAMERA_LD_PRELOAD},
    )

    camera_node_rear = Node(
        package="istrorsx_hw", executable="camera_node", name="camera_node_rear",
        output="screen",
        arguments=["-nowait", "-camdev", "1"] + common_args,
        additional_env={"LD_PRELOAD": _CAMERA_LD_PRELOAD},
    )

    drive_node = Node(
        package="istrorsx_core", executable="drive_node", name="drive_node",
        output="screen",
        arguments=["-nowait"] + common_args,
        parameters=[{
            "joystick_velocity_max_fwd": LaunchConfiguration("joystick_velocity_max_fwd"),
            "joystick_velocity_max_back": LaunchConfiguration("joystick_velocity_max_back"),
            "joystick_throttle_deadzone": LaunchConfiguration("joystick_throttle_deadzone"),
        }],
    )

    navigation_node = Node(
        package="istrorsx_core", executable="navigation_node", name="navigation_node",
        output="screen",
        arguments=["-nowait", "-path", LaunchConfiguration("navigation_path")] + common_args,
    )

    # Not a ROS node -- the external Python NN inference service vision_node
    # talks to over TCP (port 7001), see doc/ai/01_architecture.md's "NN
    # Inference Service" section. Must be started before vision_node (see
    # _VISIONN_STARTUP_DELAY_SEC above).
    #
    # Runs the selected variant's OWN start script rather than invoking
    # python directly, so the interpreter choice stays in one place: the CPU
    # variant's script uses its .venv_rosvm, the TensorRT one uses system
    # python3. VISIONN_FOREGROUND=1 makes that script exec the server in the
    # foreground and log to stdout instead of backgrounding it -- without it,
    # the script would exit immediately and ros2 launch would neither show
    # the server's output nor be able to stop it on shutdown, leaving an
    # orphaned process behind between runs.
    visionn_server = ExecuteProcess(
        cmd=["./" + _VISIONN_START_SCRIPT],
        cwd=visionn_dir,
        output="screen",
        additional_env={"VISIONN_FOREGROUND": "1"},
        condition=IfCondition(LaunchConfiguration("launch_visionn_server")),
    )

    def _vision_node(condition=None):
        # A fresh Node action per branch below -- the same action object must
        # not appear twice in one LaunchDescription, even under mutually
        # exclusive conditions.
        return Node(
            package="istrorsx_core", executable="vision_node", name="vision_node",
            output="screen",
            arguments=["-nowait"] + common_args,
            condition=condition,
        )

    # launch_visionn_server=true: the server is starting right now, so hold
    # vision_node back until its model is loaded (_VISIONN_STARTUP_DELAY_SEC).
    vision_node_delayed = TimerAction(
        period=_VISIONN_STARTUP_DELAY_SEC,
        actions=[_vision_node()],
        condition=IfCondition(LaunchConfiguration("launch_visionn_server")),
    )
    # launch_visionn_server=false: the server was started beforehand and is already
    # listening, so there is nothing to wait for -- vision_node just opens a
    # TCP connection to it at startup like any other client.
    vision_node_now = _vision_node(
        condition=UnlessCondition(LaunchConfiguration("launch_visionn_server")))

    planner_node = Node(
        package="istrorsx_core", executable="planner_node", name="planner_node",
        output="screen",
        # -lidar sets conf.useLidar=1. planner_node never opens the device --
        # it only consumes LidarData.msg -- but cb_lidar() drops every scan
        # when the flag is off, so without this the world model is built from
        # vision alone. Each node parses its own argv into its own Config, so
        # lidar_node's flag does nothing for this process.
        arguments=["-nowait", "-lidar", LaunchConfiguration("lidar_device")] + common_args,
    )

    save_node = Node(
        package="istrorsx_core", executable="save_node", name="save_node",
        output="screen",
        # -lidar sets conf.useLidar=1, without which saveLidarGroup() never
        # writes lidar.png (same reasoning as planner_node above; matches
        # save_node_run.sh).
        arguments=["-nowait", "-lidar", LaunchConfiguration("lidar_device")] + common_args,
    )

    telemetry_node = Node(
        package="istrorsx_core", executable="telemetry_node", name="telemetry_node",
        output="screen",
        # No -nowait/-cb-style arguments -- telemetry_node has no Config of
        # its own (see doc/ai/01_architecture.md's telemetry_node section);
        # output_dir stays its own ROS parameter default ("ramdisk/").
    )

    xbox_controller = Node(
        package="joy", executable="joy_node", name="joy_node",
        parameters=[{"deadzone": LaunchConfiguration("xbox_deadzone")}],
        condition=IfCondition(LaunchConfiguration("launch_xbox_controller")),
    )

    # keyboard_node is NOT launched here -- it needs a real tty and refuses to
    # start without one (see module docstring #0). This reminder is printed
    # last, so it stays visible at the bottom of the startup output.
    _xbox_on = _argv_value("launch_xbox_controller", "true").lower() == "true"
    _visionn_on = _argv_value("launch_visionn_server", "true").lower() == "true"
    keyboard_hint = LogInfo(msg=(
        "\n"
        "  ---------------------------------------------------------------\n"
        "   Manual control -- /joy has two possible sources here:\n"
        f"     - Xbox controller (joy_node): {'STARTED' if _xbox_on else 'not started'}\n"
        "     - keyboard_node:               not started (needs its own tty)\n"
        "       To drive from the keyboard, run in a SECOND terminal:\n"
        "           cd ~/istrorsx_ws && ./keyboard_node_run.sh\n"
        "\n"
        f"   visionn_server ({visionn_server_variant}): "
        f"{'started by this launch' if _visionn_on else 'NOT started -- connecting to an already-running one'}\n"
        f"       {visionn_dir}\n"
        "  ---------------------------------------------------------------"
    ))

    return LaunchDescription([
        cb_device_arg,
        cb2_device_arg,
        ctrlboard_init_str_arg,
        velocity_fwd_arg,
        velocity_fwd2_arg,
        velocity_fwd3_arg,
        velocity_back_arg,
        calib_gps_azimuth_arg,
        calib_imu_yaw_arg,
        lidar_device_arg,
        navigation_path_arg,
        launch_xbox_controller_arg,
        xbox_deadzone_arg,
        joystick_velocity_max_fwd_arg,
        joystick_velocity_max_back_arg,
        joystick_throttle_deadzone_arg,
        ups_i2c_bus_arg,
        ups_i2c_addr_arg,
        ups_poll_period_ms_arg,
        launch_visionn_server_arg,
        visionn_server_variant_arg,
        ctrlboard_node,
        ups_node,
        gps_node,
        lidar_node,
        camera_node_front,
        camera_node_rear,
        drive_node,
        navigation_node,
        visionn_server,
        vision_node_delayed,
        vision_node_now,
        planner_node,
        save_node,
        telemetry_node,
        xbox_controller,
        keyboard_hint,
    ])
