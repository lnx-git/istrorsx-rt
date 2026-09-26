#!/usr/bin/env python3
# Synthetic control-board telemetry. Publishes ImuData and ServoData, the two
# topics the real ctrlboard_node produces, for machines with no serial
# hardware (the cloud dev VM). Runs until Ctrl+C; no verification logic.
#
# "data", not "imu": it produces BOTH telemetry topics. And it is only the
# outbound half of ctrlboard_node -- the real node also subscribes to
# SpeedCommand, LedConfig and DeviceAction and writes them down the serial
# line. None of that is here, so anything the robot would be *told* to do
# goes nowhere: no speed, no steering, no LEDs, no ball drop. This is a
# telemetry source, not a control board.
#
# WHY THIS EXISTS: planner_node does nothing at all without IMU data. Its
# plannerTick() carries the gate
#
#     // dont process data if ctrlb_euler_x is not known!!
#     if (ctrlb_euler_x < ANGLE_OK) {
#
# and ctrlb_euler_x starts at ANGLE_NONE (dataset.cpp), set only by
# cb_imu() from ImuData.euler_x. Measured on this VM: planner_node alone
# produced 0 decisions; the moment ImuData arrived it produced 25 in six
# seconds. The same callback also stamps ctrlb_time2, whose change is what
# sets do_process -- so ImuData alone satisfies both halves of the gate, and
# ServoData is optional garnish.
#
# ServoData is published anyway (disable with servo:=false) because without
# it the planner sees no encoder, steering angle or motor setting at all.
# The defaults describe a robot sitting still: STATE_STOP, zero encoder
# pulses, wheels straight, motor at rest. That is the truth for a mock fed by
# a still image, so it is the honest default rather than a convenient one.
#
# Rate defaults to 50 Hz. The real node's timer runs at 5 ms, but that is its
# serial poll, not a publish rate; 50 Hz matches planner_node's own
# PROCESS_PERIOD_MIN of 20 ms, which is the ceiling above which extra
# messages cannot produce extra decisions anyway.
#
# Parameters (ros2 run ... --ros-args -p name:=value, or via the .sh wrapper):
#   rate       publish rate in Hz for both topics, default 50
#   euler_x    IMU yaw in degrees, default 90. ANY value below ANGLE_OK
#              (999998) opens the planner's gate; the number itself only
#              matters once navigation/wrongway logic is in play. The 2025
#              park reference log sat at euler_x=324.12.
#   servo      also publish ServoData, default true
#   state      ServoData.state, default 1 (STATE_STOP). 2 = STATE_FWD.
#   angle      steering servo setting, default SA_STRAIGHT
#   velocity   motor setting, default VEL_ZERO (at rest)
#   ircv500    encoder pulses over the last 500 ms, default 0.0 (not moving);
#              the park log showed 27-31 while actually driving
import os
import re
import sys

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from istrorsx_hw.msg import ImuData, ServoData

WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CTRLBOARD_DEFS_H = os.path.join(WS_ROOT, "src", "istrobtx", "include", "ctrlboard_defs.h")


def load_define(name, fallback):
    """Read a plain integer #define out of ctrlboard_defs.h.

    Parsed rather than copied so the defaults cannot drift from the firmware
    constants, the same way test_camera_gen.py takes its depth profile from
    vision_depth.cpp. Falls back if the header moves or the value stops being
    a bare literal -- a mock is not worth failing to start over.
    """
    try:
        with open(CTRLBOARD_DEFS_H) as f:
            m = re.search(r"^#define\s+" + name + r"\s+(\d+)", f.read(), re.M)
        if m:
            return int(m.group(1))
    except OSError:
        pass
    return fallback


VEL_ZERO = load_define("VEL_ZERO", 335)
SA_STRAIGHT = load_define("SA_STRAIGHT", 334)

# ANGLE_OK from config.h -- euler_x must land below this or the planner's
# "dont process data if ctrlb_euler_x is not known!!" gate never opens.
ANGLE_OK = 999998


def dyn(default):
    """Declare-parameter helper: accept 50 as readily as 50.0."""
    return default, ParameterDescriptor(dynamic_typing=True)


class CtrlBoardDataGenerator(Node):
    def __init__(self):
        super().__init__("test_ctrlboard_data_gen")

        self.declare_parameter("rate", *dyn(50.0))
        self.declare_parameter("euler_x", *dyn(90.0))
        self.declare_parameter("euler_y", *dyn(0.0))
        self.declare_parameter("euler_z", *dyn(0.0))
        self.declare_parameter("servo", True)
        self.declare_parameter("state", *dyn(ServoData.STATE_STOP))
        self.declare_parameter("angle", *dyn(SA_STRAIGHT))
        self.declare_parameter("velocity", *dyn(VEL_ZERO))
        self.declare_parameter("ircv500", *dyn(0.0))

        rate = float(self.get_parameter("rate").value)
        self.euler_x = float(self.get_parameter("euler_x").value)
        self.euler_y = float(self.get_parameter("euler_y").value)
        self.euler_z = float(self.get_parameter("euler_z").value)
        self.with_servo = bool(self.get_parameter("servo").value)
        self.state = int(self.get_parameter("state").value)
        self.angle = int(self.get_parameter("angle").value)
        self.velocity = int(self.get_parameter("velocity").value)
        self.ircv500 = float(self.get_parameter("ircv500").value)

        if rate <= 0:
            raise SystemExit("rate must be > 0, got %r" % rate)
        if self.euler_x >= ANGLE_OK:
            raise SystemExit(
                "euler_x=%r is >= ANGLE_OK (%d), which is exactly the "
                "\"not known\" case planner_node refuses to process -- it "
                "would sit idle. Pick a real heading, e.g. 90."
                % (self.euler_x, ANGLE_OK))

        self.pub_imu = self.create_publisher(ImuData, "/robot/imu_data", 10)
        self.pub_servo = self.create_publisher(ServoData, "/robot/servo_data", 10)

        self.ticks = 0
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            "publishing /robot/imu_data%s at %.1f Hz: euler_x=%.2f, "
            "state=%d, angle=%d, velocity=%d, ircv500=%.2f"
            % (" + /robot/servo_data" if self.with_servo else "",
               rate, self.euler_x, self.state, self.angle,
               self.velocity, self.ircv500))

    def tick(self):
        imu = ImuData()
        imu.euler_x = self.euler_x
        imu.euler_y = self.euler_y
        imu.euler_z = self.euler_z
        # 3 = fully calibrated, what a warmed-up BNO055 reports.
        imu.calib_gyro = 3
        imu.calib_accel = 3
        imu.calib_mag = 3
        self.pub_imu.publish(imu)

        if self.with_servo:
            servo = ServoData()
            servo.state = self.state
            servo.heading = 0.0        # firmware never populates this
            servo.ircv = 0
            servo.ircv500 = self.ircv500
            servo.angle = self.angle
            servo.velocity = self.velocity
            servo.loadd = 0
            # millis() % 10000, as the board reports it.
            servo.cbtime = int(self.get_clock().now().nanoseconds // 1_000_000) % 10000
            # 0 = no echo, i.e. nothing close. Matches the park log's own
            # front sensors while driving in the open.
            servo.ulsd1 = 0
            servo.ulsd2 = 0
            servo.ulsd3 = 0
            servo.ulsd4 = 0
            servo.ulsd5 = 0
            self.pub_servo.publish(servo)

        self.ticks += 1
        if self.ticks % 250 == 0:      # ~ every 5s at 50 Hz
            self.get_logger().info("published %d samples" % self.ticks)


def main():
    rclpy.init()
    node = CtrlBoardDataGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C and SIGTERM respectively -- both are the normal way this
        # generator ends, neither deserves a traceback.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
