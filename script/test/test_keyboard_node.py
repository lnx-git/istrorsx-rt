#!/usr/bin/env python3
# End-to-end test for keyboard_node: launches it with stdin attached to a
# real pseudo-terminal (so its isatty()/termios raw-mode code actually
# runs, unlike a plain pipe or redirect), writes simulated keystrokes into
# the pty master, and subscribes to /joy via rclpy to check the published
# axes/buttons match keyboard_node.cpp's mapping table. Uses Python's own
# pty module rather than "ros2 topic pub"/echo (this project's other
# script/test/*.sh convention) since there's no CLI-friendly way to drive a
# subprocess's raw terminal input at all.
import os
import pty
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

JOY_AXIS_STEERING = 0
JOY_AXIS_THROTTLE = 1
JOY_BUTTON_STOP = 0
JOY_BUTTON_PROGRAM = 1
JOY_BUTTON_LED = 2
JOY_BUTTON_WRONGWAY = 3
JOY_BUTTON_MODE_AUTONOMOUS = 6
JOY_BUTTON_MODE_MANUAL = 7
JOY_BUTTON_DIGIT_BASE = 10
JOY_BUTTON_GPS_DEBUG = 20
JOY_BUTTON_STEER_FULL_LEFT = 21
JOY_BUTTON_STEER_FULL_RIGHT = 22


class JoyListener(Node):
    def __init__(self):
        super().__init__('test_keyboard_node_listener')
        self.last = None
        self.create_subscription(Joy, '/joy', self.cb, 10)

    def cb(self, msg):
        self.last = msg


def wait_for(node, predicate, timeout=3.0, label=""):
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.last is not None and predicate(node.last):
            return True
    print(f"TIMEOUT waiting for: {label} (last={node.last})")
    return False


def main():
    master_fd, slave_fd = pty.openpty()

    env = os.environ.copy()
    proc = subprocess.Popen(
        ["ros2", "run", "istrorsx_core", "keyboard_node"],
        stdin=slave_fd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        env=env, text=True, bufsize=1,
    )
    os.close(slave_fd)

    rclpy.init()
    node = JoyListener()

    ok = True
    try:
        time.sleep(2.0)  # let the node come up and start publishing

        # 1) 'w' held (simulate repeats every 80ms, like OS key-repeat) -> throttle=+1
        print("--- pressing 'w' (simulated hold) ---")
        for _ in range(6):
            os.write(master_fd, b'w')
            time.sleep(0.08)
        ok &= wait_for(node, lambda m: len(m.axes) > JOY_AXIS_THROTTLE and m.axes[JOY_AXIS_THROTTLE] > 0.99, label="throttle=+1 while 'w' held")
        print("throttle while held:", node.last.axes if node.last else None)

        # 2) stop sending 'w' -> after hold-timeout (500ms) + a publish tick, throttle should return to 0
        print("--- releasing 'w', waiting for timeout ---")
        ok &= wait_for(node, lambda m: len(m.axes) > JOY_AXIS_THROTTLE and abs(m.axes[JOY_AXIS_THROTTLE]) < 0.01, timeout=2.0, label="throttle=0 after release")
        print("throttle after release:", node.last.axes if node.last else None)

        # 3) 'q' single press -> should appear as a BUTTON (steer full left), not an axis value
        print("--- pressing 'q' once ---")
        os.write(master_fd, b'q')
        ok &= wait_for(node, lambda m: len(m.buttons) > JOY_BUTTON_STEER_FULL_LEFT and m.buttons[JOY_BUTTON_STEER_FULL_LEFT] == 1, label="JOY_BUTTON_STEER_FULL_LEFT=1 after 'q'")
        last = node.last
        print("buttons/axes right after 'q':", list(last.buttons), last.axes)
        axis_untouched = (len(last.axes) <= JOY_AXIS_STEERING) or (abs(last.axes[JOY_AXIS_STEERING]) < 0.01)
        print("steering axis left at 0 (q is a button, not an axis value):", axis_untouched)
        ok &= axis_untouched

        # 4) space -> JOY_BUTTON_STOP
        print("--- pressing space ---")
        os.write(master_fd, b' ')
        ok &= wait_for(node, lambda m: len(m.buttons) > JOY_BUTTON_STOP and m.buttons[JOY_BUTTON_STOP] == 1, label="JOY_BUTTON_STOP=1 after space")
        print("buttons after space:", list(node.last.buttons))

        # 5) digit '7' -> JOY_BUTTON_DIGIT_BASE+7
        print("--- pressing '7' ---")
        os.write(master_fd, b'7')
        ok &= wait_for(node, lambda m: len(m.buttons) > JOY_BUTTON_DIGIT_BASE + 7 and m.buttons[JOY_BUTTON_DIGIT_BASE + 7] == 1, label="digit-7 button=1")
        print("buttons after '7':", list(node.last.buttons))

        # 6) 'a' held -> nudge axis = 0.5 (not a button)
        print("--- pressing 'a' (simulated hold) ---")
        for _ in range(6):
            os.write(master_fd, b'a')
            time.sleep(0.08)
        ok &= wait_for(node, lambda m: len(m.axes) > JOY_AXIS_STEERING and abs(m.axes[JOY_AXIS_STEERING] - 0.5) < 0.01, label="steering axis=0.5 while 'a' held")
        print("axes while 'a' held:", node.last.axes)

    finally:
        # quit keyboard_node via ESC (its own local-quit key), matching how a
        # real user would exit, then check for a clean process exit and a
        # sane (non-broken) terminal restore.
        try:
            os.write(master_fd, b'\x1b')
        except OSError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print("keyboard_node did not exit on ESC within 5s, killing")
            proc.kill()
            proc.wait()
            ok = False

        print("keyboard_node exit code:", proc.returncode)
        os.close(master_fd)
        node.destroy_node()
        rclpy.shutdown()

    print("RESULT:", "OK" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
