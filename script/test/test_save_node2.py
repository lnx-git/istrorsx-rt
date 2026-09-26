#!/usr/bin/env python3
# save_node test 2/4: a synthetic LidarData scan makes save_node write
# lidar.png -- needs only save_node running (see test_save_node2.sh,
# "-lidar /dev/null" to set conf.useLidar=1). No planner_node/PlannerDebugData
# is published either -- lidar_angle_min/max/process_angle_min/max all fall
# back to their -1 "none yet" sentinels (rendered as no angle markers in the
# image), which lidar_draw_output() already handles without crashing/stalling
# (see the earlier "works without planner_node" fix in save_node's design).
import sys
import time

import rclpy
from rclpy.node import Node

from istrorsx_hw.msg import LidarData
from istrorsx_core.msg import SaveEvent


class TestNode(Node):
    def __init__(self):
        super().__init__('test_save_node2')
        self.events = []
        self.pub_lidar = self.create_publisher(LidarData, '/robot/lidar_data', 10)
        self.create_subscription(SaveEvent, '/robot/save_event', self.cb_event, 10)

    def cb_event(self, msg):
        self.events.append(msg)

    def wait_for_tag(self, tag, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            for ev in self.events:
                for f in ev.files:
                    if f.tag == tag:
                        return f
        return None


def main():
    rclpy.init()
    node = TestNode()
    ok = False
    try:
        time.sleep(3.0)  # let discovery settle -- 1.0s occasionally isn't
        # enough (confirmed via a real miss: cb_lidar() never fired for an
        # entire run, save_lidar_ stuck at 0 the whole time), same fix and
        # same reasoning as test_save_node1.py/test_save_node4.py.

        lidar = LidarData()
        lidar.point_count = 8
        lidar.sync = [1, 0, 0, 0, 0, 0, 0, 0]
        lidar.angle = [0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 270.0]
        lidar.distance = [400.0, 600.0, 900.0, 1500.0, 900.0, 600.0, 400.0, 2000.0]
        lidar.quality = [20, 22, 25, 30, 25, 22, 20, 15]
        node.pub_lidar.publish(lidar)
        print("Published synthetic LidarData (8 points) to /robot/lidar_data")

        f = node.wait_for_tag("lidar", timeout=15.0)
        ok = f is not None
        if f:
            print(f"  lidar -> {f.filename}  <-- inspect this one")

        # Keep save_node up a bit longer after the tag is found instead of
        # tearing down immediately -- user-requested, so there's more time to
        # inspect the running process/logs before this script's own .sh
        # wrapper kills it.
        time.sleep(3.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("RESULT (lidar -> lidar.png):", "OK" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
