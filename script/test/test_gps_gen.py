#!/usr/bin/env python3
# Synthetic GPS generator -- mimics gps_node's publishGpsData(): publishes a
# fixed position on /robot/gps_data every 1s, fix=True, speed/course=0
# (stationary). Runs indefinitely (Ctrl+C to stop) as a standalone stand-in
# for gps_node when testing other GpsData consumers (e.g. navigation_node,
# save_node) in isolation -- no verification/consumer logic here, just the
# generator.
import rclpy
from rclpy.node import Node

from istrorsx_hw.msg import GpsData

TICK_PERIOD_SEC = 1.0  # gps_node: "GPS fixes update at roughly 1 Hz"

LONGITUDE = 17.1098255
LATITUDE = 48.1352596


class GpsGenerator(Node):
    def __init__(self):
        super().__init__('test_gps_gen')
        self.count = 0
        self.pub = self.create_publisher(GpsData, '/robot/gps_data', 10)
        self.timer = self.create_timer(TICK_PERIOD_SEC, self.tick)

    def tick(self):
        msg = GpsData()
        msg.fix = True
        msg.latitude = LATITUDE
        msg.longitude = LONGITUDE
        msg.speed = 0.0
        msg.course = 0.0
        self.pub.publish(msg)
        self.count += 1


def main():
    rclpy.init()
    node = GpsGenerator()
    print(f"Publishing GpsData on /robot/gps_data every "
          f"{int(TICK_PERIOD_SEC * 1000)}ms, fix=True, "
          f"lat={LATITUDE}, lon={LONGITUDE} (Ctrl+C to stop)...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"Stopped after publishing {node.count} messages.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
