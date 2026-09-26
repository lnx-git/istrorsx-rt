#!/usr/bin/env python3
# Synthetic ImageNumber generator -- mimics planner_node's publishImageNumber()
# (src/istrorsx_core/src/planner_node.cpp): publishes int64 image_number on
# /robot/image_number every 5ms, starting at 0, incrementing by 1 each tick.
# Runs indefinitely (Ctrl+C to stop) as a standalone stand-in for planner_node
# when testing other ImageNumber consumers (e.g. camera_node) in isolation --
# no verification/consumer logic here, just the generator.
import rclpy
from rclpy.node import Node

from istrorsx_hw.msg import ImageNumber

TICK_PERIOD_SEC = 0.005  # matches planner_node's PLANNER_TICK_PERIOD (5ms)


class ImageNumberGenerator(Node):
    def __init__(self):
        super().__init__('test_image_number_gen')
        self.image_number = 0
        self.pub = self.create_publisher(ImageNumber, '/robot/image_number', 10)
        self.timer = self.create_timer(TICK_PERIOD_SEC, self.tick)

    def tick(self):
        msg = ImageNumber()
        msg.image_number = self.image_number
        self.pub.publish(msg)
        self.image_number += 1


def main():
    rclpy.init()
    node = ImageNumberGenerator()
    print(f"Publishing ImageNumber on /robot/image_number every "
          f"{int(TICK_PERIOD_SEC * 1000)}ms, starting at 0 (Ctrl+C to stop)...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"Stopped after publishing {node.image_number} messages.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
