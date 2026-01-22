#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
import math
import time


class UltrasonicToLaser(Node):

    def __init__(self):
        super().__init__('ultrasonic_to_laserscan')

        self.sub = self.create_subscription(
            Range,
            '/ultrasonic_range',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )

        self.last_time = self.get_clock().now()

    def cb(self, msg):
        scan = LaserScan()

        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'

        # ---- Fake LiDAR parameters ----
        scan.angle_min = -0.1
        scan.angle_max = 0.1
        scan.angle_increment = 0.01

        scan.range_min = 0.02
        scan.range_max = 1.0

        beam_count = int(
            (scan.angle_max - scan.angle_min) /
            scan.angle_increment
        )

        ranges = [float('inf')] * beam_count

        # Put ultrasonic reading in the center beam
        center = beam_count // 2
        if msg.range < scan.range_max:
            ranges[center] = msg.range

        scan.ranges = ranges
        self.pub.publish(scan)


def main():
    rclpy.init()
    node = UltrasonicToLaser()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
