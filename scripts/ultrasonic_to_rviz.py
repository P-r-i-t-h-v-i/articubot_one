#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker


class UltrasonicRViz(Node):

    def __init__(self):
        super().__init__('ultrasonic_rviz')

        self.sub = self.create_subscription(
            Range,
            '/ultrasonic_range',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Marker,
            '/ultrasonic_marker',
            10
        )

    def callback(self, msg):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "ultrasonic"
        marker.id = 0

    # ---- PARAMETERS ----
        MAX_RANGE = 1.0          # 100 cm
        FRONT_OFFSET = 0.125     # half robot length
        SENSOR_HEIGHT = 0.08     # meters
        ALPHA = 0.7              # smoothing factor (0–1)

    # ---- FILTER NOISE ----
        if not hasattr(self, "filtered_range"):
            self.filtered_range = msg.range

        self.filtered_range = (
            ALPHA * self.filtered_range +
            (1 - ALPHA) * msg.range
        )

        distance = self.filtered_range

    # ---- NO OBJECT → DELETE MARKER ----
        if distance <= 0.02 or distance > MAX_RANGE:
            marker.action = Marker.DELETE
            self.pub.publish(marker)
            return

    # ---- VALID OBJECT → SHOW MARKER ----
        marker.action = Marker.ADD
        marker.type = Marker.CUBE

        marker.pose.position.x = FRONT_OFFSET + distance
        marker.pose.position.y = 0.0
        marker.pose.position.z = SENSOR_HEIGHT
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.pub.publish(marker)




def main():
    rclpy.init()
    node = UltrasonicRViz()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
