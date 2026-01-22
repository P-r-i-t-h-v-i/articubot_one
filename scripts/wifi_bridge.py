#!/usr/bin/env python3

import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class WifiBridge(Node):

    def __init__(self):
        super().__init__('wifi_bridge')

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.range_pub = self.create_publisher(
            Range, '/ultrasonic_range', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 8888))
        self.sock.setblocking(False)

        self.esp_addr = None
        self.timer = self.create_timer(0.05, self.receive_ultrasonic)

        self.get_logger().info("WiFi bridge running (no PWM)")

    def cmd_callback(self, msg):
        if self.esp_addr is None:
            return
        data = f"{msg.linear.x},{msg.angular.z}"
        self.sock.sendto(data.encode(), self.esp_addr)
        self.get_logger().info(
        f"CMD_VEL -> linear: {msg.linear.x}, angular: {msg.angular.z}"
)


    def receive_ultrasonic(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            self.esp_addr = addr

            r = Range()
            r.header.frame_id = "ultrasonic_link"
            r.radiation_type = Range.ULTRASOUND
            r.min_range = 0.02
            r.max_range = 4.0
            r.range = float(data.decode())

            self.range_pub.publish(r)
        except BlockingIOError:
            pass


def main():
    rclpy.init()
    node = WifiBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
