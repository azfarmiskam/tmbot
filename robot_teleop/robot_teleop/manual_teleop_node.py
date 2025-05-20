#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import tty
import termios

class ManualTeleopNode(Node):
    def __init__(self):
        super().__init__('manual_teleop_node')
        self.cmd_pub = self.create_publisher(String, 'cmd_drive', 10)
        self.get_logger().info("Use keys: W=F, S=B, A=L, D=R, X=Stop, H=Go Home")

    def run(self):
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1).lower()
                if key == 'w':
                    self.send_cmd('F')
                elif key == 's':
                    self.send_cmd('B')
                elif key == 'a':
                    self.send_cmd('L')
                elif key == 'd':
                    self.send_cmd('R')
                elif key == 'x':
                    self.send_cmd('S')
                elif key == 'h':
                    self.send_cmd('H')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def send_cmd(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sent command: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = ManualTeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

