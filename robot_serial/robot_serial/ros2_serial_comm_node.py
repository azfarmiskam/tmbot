#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String, UInt8, Bool

class SerialCommNode(Node):
    def __init__(self):
        super().__init__('ros2_serial_comm_node')

        # Adjust the port as needed (e.g., /dev/ttyUSB0 or /dev/arduino)
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        self.battery_pub = self.create_publisher(UInt8, 'battery_level', 10)
        self.obstacle_pub = self.create_publisher(Bool, 'obstacle_alert', 10)
        self.cliff_pub = self.create_publisher(Bool, 'cliff_alert', 10)

        self.cmd_sub = self.create_subscription(
            String, 'cmd_drive', self.send_command_to_arduino, 10)

        self.timer = self.create_timer(0.5, self.read_from_arduino)

    def send_command_to_arduino(self, msg):
        cmd = msg.data.strip().upper()
        if cmd in ['F', 'B', 'L', 'R', 'S', 'H']:
            self.ser.write((cmd + '\n').encode())
            self.get_logger().info(f'Sent command to Arduino: {cmd}')

    def read_from_arduino(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                # Expected: "BAT:82 OBS:0 CLIFF:0"
                if line.startswith("BAT:"):
                    parts = line.split()
                    battery = int(parts[0].split(":")[1])
                    obs = bool(int(parts[1].split(":")[1]))
                    cliff = bool(int(parts[2].split(":")[1]))

                    self.battery_pub.publish(UInt8(data=battery))
                    self.obstacle_pub.publish(Bool(data=obs))
                    self.cliff_pub.publish(Bool(data=cliff))

            except Exception as e:
                self.get_logger().warn(f'Error reading serial: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

