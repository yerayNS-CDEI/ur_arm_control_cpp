#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

# Node to use with the serial_test.ino code
class SerialNode(Node):
    def __init__(self):
        super().__init__("led_node_ros")

        # Initialize serial communication with Arduino
        self.arduinoData = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Create subscriber for motor control commands
        self.subscription = self.create_subscription(String, 'led_control', self.command_callback, 10)

        self.timer = self.create_timer(0.1, self.read_serial_callback)

        self.get_logger().info("Serial node initialized")

    def command_callback(self, msg):
        self.get_logger().info(f"Sending command to Arduino: {msg.data}")
        try:
            self.arduinoData.write(msg.data.encode())
        except Exception as e:
            self.get_logger().error(f"Error writing to serial: {e}")

    def read_serial_callback(self):
        try:
            if self.arduinoData.in_waiting > 0:
                line = self.arduinoData.readline().decode('utf-8').strip()
                if line:
                    self.get_logger().info(f"Received from Arduino: {line}")
        except Exception as e:
            self.get_logger().error(f"Error reading from serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()