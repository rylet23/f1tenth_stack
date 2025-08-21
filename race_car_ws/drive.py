#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class DriveStraight(Node):
    def __init__(self):
        super().__init__('drive_straight')
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.steer_pub = self.create_publisher(Float64, '/commands/servo/position', 10)

    def run(self):
        # Keep steering centered (0.5 means straight)
        self.steer_pub.publish(Float64(data=0.5))
        # Set speed to 3000 RPM
        self.speed_pub.publish(Float64(data=3000.0))
        self.get_logger().info("Driving straight at 3000 RPM for 4 seconds...")
        time.sleep(6)
        # Stop the car
        self.speed_pub.publish(Float64(data=0.0))
        self.get_logger().info("Stopped.")

def main():
    rclpy.init()
    node = DriveStraight()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
