import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class LiveLidarReader(Node):
    def __init__(self):
        super().__init__('live_lidar_reader')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.get_logger().info("Subscribed to /scan")

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)

        mask = (ranges < msg.range_min) | (ranges > msg.range_max) | np.isnan(ranges)
        ranges[mask] = 0.0

        ranges_list = ranges.tolist()

        print(f"Scan row (len={len(ranges_list)}): {ranges_list[:10]} ...")


    def feed_to_nn(self, lidar_row):
        # Placeholder for your neural network inference
        pass

def main():
    rclpy.init()
    node = LiveLidarReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
