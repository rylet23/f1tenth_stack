import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

class CSVPathPublisher(Node):
    def __init__(self):
        super().__init__('csv_path_publisher')
        self.publisher = self.create_publisher(Path, '/global_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.path = Path()
        self.load_csv('/adjusted_smoothed_path.csv')

    def load_csv(self, filepath):
        with open(filepath, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(row['x'])
                pose.pose.position.y = float(row['y'])
                pose.pose.orientation.w = 1.0
                self.path.poses.append(pose)

    def publish_path(self):
        self.path.header.frame_id = "map"
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = CSVPathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
