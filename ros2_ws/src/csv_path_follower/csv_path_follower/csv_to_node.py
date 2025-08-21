import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import csv
import os

class CSVPathPublisher(Node):
    def __init__(self):
        super().__init__('csv_path_publisher')
        self.publisher = self.create_publisher(Path, '/global_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)
        self.path = Path()

        # Broadcast TF between 'map' and 'base_link'
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.1, self.broadcast_tf)  # Broadcast at 10Hz

        self.load_csv(os.path.expanduser('~/ros2_ws/src/csv_path_follower/csv_path_follower/adjusted_smoothed_path.csv'))

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

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CSVPathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

