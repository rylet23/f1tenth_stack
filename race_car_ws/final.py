#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped
import serial
import math
import csv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped

class SerialPWMReader(Node):
    def __init__(self):
        super().__init__('serial_pwm_reader')

        # ─── PARAMETERS ─────────────────────────────────────────────────────────────
        waypoints_file_path = (
            '/home/xavier/race_car_ws/route.csv'
        )
        port = '/dev/ttyUSB0'
        baud = 115200
        self.safety_thr = 1600        # pulling up = autonomous, pulling down = manual
        self.lookahead_distance = 1.0 # meters
        self.wheel_base = 0.32        # meters
        self.max_steering_angle = math.radians(40) # radians
        self.speed_kp = 1.0           # P-gain for speed control
        self.constant_speed = 2000.0  # mm/s target speed
        self.maxlookahead_distance = 1.1 # meters
        self.goal_x = None
        self.goal_y = None

        # ─── PURE PURSUIT STATE ─────────────────────────────────────────────────────
        self.current_waypoint_idx = 0
        self.completed_laps = 0

        # ─── SERIAL SETUP ───────────────────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port {port} @ {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            self.ser = None

        # ─── STATE ─────────────────────────────────────────────────────────────────
        self.pose_x = None
        self.pose_y = None
        self.yaw = None
        self.current_speed = 0.0   # m/s

        # ─── LOAD WAYPOINTS ─────────────────────────────────────────────────────────
        self.waypoints = []
        try:
            with open(waypoints_file_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # expects header: x,y
                    self.waypoints.append((float(row['x']), float(row['y'])))
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {waypoints_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")

        # ─── ROS TOPICS ──────────────────────────────────────────────────────────────
        self.servo_pub  = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.speed_pub  = self.create_publisher(Float64, '/commands/motor/speed',    10)
        self.marker_pub = self.create_publisher(Marker,   '/lookahead_marker',       10)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/pose_with_cov',
            10
        )

        self.create_subscription(Odometry,         '/pf/pose/odom', self.odom_callback,    10)
        self.create_subscription(VescStateStamped, '/sensors/core',       self.speed_fb_callback, 10)

        # ─── TIMER @ 20 Hz ────────────────────────────────────────────────────────────
        self.create_timer(0.05, self.timer_callback)

    def map_pwm(self, value, in_min, in_max, out_min, out_max):
        v = max(min(value, in_max), in_min)
        return (v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def odom_callback(self, msg: Odometry):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def speed_fb_callback(self, msg: VescStateStamped):
        self.current_speed = msg.state.speed

    def find_lookahead_waypoint(self):
        if self.pose_x is None or not self.waypoints:
            return False

        for idx in range(self.current_waypoint_idx, len(self.waypoints)):
            wx, wy = self.waypoints[idx]
            dist = math.hypot(wx - self.pose_x, wy - self.pose_y)
            if dist >= self.lookahead_distance and dist <= self.maxlookahead_distance:
                self.current_waypoint_idx = idx
                self.goal_x, self.goal_y = wx, wy
                # Publish marker at goal position
                return True

        self.current_waypoint_idx = 0
        return False


    def compute_servo(self):
        if self.pose_x is None or not self.waypoints:
            print("No pose or waypoints available, returning default servo value")
            return 0.5

        # lookahead noktasını al
        if self.find_lookahead_waypoint() is False:
            return None
        if self.goal_x == 0.0 and self.goal_y == 0.0:
            print("No valid lookahead waypoint found, returning default servo value")
            return None

        if self.goal_x is None or self.goal_y is None:
            print("Goal coordinates not set, returning default servo value")
            return None

        print(f"Distance to goal: {math.hypot(self.goal_x - self.pose_x, self.goal_y - self.pose_y):.2f} m")


        dx = self.goal_x - self.pose_x
        dy = self.goal_y - self.pose_y
        alpha = math.atan2(dy, dx) - self.yaw
        alpha = (alpha + math.pi) % (2*math.pi) - math.pi  # normalize

        L = max(math.hypot(dx, dy), 0.1)
        steering_angle = math.atan2(2*self.wheel_base*math.sin(alpha), L)
        steering_angle = max(min(steering_angle, self.max_steering_angle),
                            -self.max_steering_angle)

        ratio = steering_angle / self.max_steering_angle
        ratio = -ratio
        return 0.5 - max(min(ratio, 1.0), -1.0) * 0.3

    def timer_callback(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not open")
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return
        if not line:
            return

        parts = line.split(',')
        if len(parts) != 4:
            self.get_logger().warn(f"Bad format: '{line}'")
            return

        try:
            pwm = [int(x) for x in parts]
        except ValueError:
            self.get_logger().warn(f"Non-int PWM: {parts}")
            return

        servo_val = self.compute_servo()
        if servo_val is  not None:
            print(f"Servo value: {servo_val:.2f}, Speed: {self.current_speed:.2f} m/s")
        if servo_val is None:
            self.get_logger().warn("Failed to compute servo value")
            servo_val = 0.5  # default value if computation fails
            servo_speed = 0.0
            self.servo_pub.publish(Float64(data=servo_val))
            self.speed_pub.publish(Float64(data=servo_speed))
            return

        # Emergency/manual control
        if pwm[1] < self.safety_thr:
            try:
                line2 = self.ser.readline().decode('utf-8').strip()
                parts2 = line2.split(',')
                if len(parts2) == 4:
                    pwm2 = [int(x) for x in parts2]
                    servo_val = self.map_pwm(pwm2[0], 1000, 2000, 0.8, 0.2)
                    speed_val = self.map_pwm(pwm2[2], 1000, 2000, -5000.0, 5000.0)
                    self.servo_pub.publish(Float64(data=servo_val))
                    self.speed_pub.publish(Float64(data=speed_val))
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "lookahead"
                    marker.id = 0
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x = self.goal_x
                    marker.pose.position.y = self.goal_y
                    marker.pose.position.z = 0.0
                    marker.pose.orientation.w = 1.0
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.scale.z = 0.2
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
                    pose_cov = PoseWithCovarianceStamped()
                    pose_cov.header.frame_id = "odom"
                    pose_cov.header.stamp = self.get_clock().now().to_msg()
                    pose_cov.pose.pose.position.x = self.goal_x
                    pose_cov.pose.pose.position.y = self.goal_y
                    pose_cov.pose.pose.position.z = 0.0
                    pose_cov.pose.pose.orientation.w = 1.0
                    self.pose_cov_pub.publish(pose_cov)

                    self.marker_pub.publish(marker)
            except Exception as e:
                self.get_logger().warn(f"Failed manual-control parse: {e}")
            return

        # Pure Pursuit steering + speed control
        tgt_m_s = self.constant_speed
        err = tgt_m_s - self.current_speed
        cmd_m_s = min(tgt_m_s, tgt_m_s + self.speed_kp * err * 3)

        self.servo_pub.publish(Float64(data=servo_val))
        self.speed_pub.publish(Float64(data=cmd_m_s))
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rclpy.duration.Duration(seconds=0.0).to_msg()
        self.marker_pub.publish(marker)
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header.frame_id = "odom"
        pose_cov.header.stamp = self.get_clock().now().to_msg()
        pose_cov.pose.pose.position.x = self.goal_x
        pose_cov.pose.pose.position.y = self.goal_y
        pose_cov.pose.pose.position.z = 0.0
        pose_cov.pose.pose.orientation.w = 1.0
        self.pose_cov_pub.publish(pose_cov)

def main(args=None):
    rclpy.init(args=args)
    node = SerialPWMReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
