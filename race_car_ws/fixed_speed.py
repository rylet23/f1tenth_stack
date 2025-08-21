import math, os, traceback
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from Zed_Camera import Camera
def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

class ReactiveController(Node):
    def __init__(self):
        super().__init__('reactive_speed_controller')

        # Publishers
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.steer_pub = self.create_publisher(Float64, '/commands/servo/position', 10)

        # Lidar
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(LaserScan, '/scan', self.lidar_cb, qos)
        
        self.servo_center = 0.5
        self.servo_span   = 0.5                     
        self.max_steer_rad = math.radians(50)
        self.rpm_per_mps   = 900.0
        self.cruise_mps    = 2.6
        self.slow_zone_m   = 1.2
        self.stop_zone_m   = 0.40
        self.front_half_deg = 15
        self.side_deg       = 45
        self.side_half_deg  = 20
        self.steer_gain     = 0.8 
        self.resume_clear_m    = 0.85
        self.reverse_min_s     = 1.2
        self.reverse_max_s     = 3.0
        self.forward_delay_s   = 1.2


        self.forward_offset_deg = 0.0

        self.panic_zone_m         = 0.3
        self.reverse_duration_s   = 0.7
        self.reverse_rpm_mag      = 2000.0
        self.reverse_cooldown_s   = 1.0

        self.target_rpm = 0.0
        self.last_ranges = None
        self.cb_count = 0

        self.speed_timer = self.create_timer(0.05, self.publish_speed_unclamped)
        self._scan_meta_logged = False

        self.escape_bias_rad = math.radians(12)
        self.escape_bias_s = 1.5
        self._escape_until = 0.0
        self._escape_sign  = 0.0
        self.cam = Camera(self, topic='/zed2i/zed_node/point_cloud/cloud', forward_axis='z')

    def publish_speed_unclamped(self):
        self.speed_pub.publish(Float64(data=clamp(self.target_rpm, -5000.0, 5000.0)))

    def _sector_stats(self, r, angles, center_deg, half_deg):
        c = math.radians(center_deg); h = math.radians(half_deg)
        m_ang = (angles >= c - h) & (angles <= c + h)
        if not np.any(m_ang):
            return 0.0, 0.0, 0.0
        vals = r[m_ang]
        valid = vals[vals > 0.0]
        if valid.size == 0:
            return 0.0, 0.0, 0.0
        return float(np.min(valid)), float(np.mean(valid)), float(np.median(valid))

    def lidar_cb(self, msg: LaserScan):
        
        try:
            r = np.asarray(msg.ranges, dtype=np.float32)
            bad = (r < msg.range_min) | (r > msg.range_max) | np.isnan(r)
            r[bad] = 0.0
            self.last_ranges = r

            n = r.size
            angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
            angles = angles + math.radians(self.forward_offset_deg)

            if not self._scan_meta_logged:
                self._scan_meta_logged = True
                self.get_logger().warn(
                    f"scan: angle_min={math.degrees(msg.angle_min):.1f}°, "
                    f"angle_max={math.degrees(msg.angle_max):.1f}°, "
                    f"inc={math.degrees(msg.angle_increment):.2f}°, "
                    f"forward_offset={self.forward_offset_deg:.1f}°"
                )

            # Sector stats
            front_min, front_mean, _ = self._sector_stats(r, angles, 0.0, self.front_half_deg)
            left_min,  left_mean,  _ = self._sector_stats(r, angles, +self.side_deg, self.side_half_deg)
            right_min, right_mean, _ = self._sector_stats(r, angles, -self.side_deg, self.side_half_deg)
            cam_front, cam_left_mean_cam, cam_right_mean_cam = self.cam.sector_stats(
                front_half_deg=self.front_half_deg, side_deg=self.side_deg, side_half_deg=self.side_half_deg
            )       

            front_min = min(front_min, cam_front)
            left_mean = min(left_mean, cam_left_mean_cam)
            right_mean = min(right_mean, cam_right_mean_cam)
            now = self.get_clock().now().nanoseconds / 1e9

            if getattr(self, "_fwd_hold_until", 0.0) > now:
                denom = max(left_mean, right_mean, 0.3)
                side_diff = (left_mean - right_mean) / denom
                steer_rad = clamp(self.steer_gain * side_diff, -self.max_steer_rad, self.max_steer_rad)
                servo = clamp(self.servo_center + (steer_rad / self.max_steer_rad) * self.servo_span, 0.0, 1.0)
                self.target_rpm = 0.0
                self.steer_pub.publish(Float64(data=servo))
                return

            rev, rev_rpm, rev_servo = self.maybe_reverse(front_min, left_mean, right_mean)
            if rev:
                self.target_rpm = rev_rpm
                self.steer_pub.publish(Float64(data=rev_servo))
                return

            denom = max(left_mean, right_mean, 0.3)
            side_diff = (left_mean - right_mean) / denom
            steer_rad = clamp(self.steer_gain * side_diff, -self.max_steer_rad, self.max_steer_rad)
            if now < getattr(self, "_escape_until", 0.0):
                steer_rad = clamp(
                    steer_rad + self.escape_bias_rad * (self._escape_sign or 0.0),
                    -self.max_steer_rad, self.max_steer_rad
                )

            if front_min < self.stop_zone_m:
                speed_mps = 0.0

            elif front_min < self.slow_zone_m:
                alpha = (front_min - self.stop_zone_m) / max(1e-6, (self.slow_zone_m - self.stop_zone_m))
                speed_mps = clamp(alpha * self.cruise_mps, 0.0, self.cruise_mps)
            else:
                speed_mps = self.cruise_mps

            if abs(steer_rad) > math.radians(25):
                speed_mps = min(speed_mps, 0.8 * self.cruise_mps)
            if abs(steer_rad) > math.radians(35):
                speed_mps = min(speed_mps, 0.6 * self.cruise_mps)

            # Outputs
            servo = self.servo_center + (steer_rad / self.max_steer_rad) * self.servo_span
            servo = clamp(servo, 0.0, 1.0)
            rpm = max(0.0, speed_mps * self.rpm_per_mps)

            self.target_rpm = rpm
            self.steer_pub.publish(Float64(data=servo))

            self.cb_count += 1
            if self.cb_count % 10 == 0:
                self.get_logger().info(
                    f"front_min={front_min:.2f}m  L/R_mean={left_mean:.2f}/{right_mean:.2f}  "
                    f"steer={math.degrees(steer_rad):.1f}°  rpm={rpm:.0f}"
                )

        except Exception as e:
            self.get_logger().error("lidar_cb exception: " + ''.join(
                traceback.format_exception_only(type(e), e)).strip())
            self.steer_pub.publish(Float64(data=self.servo_center))

    def maybe_reverse(self, front_min, left_mean, right_mean):

        now = self.get_clock().now().nanoseconds / 1e9

        if not hasattr(self, "_rev_until"):
            self._rev_until = 0.0
            self._rev_min_until = 0.0
            self._rev_max_until = 0.0
            self._rev_cooldown_until = 0.0
            self._fwd_hold_until = 0.0

        panic   = getattr(self, "panic_zone_m", 0.3)
        clear   = getattr(self, "resume_clear_m", 0.60)
        min_s   = getattr(self, "reverse_min_s", 1.0)
        max_s   = getattr(self, "reverse_max_s", 2.5)
        rev_cd  = getattr(self, "reverse_cooldown_s", 1.0)
        rev_mag = abs(getattr(self, "reverse_rpm_mag", 2200.0))

        def reverse_servo():

            denom = max(left_mean, right_mean, 0.3)
            side_diff = (left_mean - right_mean) / denom
            steer_rad = clamp(-self.steer_gain * side_diff, -self.max_steer_rad, self.max_steer_rad)
            return clamp(self.servo_center + (steer_rad / self.max_steer_rad) * self.servo_span, 0.0, 1.0)

        if now < self._rev_until:
            if now < self._rev_min_until:
                return True, -rev_mag, reverse_servo()

            if (front_min < clear) and (now < self._rev_max_until):
                return True, -rev_mag, reverse_servo()

            self._rev_until = 0.0
            self._fwd_hold_until = now + getattr(self, "forward_delay_s", 0.6)
            self._rev_cooldown_until = now + rev_cd
            self._escape_until = now + self.escape_bias_s
            self._escape_sign = 1.0 if left_mean >= right_mean else -1.0

            return False, 0.0, self.servo_center

        # Not reversing
        if (front_min < panic) and (now >= self._rev_cooldown_until):
            self._escape_sign = 1.0 if left_mean >= right_mean else -1.0
            self._rev_min_until = now + min_s
            self._rev_max_until = now + max_s
            self._rev_until     = self._rev_max_until
            self.get_logger().warn(f"REVERSING: front_min={front_min:.2f} < panic={panic:.2f}")
            return True, -rev_mag, reverse_servo()

        return False, 0.0, self.servo_center

def main():
    rclpy.init()
    node = ReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.speed_pub.publish(Float64(data=0.0))
        node.steer_pub.publish(Float64(data=node.servo_center))
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
