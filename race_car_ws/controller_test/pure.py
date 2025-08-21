#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial

# -------------------- TUNING CONSTANTS --------------------
# RC input PWM range
PWM_IN_MIN    = 1000
PWM_IN_MAX    = 2000

# Servo output [0.8 ← full left, 0.2 ← full right]
SERVO_OUT_MIN = 0.8
SERVO_OUT_MAX = 0.2

# Speed output in m/s
# full reverse → –3 m/s, full forward → +3 m/s
SPEED_OUT_MIN = -3000.0
SPEED_OUT_MAX =  3000.0

# Safety: PWM channel-1 threshold
SAFETY_PWM1_THRESHOLD = 1600
# -----------------------------------------------------------

class SerialPWMReader(Node):
    def __init__(self):
        super().__init__('serial_pwm_reader')

        port = '/dev/ttyUSB0'
        baud = 115200
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Could not open {port}: {e}")
            self.ser = None

        # publishers
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed',    10)

        # 20 Hz timer
        self.create_timer(0.05, self.timer_callback)


    def map_pwm(self, value, in_min, in_max, out_min, out_max):
        value = max(min(value, in_max), in_min)
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


    def timer_callback(self):
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial port not open")
            return

        line = self.ser.readline().decode('utf-8').strip()
        if not line:
            return

        parts = line.split(',')
        if len(parts) != 4:
            self.get_logger().warn(f"Bad format: '{line}'")
            return

        try:
            pwm = [int(x) for x in parts]
        except ValueError:
            self.get_logger().warn(f"Non-int PWM data: {parts}")
            return

        # ─── SAFETY CUTOUT ─────────────────────────────────────────
        if pwm[1] > SAFETY_PWM1_THRESHOLD:
            # channel1 too high → Emergency stop
            neutral_servo = self.map_pwm(
                1500, PWM_IN_MIN, PWM_IN_MAX,
                SERVO_OUT_MIN, SERVO_OUT_MAX
            )
            self.servo_pub.publish(Float64(data=neutral_servo))
            self.speed_pub.publish(Float64(data=0.0))
            self.get_logger().warn(
                f"SAFETY STOP! pwm1={pwm[1]} > {SAFETY_PWM1_THRESHOLD}"
            )
            return

        # ─── NORMAL MAPPING ─────────────────────────────────────────
        # channel0 → steering
        servo_val = self.map_pwm(
            pwm[0],
            PWM_IN_MIN, PWM_IN_MAX,
            SERVO_OUT_MIN, SERVO_OUT_MAX
        )

        # channel2 → speed (now in m/s)
        speed_val = self.map_pwm(
            pwm[2],
            PWM_IN_MIN, PWM_IN_MAX,
            SPEED_OUT_MIN, SPEED_OUT_MAX
        )

        self.servo_pub.publish(Float64(data=servo_val))
        self.speed_pub.publish(Float64(data=speed_val))
        self.get_logger().info(
            f"Mapped → pwm0={pwm[0]}→servo={servo_val:.2f}, "
            f"pwm2={pwm[2]}→speed={speed_val:.2f} m/s"
        )


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

