import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

class JoyController(Node):
    def __init__(self):
        super().__init__('joy_controller')

        # which axes on your joystick control steering + forward/reverse?
        self.declare_parameter('servo_axis', 0)
        self.declare_parameter('forward_axis', 5)
        self.declare_parameter('reverse_axis', 2)

        p = self.get_parameter  # shorthand
        self.servo_axis   = p('servo_axis').get_parameter_value().integer_value
        self.forward_axis = p('forward_axis').get_parameter_value().integer_value
        self.reverse_axis = p('reverse_axis').get_parameter_value().integer_value

        # publishers
        self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed',    10)

        # subscribe to your /joy topic
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def map_val(self, x, in_min, in_max, out_min, out_max):
        x = max(min(x, in_max), in_min)
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def joy_callback(self, msg: Joy):
        # steering
        try:
            raw_steer = msg.axes[self.servo_axis]
        except IndexError:
            self.get_logger().error(f"Axis {self.servo_axis} not in Joy.axes")
            return

        # forward/reverse triggers
        try:
            a_fwd = msg.axes[self.forward_axis]
            a_rev = msg.axes[self.reverse_axis]
        except IndexError:
            self.get_logger().error(f"Axis {self.forward_axis} or {self.reverse_axis} missing")
            return

        # many controllers rest triggers at +1 and pull to –1
        # convert each into [0…1]:  0 at rest, 1 fully pressed
        fwd_norm = (1.0 - a_fwd) / 2.0
        rev_norm = (1.0 - a_rev) / 2.0

        # combine into single throttle in [–1…1]:
        # +1 = full forward, –1 = full reverse
        raw_speed = fwd_norm - rev_norm

        # map steering: joystick [–1,1] → servo [0.8,0.2]
        servo_cmd = self.map_val(raw_steer, -1.0, 1.0, 0.2, 0.8)

        # map speed: combined [–1,1] → [–5000,5000]
        # In order to speed up car - change here <23k rpm
        speed_cmd = self.map_val(raw_speed, -1.0, 1.0, -5000.0, 5000.0)

        # publish
        self.servo_pub.publish(Float64(data=servo_cmd))
        self.speed_pub.publish(Float64(data=speed_cmd))

        self.get_logger().info(
            f"Steer(raw={raw_steer:.2f})→{servo_cmd:.2f}  |  "
            f"Speed(raw={raw_speed:.2f})→{speed_cmd:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = JoyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
