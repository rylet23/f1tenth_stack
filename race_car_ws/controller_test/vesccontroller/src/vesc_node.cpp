#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <string>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;
namespace asio = boost::asio;

class VESCControllerNode : public rclcpp::Node
{
public:
  VESCControllerNode()
  : Node("vesc_controller_cpp"), io_context_(), serial_port_(io_context_)
  {
    status_publisher_ = this->create_publisher<std_msgs::msg::Float64>("vesc_status", 10);
    command_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "vesc_command", 10, std::bind(&VESCControllerNode::commandCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(100ms, std::bind(&VESCControllerNode::publishStatus, this));

    connectToVESC();
  }

private:
  void connectToVESC()
  {
    try {
      serial_port_.open("/dev/ttyACM0"); // Adjust port
      serial_port_.set_option(asio::serial_port::baud_rate(115200)); // Adjust baudrate
      serial_port_.set_option(asio::serial_port::parity(asio::serial_port::parity::none));
      serial_port_.set_option(asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
      serial_port_.set_option(asio::serial_port::flow_control(asio::serial_port::flow_control::none));
      RCLCPP_INFO(this->get_logger(), "Connected to VESC");
    } catch (const boost::system::system_error& error) {
      RCLCPP_ERROR(this->get_logger(), "Error connecting to VESC: %s", error.what());
    }
  }

  void commandCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (serial_port_.is_open()) {
      try {
        // Example: Assuming 'vesc_command' sends duty cycle (float64)
        float duty_cycle = msg->data;
        std::vector<uint8_t> command = encodeDutyCycle(duty_cycle); // Implement this function
        asio::write(serial_port_, asio::buffer(command));
        RCLCPP_INFO(this->get_logger(), "Sent command: Duty Cycle = %f", duty_cycle);
      } catch (const boost::system::system_error& error) {
        RCLCPP_WARN(this->get_logger(), "Error sending command: %s", error.what());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "VESC not connected. Cannot send command.");
    }
  }

  void publishStatus()
  {
    if (serial_port_.is_open()) {
      try {
        // Example: Request and read RPM (you'll need to implement the VESC protocol for this)
        int rpm = getRPM(); // Implement this function
        auto status_msg = std_msgs::msg::Float64();
        status_msg.data = static_cast<double>(rpm);
        status_publisher_->publish(status_msg);
        RCLCPP_INFO(this->get_logger(), "Published status: RPM = %d", rpm);
      } catch (const boost::system::system_error& error) {
        RCLCPP_WARN(this->get_logger(), "Error getting status: %s", error.what());
      }
    }
  }

private:
  asio::io_context io_context_;
  asio::serial_port serial_port_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr status_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- VESC Protocol Implementation (You need to implement these) ---
  std::vector<uint8_t> encodeDutyCycle(float duty)
  {
    std::vector<uint8_t> data;
    // Implement the VESC duty cycle command encoding here
    // Refer to the VESC communication protocol documentation
    return data;
  }

  int getRPM()
  {
    int rpm = 0;
    // Implement the VESC RPM request and decoding here
    // Refer to the VESC communication protocol documentation
    return rpm;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VESCControllerNode>());
  rclcpp::shutdown();
  return 0;
}
