#include "race_car_controller/race_car_controller.hpp"
#include <cmath>
#include <algorithm>

namespace racecar
{

RaceCarController::RaceCarController()
: Node("racecar_controller"),
  desired_speed_(0.0), autonomous_flag_(false)
{

    
    this->declare_parameter<double>("lookahead_distance", 2.0);
    this->declare_parameter<double>("wheelbase", 0.33);
    this->declare_parameter<double>("min_steer", -1.0);
    this->declare_parameter<double>("max_steer", 1.0);
    this->declare_parameter<double>("min_speed", -5.0);
    this->declare_parameter<double>("max_speed", 10.0);
    this->declare_parameter<std::string>("namespace_str", "/ego_racecar");
    this->declare_parameter<std::string>("waypoints_path", "src/pure_pursuit/racelines/adjusted_smoothed_path.csv");
    this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    this->declare_parameter<std::string>("car_refFrame", "ego_racecar/base_link");
    this->declare_parameter<std::string>("global_refFrame", "map");
    this->declare_parameter<std::string>("drive_topic", "/drive");
    this->declare_parameter<std::string>("rviz_current_waypoint_topic", "/ego_racecar/current_waypoint");
    this->declare_parameter<std::string>("rviz_lookahead_waypoint_topic", "/ego_racecar/lookahead_waypoint");
    this->declare_parameter<std::string>("path_topic", "/path");
    this->declare_parameter<std::string>("servo_topic_pub", "/commands/servo/position");
    this->declare_parameter<std::string>("motor_topic_pub", "/commands/motor/speed");

    lookahead_distance_         = this->get_parameter("lookahead_distance").as_double();
    wheelbase_                  = this->get_parameter("wheelbase").as_double();
    min_steer_                  = this->get_parameter("min_steer").as_double();
    max_steer_                  = this->get_parameter("max_steer").as_double();
    min_speed_                  = this->get_parameter("min_speed").as_double();
    max_speed_                  = this->get_parameter("max_speed").as_double();
    namespace_str_              = this->get_parameter("namespace_str").as_string();
    waypoints_path_             = this->get_parameter("waypoints_path").as_string();
    odom_topic_                 = this->get_parameter("odom_topic").as_string();
    car_refFrame_               = this->get_parameter("car_refFrame").as_string();
    global_refFrame_            = this->get_parameter("global_refFrame").as_string();
    drive_topic_                = this->get_parameter("drive_topic").as_string();
    rviz_current_waypoint_topic_ = this->get_parameter("rviz_current_waypoint_topic").as_string();
    rviz_lookahead_waypoint_topic_ = this->get_parameter("rviz_lookahead_waypoint_topic").as_string();
    path_topic_                 = this->get_parameter("path_topic").as_string();
    servo_topic_pub_            = this->get_parameter("servo_topic_pub").as_string();
    motor_topic_pub_            = this->get_parameter("motor_topic_pub").as_string();

    RCLCPP_INFO(this->get_logger(), "Initialized RaceCarController with parameters:");
    RCLCPP_INFO(this->get_logger(), "  Lookahead Distance: %.2f", lookahead_distance_);
    RCLCPP_INFO(this->get_logger(), "  Wheelbase: %.2f", wheelbase_);
    RCLCPP_INFO(this->get_logger(), "  Min Steer: %.2f, Max Steer: %.2f", min_steer_, max_steer_);
    RCLCPP_INFO(this->get_logger(), "  Min Speed: %.2f, Max Speed: %.2f", min_speed_, max_speed_);
    RCLCPP_INFO(this->get_logger(), "  Namespace: %s", namespace_str_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Waypoints Path: %s", waypoints_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Odom Topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Car Ref Frame: %s", car_refFrame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Global Ref Frame: %s", global_refFrame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Drive Topic: %s", drive_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  RViz Current Waypoint Topic: %s", rviz_current_waypoint_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  RViz Lookahead Waypoint Topic: %s", rviz_lookahead_waypoint_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Path Topic: %s", path_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Servo Topic Pub: %s", servo_topic_pub_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Motor Topic Pub: %s", motor_topic_pub_.c_str());


    // Initialize publishers and subscribers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        rviz_lookahead_waypoint_topic_, 10
    );
    steering_pub_ = this->create_publisher<std_msgs::msg::Float64>(servo_topic_pub_, 10);
    throttle_pub_ = this->create_publisher<std_msgs::msg::Float64>(motor_topic_pub_, 10);

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        path_topic_, 10, std::bind(&RaceCarController::pathCallback, this, std::placeholders::_1)
    );
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&RaceCarController::poseCallback, this, std::placeholders::_1)
    );
    speed_sub_ = this->create_subscription<vesc_msgs::msg::VescStateStamped>(
        "/sensors/core", 10, std::bind(&RaceCarController::speedCallback, this, std::placeholders::_1)
    );

    target_speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/target_speed", 10,
        std::bind(&RaceCarController::speed_callback, this, std::placeholders::_1)
    );


    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10, std::bind(&RaceCarController::joyCallback, this, std::placeholders::_1)
    );

    last_autonomous_toggle_time_ = this->get_clock()->now();

}

RaceCarController::~RaceCarController() {
    RCLCPP_INFO(this->get_logger(), "RaceCarController is shutting down.");
}

void RaceCarController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    this->path_ = msg;
}

void RaceCarController::speed_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    this->desired_speed_ = msg->data * 1000.0;
    RCLCPP_INFO(this->get_logger(), "velocity = %.3f ", msg->data);
    set_desired_speed(this->desired_speed_);
}

void RaceCarController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{

    auto now = this->get_clock()->now();
    if (msg->buttons.size() > 0 && msg->buttons[0] == 1) {
        if ((now - last_autonomous_toggle_time_).seconds() > toggle_cooldown_sec_) {
            autonomous_flag_ = !autonomous_flag_;
            last_autonomous_toggle_time_ = now;
          //  RCLCPP_INFO(this->get_logger(), "Autonomous mode: %s", autonomous_flag_ ? "ON" : "OFF");
        }
    }

    // Autonmous flag open means we are in autonomous mode, if not, it is Idle mode
    if (!autonomous_flag_) {
        if (msg->axes.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Joystick axes not enough!");
            return;
        }
       // RCLCPP_INFO(this->get_logger(), "IDLE mode");

        std_msgs::msg::Float64 steer_msg;
        steer_msg.data = STOP_STEER; 
        steering_pub_->publish(steer_msg);
	
        std_msgs::msg::Float64 speed_msg;
        speed_msg.data = STOP_SPEED;
        throttle_pub_->publish(speed_msg);
    }
}

void RaceCarController::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg;
    if (!path_) {
   // RCLCPP_INFO(this->get_logger(), "Path message not received yet, exiting.");
    return;
    }
    if (path_->poses.empty()) {
        RCLCPP_INFO(this->get_logger(), "Path message is empty, exiting.");
        return;
    }
    if (!autonomous_flag_) {
        RCLCPP_INFO(this->get_logger(), "Autonomous mode is not active, exiting.");
        return;
    }

    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    auto& curr_pt = msg->pose.pose.position;
    for (size_t i = 0; i < path_->poses.size(); ++i) {
        const auto& pt = path_->poses[i].pose.position;
        double dist = std::hypot(pt.x - curr_pt.x, pt.y - curr_pt.y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    
    double lookahead_dist = lookahead_distance_;
    size_t lookahead_idx = path_->poses.size() - 1;
    for (size_t i = closest_idx; i < path_->poses.size(); ++i) {
        const auto& pt = path_->poses[i].pose.position;
        double dist = std::hypot(pt.x - curr_pt.x, pt.y - curr_pt.y);
        if (dist > lookahead_dist) {
            lookahead_idx = i;
            break;
        }
    }
    const auto& lookahead_pt = path_->poses[lookahead_idx].pose.position;

    // "lookahead_target" is the namespace for the marker
    Marker_publisher(frame_id_, lookahead_pt, "lookahead_target");

    double yaw = tf2::getYaw(msg->pose.pose.orientation);
    double dx = lookahead_pt.x - curr_pt.x;
    double dy = lookahead_pt.y - curr_pt.y;
    double alpha = std::atan2(dy, dx) - yaw;
    while (alpha > M_PI) alpha -= 2 * M_PI;
    while (alpha < -M_PI) alpha += 2 * M_PI;
    double Ld = std::hypot(dx, dy);
    double theta = std::atan2(2.0 * wheelbase_ * std::sin(alpha), Ld);
    theta = theta * (180.0 /M_PI);
    RCLCPP_WARN(this->get_logger(),
    "DEBUG: dx=%.3f dy=%.3f alpha=%.3f Ld=%.3f theta=%.3f",
    dx, dy, alpha, Ld, theta);
    theta = theta * 2.2;
    double servo_cmd = clamp(theta,-45.0, 45.0);
    servo_cmd = mapValue(servo_cmd,-45.0,45.0,0.2,0.8);
    RCLCPP_WARN(this->get_logger(),
    "DEBUG: theta=%.3f servo_cmd=%.3f (min_steer=%.2f max_steer=%.2f)", 
    theta, servo_cmd, min_steer_, max_steer_);


    std_msgs::msg::Float64 steer_msg;
    steer_msg.data = servo_cmd;
    steering_pub_->publish(steer_msg);

    std_msgs::msg::Float64 speed_msg;
    speed_msg.data = desired_speed_;
        RCLCPP_WARN(this->get_logger(),
    "DEBUG: desired_vel=%.3f)", 
    desired_speed_);

    throttle_pub_->publish(speed_msg);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "steer: %.3f | spd: %.2f | err: %.2f | lookahead idx: %lu", 
    //     steer_msg.data, current_speed_, desired_speed_ - current_speed_, lookahead_idx);
}

void RaceCarController::speedCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg) {
    this->current_speed_ = msg->state.speed;
}

void RaceCarController::set_desired_speed(double speed) {
    this->desired_speed_ = clamp(speed, min_speed_, max_speed_);
}

double RaceCarController::mapValue(double x, double in_min, double in_max, double out_min, double out_max) {
    x = clamp(x, in_min, in_max);
    RCLCPP_INFO(this->get_logger(), "X = %.3f ", x);
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void RaceCarController::Marker_publisher(std::string frame_id, geometry_msgs::msg::Point point, std::string ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point;
    marker.pose.orientation.w = 1.0; // No rotation
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration(0, 100000000); // 100 ms
    marker_pub_->publish(marker);
}

}  // namespace racecar

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<racecar::RaceCarController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
