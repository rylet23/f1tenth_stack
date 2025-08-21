#ifndef RACE_CAR_CONTROLLER_HPP_
#define RACE_CAR_CONTROLLER_HPP_


// Constants
#define toggle_cooldown_sec_ 0.5
#define STOP_STEER 0.5 // Neutral position for steering
#define STOP_SPEED 0.0 // Speed to stop the car


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // foxy
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // humble
#include <tf2/utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <algorithm>
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>

namespace racecar
{

class RaceCarController : public rclcpp::Node
{
public:
    RaceCarController();
    ~RaceCarController();

    // Callback functions
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void speedCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr msg);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void speed_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void Marker_publisher(std::string frame_id, geometry_msgs::msg::Point point, std::string ns);
    void set_desired_speed(double speed);
    double mapValue(double x, double in_min, double in_max, double out_min, double out_max);

    // std::clamp is not working.
    template <typename T>
    constexpr const T& clamp(const T& v, const T& lo, const T& hi)
    {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }


private:
    // ROS publisher/subscriber
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr throttle_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr speed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_speed_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Path and pose
    nav_msgs::msg::Path::SharedPtr path_;
    nav_msgs::msg::Odometry::SharedPtr current_pose_;

    // Controller parameters
    double min_steer_;
    double max_steer_;
    double min_speed_;
    double max_speed_;
    double lookahead_distance_;
    double wheelbase_;
    double desired_speed_;
    double current_speed_;
    std::string frame_id_;
    std::string namespace_str_;
    std::string waypoints_path_;
    std::string odom_topic_;
    std::string car_refFrame_;
    std::string global_refFrame_;
    std::string drive_topic_;
    std::string rviz_current_waypoint_topic_;
    std::string rviz_lookahead_waypoint_topic_;
    std::string path_topic_;
    std::string servo_topic_pub_;
    std::string motor_topic_pub_;
    
    bool autonomous_flag_;
    rclcpp::Time last_autonomous_toggle_time_;

};

} // namespace racecar

#endif // RACE_CAR_CONTROLLER_HPP_
