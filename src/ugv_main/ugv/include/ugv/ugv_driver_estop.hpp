#ifndef UGV_UGV_DRIVER_ESTOP_HPP_
#define UGV_UGV_DRIVER_ESTOP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ugv/robot_tools.hpp"
#include <nlohmann/json.hpp>
#include <cstdlib> // For std::system

class UgvDriverEstop : public rclcpp::Node {
public:
    UgvDriverEstop();
    ~UgvDriverEstop();

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void led_ctrl_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void voltage_callback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr led_ctrl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltage_sub_;

    RobotTools robot_tools_;
    std::atomic<bool> estop_;

    // Vendor and product IDs should ideally be configurable via ROS parameters.
    const std::string DRIVER_VENDOR_ID = "1a86"; // Example Vendor ID
    const std::string DRIVER_PRODUCT_ID = "55d3"; // Example Product ID

    // These should be parameters
    const double wheel_separation = 0.174;
    const double understeer_factor = 2.0;
};

#endif  // UGV_UGV_DRIVER_ESTOP_HPP_
