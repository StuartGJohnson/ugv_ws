#ifndef UGV_UGV_HPP_
#define UGV_UGV_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "ugv/robot_tools.hpp"
#include "ugv/base_controller.hpp"
#include "ugv/ekf1.hpp"
#include <nlohmann/json.hpp>
#include <atomic> // For std::atomic<bool>
#include <cstdlib> // For std::system

class Ugv : public rclcpp::Node {
public:
    Ugv();
    ~Ugv();

private:
    void set_timestamp(const nlohmann::json& data);
    void publish_imu_data(const nlohmann::json& data);
    void publish_mag_data(const nlohmann::json& data);
    void publish_odom_data(const nlohmann::json& data);
    void publish_voltage_data(const nlohmann::json& data);
    void publish_odom(const nlohmann::json& data);
    // --- Callbacks ---
    void feedback_loop(); // Fast timer for reading sensor data
    void clock_sync(); // Slow timer for sending clock sync
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void led_ctrl_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void voltage_callback(const std_msgs::msg::Float32::SharedPtr msg);

    // --- Publishers ---
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // --- Subscriptions ---
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr led_ctrl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltage_sub_;

    // --- Timers ---
    rclcpp::TimerBase::SharedPtr feedback_timer_; // Fast timer for sensor feedback
    rclcpp::TimerBase::SharedPtr clock_sync_timer_; // Slow timer for clock synchronization

    // --- Serial Communication Components ---
    RobotTools robot_tools_;
    BaseController base_controller_;

    // odometry
    bool pub_odom_tf_ = false;
    std::string odom_frame = "odom";
    std::string base_footprint_frame = "base_footprint";
    Ekf1 ekf1;

    // --- Node State ---
    std::atomic<bool> estop_;
    rclcpp::Time last_clock_sync_sent_time_; // Stores ROS2 time when clock sync was sent
    rclcpp::Time robot_timestamp_; // computed timestamp from robot update

    // --- Parameters ---
    std::string vendor_id_;
    std::string product_id_;
    std::string low_battery_sound_path_;
    double wheel_separation_;
    double understeer_factor_;

    // --- Callback Groups ---
    rclcpp::CallbackGroup::SharedPtr read_cb_group_; // For feedback_loop timer and sensor pubs
    rclcpp::CallbackGroup::SharedPtr write_cb_group_; // For cmd_vel, joy, joint_states, led_ctrl, clock_sync timer
};

#endif  // UGV_UGV_HPP_