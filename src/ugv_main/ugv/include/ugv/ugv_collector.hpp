// this is a ugv (node) but it collects and analyzes robot data, while
// issuing a sequence of commands


#ifndef UGV_UGV_COLLECTOR_HPP_
#define UGV_UGV_COLLECTOR_HPP_

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
#include <map>
#include <variant>

#include <vector>
#include <stdexcept>
#include <Eigen/Dense>

struct LinearFitResult {
    double slope;
    double intercept;
    std::vector<double> y_fit;
    std::vector<double> x_fit;
};

LinearFitResult linear_least_squares_fit(
    const std::vector<double>& x,
    const std::vector<double>& y);

struct CmdVel {
    double left_rps;
    double right_rps;
};


struct CmdPID {
    double p;
    double i;
    double d;
    double l;
};

struct CmdFF {
    double gain;
    double offset;
};

using Command = std::variant<CmdVel, CmdPID, CmdFF>;

class UgvCollector : public rclcpp::Node {
public:
    UgvCollector(std::map<int, Command> commands);
    ~UgvCollector();
    
    bool is_complete() const;
    double last_cmd_m_per_sec_left() const;
    double last_cmd_m_per_sec_right() const;

    void plot();
    void plot_calibrate(const std::vector<double>& v);
    std::vector<size_t> find_indices(const std::vector<double>& targets, const std::vector<double>& v);

private:

    std::map<int, Command> commands_;

    std::atomic<bool> complete_;
    std::atomic<double> last_cmd_left_;
    std::atomic<double> last_cmd_right_;

    void set_timestamp(const nlohmann::json& data);
    void publish_imu_data(const nlohmann::json& data);
    void publish_mag_data(const nlohmann::json& data);
    void publish_odom_data(const nlohmann::json& data);
    void publish_voltage_data(const nlohmann::json& data);
    void publish_odom(const nlohmann::json& data);
    // --- Callbacks ---
    void feedback_loop(); // Fast timer for reading sensor data
    void clock_sync(); // function for syncing clock on MCU
    void command_loop(); // Slow timer for sending robot command sequences
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

    // storage (this is a collector!)
    std::vector<double> enc_m_per_sec_left;
    std::vector<double> cmd_m_per_sec_left;
    std::vector<double> pwm_left;
    std::vector<double> enc_m_per_sec_right;
    std::vector<double> cmd_m_per_sec_right;
    std::vector<double> pwm_right;
    std::vector<double> voltage;
    std::vector<int64_t> time_vec_raw;
    std::vector<double> time_vec;

    // --- Subscriptions ---
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr led_ctrl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr voltage_sub_;

    // --- Timers ---
    rclcpp::TimerBase::SharedPtr feedback_timer_; // Fast timer for sensor feedback
    rclcpp::TimerBase::SharedPtr command_timer_; // Slow timer for command sequence

    // --- Serial Communication Components ---
    RobotTools robot_tools_;
    BaseController base_controller_;

    // odometry
    bool pub_odom_tf_ = false;
    std::string odom_topic = "odom";
    std::string odom_frame = "odom";
    std::string base_footprint_frame = "base_footprint";
    Ekf1 ekf1;

    // --- Node State ---
    std::atomic<bool> estop_;
    rclcpp::Time last_clock_sync_sent_time_; // Stores ROS2 time when clock sync was sent
    rclcpp::Time robot_timestamp_; // computed timestamp from robot update
    rclcpp::Time last_timestamp_; // computed timestamp from robot update
    bool last_timestamp_set_;
    int command_step_;
    int iter_count;
    int64_t first_timestamp;
    bool first_timestamp_set;


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

#endif  // UGV_UGV_COLLECTOR_HPP_