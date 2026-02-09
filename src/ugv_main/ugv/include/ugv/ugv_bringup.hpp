#ifndef UGV_UGV_BRINGUP_HPP_
#define UGV_UGV_BRINGUP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "ugv/robot_tools.hpp"
#include "ugv/base_controller.hpp"
#include <nlohmann/json.hpp>

class UgvBringup : public rclcpp::Node {
public:
    UgvBringup();
    ~UgvBringup();

private:
    void feedback_loop();
    void publish_imu_data(const nlohmann::json& data);
    void publish_mag_data(const nlohmann::json& data);
    void publish_odom_data(const nlohmann::json& data);
    void publish_voltage_data(const nlohmann::json& data);

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_pub_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    RobotTools robot_tools_;
    BaseController base_controller_;

    // Vendor and product IDs should ideally be configurable via ROS parameters.
    const std::string VENDOR_ID = "1a86"; // Example Vendor ID
    const std::string PRODUCT_ID = "55d3"; // Example Product ID
};

#endif  // UGV_UGV_BRINGUP_HPP_