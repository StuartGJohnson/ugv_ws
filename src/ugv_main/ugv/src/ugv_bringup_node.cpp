#include "ugv/ugv_bringup.hpp"
#include "ugv/robot_tools.hpp"
#include "ugv/base_controller.hpp"

// Other includes used in the implementation
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include <nlohmann/json.hpp>

// Class methods implementation
UgvBringup::UgvBringup() : Node("ugv_bringup"), robot_tools_(), base_controller_(robot_tools_) {
    // Publishers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 100);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 100);
    odom_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("odom/odom_raw", 100);
    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("voltage", 50);

    // Find and open serial port and setup robot
    std::string serial_port_path = RobotTools::find_robot_tty(VENDOR_ID, PRODUCT_ID);
    if (serial_port_path.empty()) {
        RCLCPP_ERROR(get_logger(), "Robot not found with Vendor ID: %s, Product ID: %s", VENDOR_ID.c_str(), PRODUCT_ID.c_str());
        return;
    }

    if (!robot_tools_.open_serial(serial_port_path, 115200)) {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", serial_port_path.c_str());
        return;
    }
    robot_tools_.setup_robot();

    // Start the dedicated reading thread in BaseController
    base_controller_.start();

    // Timer for the feedback loop
    feedback_timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&UgvBringup::feedback_loop, this));
}

UgvBringup::~UgvBringup() {
    base_controller_.stop(); // Stop the reading thread
    robot_tools_.close_serial();
}

void UgvBringup::feedback_loop() {
    nlohmann::json data;
    // Get the latest message from the queue, discarding older ones.
    if (base_controller_.get_message_from_queue(data)) {
        if (data.contains("T") && data["T"] == 1001) {
            publish_imu_data(data);
            publish_mag_data(data);
            publish_odom_data(data);
            publish_voltage_data(data);
        }
    }
}

void UgvBringup::publish_imu_data(const nlohmann::json& data) {
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();
    msg->header.stamp = get_clock()->now();
    msg->header.frame_id = "base_imu_link";
    
    msg->linear_acceleration.x = 9.8 * data["ax"].get<float>() / 8192.0;
    msg->linear_acceleration.y = 9.8 * data["ay"].get<float>() / 8192.0;
    msg->linear_acceleration.z = 9.8 * data["az"].get<float>() / 8192.0;

    msg->angular_velocity.x = 3.1415926 * data["gx"].get<float>() / (16.4 * 180.0);
    msg->angular_velocity.y = 3.1415926 * data["gy"].get<float>() / (16.4 * 180.0);
    msg->angular_velocity.z = 3.1415926 * data["gz"].get<float>() / (16.4 * 180.0);
          
    imu_pub_->publish(std::move(msg));
}

void UgvBringup::publish_mag_data(const nlohmann::json& data) {
    auto msg = std::make_unique<sensor_msgs::msg::MagneticField>();
    msg->header.stamp = get_clock()->now();
    msg->header.frame_id = "base_imu_link";

    msg->magnetic_field.x = data["mx"].get<float>() * 0.15;
    msg->magnetic_field.y = data["my"].get<float>() * 0.15;
    msg->magnetic_field.z = data["mz"].get<float>() * 0.15;
          
    mag_pub_->publish(std::move(msg));
}

void UgvBringup::publish_odom_data(const nlohmann::json& data) {
    auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
    msg->data.push_back(data["L"].get<float>());
    msg->data.push_back(data["R"].get<float>());
    odom_pub_->publish(std::move(msg));
}

void UgvBringup::publish_voltage_data(const nlohmann::json& data) {
    auto msg = std::make_unique<std_msgs::msg::Float32>();
    msg->data = data["v"].get<float>() / 100.0;
    voltage_pub_->publish(std::move(msg));
}