#include "ugv/ugv_driver_estop.hpp"
#include "ugv/robot_tools.hpp"
#include <nlohmann/json.hpp>

// Other includes used in the implementation
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cstdlib> // For std::system

// Class methods implementation
UgvDriverEstop::UgvDriverEstop() : Node("ugv_driver_estop"), estop_(false) {
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&UgvDriverEstop::joy_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&UgvDriverEstop::cmd_vel_callback, this, std::placeholders::_1));
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "ugv/joint_states", 10, std::bind(&UgvDriverEstop::joint_states_callback, this, std::placeholders::_1));
    led_ctrl_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>( 
        "ugv/led_ctrl", 10, std::bind(&UgvDriverEstop::led_ctrl_callback, this, std::placeholders::_1));
    voltage_sub_ = create_subscription<std_msgs::msg::Float32>(
        "voltage", 10, std::bind(&UgvDriverEstop::voltage_callback, this, std::placeholders::_1));

    // Find and open serial port and setup robot
    std::string serial_port_path = RobotTools::find_robot_tty(DRIVER_VENDOR_ID, DRIVER_PRODUCT_ID);
    if (serial_port_path.empty()) {
        RCLCPP_ERROR(get_logger(), "Robot not found with Vendor ID: %s, Product ID: %s", DRIVER_VENDOR_ID.c_str(), DRIVER_PRODUCT_ID.c_str());
        return;
    }

    if (!robot_tools_.open_serial(serial_port_path, 115200)) {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", serial_port_path.c_str());
        return;
    }
    robot_tools_.setup_robot();
}

UgvDriverEstop::~UgvDriverEstop() {
    robot_tools_.close_serial();
}

void UgvDriverEstop::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (estop_) return;

    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    if (linear_velocity == 0) {
        if (0 < angular_velocity && angular_velocity < 0.2) {
            angular_velocity = 0.2;
        } else if (-0.2 < angular_velocity && angular_velocity < 0) {
            angular_velocity = -0.2;
        }
    }
    
    double left_vel = linear_velocity - angular_velocity * wheel_separation * understeer_factor / 2.0;
    double right_vel = linear_velocity + angular_velocity * wheel_separation * understeer_factor / 2.0;

    nlohmann::json cmd;
    cmd["T"] = "1";
    cmd["L"] = left_vel;
    cmd["R"] = right_vel;
    robot_tools_.send_command(cmd);
}

void UgvDriverEstop::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    bool x_button = msg->buttons[3] > 0;
    bool y_button = msg->buttons[4] > 0;

    if (x_button) {
        RCLCPP_WARN(get_logger(), "Estop!");
        estop_ = true;
        robot_tools_.reset_robot();
    }
    if (y_button) {
        RCLCPP_WARN(get_logger(), "Un-Estop!");
        estop_ = false;
    }
}

void UgvDriverEstop::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (estop_) return;

    auto it_x = std::find(msg->name.begin(), msg->name.end(), "pt_base_link_to_pt_link1");
    auto it_y = std::find(msg->name.begin(), msg->name.end(), "pt_link1_to_pt_link2");

    if (it_x != msg->name.end() && it_y != msg->name.end()) {
        double x_rad = msg->position[std::distance(msg->name.begin(), it_x)];
        double y_rad = msg->position[std::distance(msg->name.begin(), it_y)];

        double x_degree = (180.0 * x_rad) / 3.1415926;
        double y_degree = (180.0 * y_rad) / 3.1415926;

        nlohmann::json cmd;
        cmd["T"] = 134; 
        cmd["X"] = x_degree; 
        cmd["Y"] = y_degree; 
        cmd["SX"] = 600;
        cmd["SY"] = 600;
        robot_tools_.send_command(cmd);
    }
}

void UgvDriverEstop::led_ctrl_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        nlohmann::json cmd;
        cmd["T"] = 132; 
        cmd["IO4"] = msg->data[0];
        cmd["IO5"] = msg->data[1];
        robot_tools_.send_command(cmd);
    }
}

void UgvDriverEstop::voltage_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (0.1 < msg->data && msg->data < 9.0) { 
        // The python code has a hardcoded path. This is not ideal.
        // A better solution would be to use a ROS parameter for the path.
        std::system("aplay -D plughw:2,0 /home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav");
    }
}