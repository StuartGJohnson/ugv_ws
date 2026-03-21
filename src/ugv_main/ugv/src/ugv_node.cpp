#include "ugv/ugv.hpp"

// Other includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ugv/robot_tools.hpp"
#include "ugv/base_controller.hpp"
#include "ugv/ekf1.hpp"
#include <nlohmann/json.hpp>
#include <cstdlib> // For std::system

Ugv::Ugv()
    : rclcpp::Node("ugv"), robot_tools_(), base_controller_(robot_tools_), ekf1(), estop_(false) {

    // --- Declare and Get Parameters ---
    this->declare_parameter("vendor_id", "1a86");
    this->declare_parameter("product_id", "55d3");
    this->declare_parameter("low_battery_sound_path", "/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav"); // Default path from Python
    this->declare_parameter("wheel_separation", 0.174);
    this->declare_parameter("understeer_factor", 2.0);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
    this->declare_parameter<bool>("pub_odom_tf", false);

    vendor_id_ = this->get_parameter("vendor_id").as_string();
    product_id_ = this->get_parameter("product_id").as_string();
    low_battery_sound_path_ = this->get_parameter("low_battery_sound_path").as_string();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    understeer_factor_ = this->get_parameter("understeer_factor").as_double();
    this->get_parameter<bool>("pub_odom_tf", pub_odom_tf_);
    this->get_parameter<std::string>("odom_frame", odom_frame);
    this->get_parameter<std::string>("base_footprint_frame", base_footprint_frame);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // --- Create Callback Groups ---
    read_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    write_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // --- Publishers ---
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 100);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 100);
    odom_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("odom/odom_raw", 100);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);
    voltage_pub_ = create_publisher<std_msgs::msg::Float32>("voltage", 50);

    // --- Subscriptions ---
    // Assign subscriptions to write_cb_group_
    auto write_sub_opt = rclcpp::SubscriptionOptions();
    write_sub_opt.callback_group = write_cb_group_;

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Ugv::joy_callback, this, std::placeholders::_1), write_sub_opt);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&Ugv::cmd_vel_callback, this, std::placeholders::_1), write_sub_opt);
    led_ctrl_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>( 
        "ugv/led_ctrl", 10, std::bind(&Ugv::led_ctrl_callback, this, std::placeholders::_1), write_sub_opt);
//    voltage_sub_ = create_subscription<std_msgs::msg::Float32>(
//        "voltage", 10, std::bind(&Ugv::voltage_callback, this, std::placeholders::_1), write_sub_opt);


    // --- Find and Open Serial Port ---
    std::string serial_port_path = RobotTools::find_robot_tty(vendor_id_, product_id_);
    if (serial_port_path.empty()) {
        RCLCPP_ERROR(get_logger(), "Robot not found with Vendor ID: %s, Product ID: %s", vendor_id_.c_str(), product_id_.c_str());
        return; // Node cannot operate without serial port
    }

    if (!robot_tools_.open_serial(serial_port_path, 115200)) {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port %s", serial_port_path.c_str());
        return; // Node cannot operate without serial port
    }
    robot_tools_.setup_robot();

    clock_sync();

    // Start the dedicated reading thread in BaseController
    base_controller_.start();

    // startup the EKF
    ekf1.Init();

    // --- Timers ---
    // Feedback loop timer (fast) assigned to read_cb_group_
    feedback_timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&Ugv::feedback_loop, this),
        read_cb_group_);

    // Clock sync timer (slow) assigned to write_cb_group_
    clock_sync_timer_ = create_wall_timer(
        std::chrono::seconds(60), // Every 1 minute
        std::bind(&Ugv::clock_sync, this),
        write_cb_group_);
}

Ugv::~Ugv() {
    base_controller_.stop(); // Stop the reading thread
    robot_tools_.close_serial();
}

// --- Feedback Loop (from ugv_bringup) ---
void Ugv::feedback_loop() {
    nlohmann::json data;
    if (base_controller_.get_message_from_queue(data)) {
        if (data.contains("T") && data["T"] == 1001) {
            set_timestamp(data);
            publish_odom(data);
            publish_imu_data(data);
            publish_mag_data(data);
            publish_odom_data(data);
            publish_voltage_data(data);
        }
        // TODO: Handle clock sync response if robot sends it back.
        // If data contains clock sync response (e.g. {"T":42, "time_us":...}), compare with last_clock_sync_sent_time_
    }
}

void Ugv::publish_imu_data(const nlohmann::json &data)
{
  auto msg = std::make_unique<sensor_msgs::msg::Imu>();
  msg->header.stamp = robot_timestamp_;
  msg->header.frame_id = "base_imu_link";

  msg->linear_acceleration.x = 9.8 * data["ax"].get<float>() / 8192.0;
  msg->linear_acceleration.y = 9.8 * data["ay"].get<float>() / 8192.0;
  msg->linear_acceleration.z = 9.8 * data["az"].get<float>() / 8192.0;

  msg->angular_velocity.x = 3.1415926 * data["gx"].get<float>() / (16.4 * 180.0);
  msg->angular_velocity.y = 3.1415926 * data["gy"].get<float>() / (16.4 * 180.0);
  msg->angular_velocity.z = 3.1415926 * data["gz"].get<float>() / (16.4 * 180.0);

  imu_pub_->publish(std::move(msg));
}

void Ugv::publish_mag_data(const nlohmann::json &data)
{
  auto msg = std::make_unique<sensor_msgs::msg::MagneticField>();
  msg->header.stamp = robot_timestamp_;
  msg->header.frame_id = "base_imu_link";

  msg->magnetic_field.x = data["mx"].get<float>() * 0.15;
  msg->magnetic_field.y = data["my"].get<float>() * 0.15;
  msg->magnetic_field.z = data["mz"].get<float>() * 0.15;

  mag_pub_->publish(std::move(msg));
}

void Ugv::publish_odom_data(const nlohmann::json &data)
{
  auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
  msg->data.push_back(data["L"].get<float>());
  msg->data.push_back(data["R"].get<float>());
  odom_pub_->publish(std::move(msg));
}

void Ugv::set_timestamp(const nlohmann::json &data)
{
  double robot_stamp = data["tsec"].get<float>();
  rclcpp::Duration dtSec = rclcpp::Duration::from_seconds(robot_stamp);
  robot_timestamp_  = last_clock_sync_sent_time_ + dtSec;
}

void Ugv::publish_odom(const nlohmann::json &data)
{
  ekf1Data ekf_data;
  ekf_data.gz_rps = 3.1415926 * data["gz"].get<float>() / (16.4 * 180.0);
  ekf_data.v_lr_mps = Vector2d({data["L"].get<float>(), data["R"].get<float>()});
  ekf_data.tSec = robot_timestamp_.seconds();

  ekf1Out ekf_out = ekf1.Update(ekf_data);

  if (!ekf_out.valid) return;

  // extract the latest estimates and publish
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = robot_timestamp_;
  odom.header.frame_id = odom_frame;
  odom.child_frame_id = base_footprint_frame;

  double x_pos = ekf_out.x(0);
  double y_pos = ekf_out.x(1);
  double yaw = ekf_out.x(2);

  // Robot's position in x, y, and z
  odom.pose.pose.position.x = x_pos;
  odom.pose.pose.position.y = y_pos;
  odom.pose.pose.position.z = 0.0;

  // Robot's heading in quaternion
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = sin(yaw / 2.0);
  odom.pose.pose.orientation.w = cos(yaw / 2.0);

  odom.twist.twist.linear.x = ekf_out.twist[0];
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = ekf_out.twist[1];

  Eigen::Matrix<double, 6, 6> odom_cov = Eigen::Matrix<double, 6, 6>::Zero();
  odom_cov.block<3,3>(0, 0) = ekf_out.P.block<3,3>(0, 0);
  eigen_to_ros_cov(odom_cov, odom.pose.covariance);

  Eigen::Matrix<double, 6, 6> twist_cov = Eigen::Matrix<double, 6, 6>::Zero();
  twist_cov(0,0) = ekf_out.Pt(0,0);
  twist_cov(0,5) = ekf_out.Pt(0,1);
  twist_cov(5,0) = ekf_out.Pt(1,0);
  twist_cov(5,5) = ekf_out.Pt(1,1);
  eigen_to_ros_cov(twist_cov, odom.twist.covariance);

  odom_publisher_->publish(odom);

  if (pub_odom_tf_)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = robot_timestamp_;
    t.header.frame_id = odom_frame;
    t.child_frame_id = base_footprint_frame;
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = sin(yaw / 2.0);
    t.transform.rotation.w = cos(yaw / 2.0);
    tf_broadcaster_->sendTransform(t);
  }
}

void Ugv::publish_voltage_data(const nlohmann::json& data) {
    auto msg = std::make_unique<std_msgs::msg::Float32>();
    msg->data = data["v"].get<float>() / 100.0;
    voltage_pub_->publish(std::move(msg));
}

// --- Clock Sync Loop ---
void Ugv::clock_sync() {
    nlohmann::json clock_sync_cmd;
    clock_sync_cmd["T"] = 42;
    
    // Store ROS2 time *just before* sending the signal
    last_clock_sync_sent_time_ = this->get_clock()->now();
    robot_tools_.send_command(clock_sync_cmd);
    // RCLCPP_DEBUG(get_logger(), "Sent clock sync command. Stored time: %f", last_clock_sync_sent_time_.seconds());
}

// --- cmd_vel Callback (from ugv_driver_estop) ---
void Ugv::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (estop_) return;

    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    // Use parameterized wheel_separation and understeer_factor
    if (linear_velocity == 0) {
        if (0 < angular_velocity && angular_velocity < 0.2) {
            angular_velocity = 0.2;
        } else if (-0.2 < angular_velocity && angular_velocity < 0) {
            angular_velocity = -0.2;
        }
    }
    
    double left_vel = linear_velocity - angular_velocity * wheel_separation_ * understeer_factor_ / 2.0;
    double right_vel = linear_velocity + angular_velocity * wheel_separation_ * understeer_factor_ / 2.0;

    nlohmann::json cmd;
    cmd["T"] = "1";
    cmd["L"] = left_vel;
    cmd["R"] = right_vel;
    robot_tools_.send_command(cmd);
}

void Ugv::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
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

void Ugv::led_ctrl_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        nlohmann::json cmd;
        cmd["T"] = 132; 
        cmd["IO4"] = msg->data[0];
        cmd["IO5"] = msg->data[1];
        robot_tools_.send_command(cmd);
    }
}

void Ugv::voltage_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (0.1 < msg->data && msg->data < 9.0) { 
        // Use parameterized sound path
        std::string command = "aplay -D plughw:2,0 " + low_battery_sound_path_;
        std::system(command.c_str());
    }
}
