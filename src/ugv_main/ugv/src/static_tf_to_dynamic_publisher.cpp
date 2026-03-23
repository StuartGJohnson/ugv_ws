#include "ugv/static_tf_to_dynamic_publisher.hpp"

#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

StaticTfToDynamicPublisher::StaticTfToDynamicPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("static_tf_to_dynamic_publisher", options),
  publish_rate_hz_(40.0),
  transform_cached_(false)
{
  source_parent_frame_ = this->declare_parameter<std::string>("source_parent_frame", "base_link");
  source_child_frame_ = this->declare_parameter<std::string>("source_child_frame", "lidar_link");
  output_parent_frame_ = this->declare_parameter<std::string>("output_parent_frame", source_parent_frame_);
  output_child_frame_ = this->declare_parameter<std::string>("output_child_frame", source_child_frame_);
  publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 40.0);

  if (source_parent_frame_.empty() || source_child_frame_.empty()) {
    throw std::runtime_error("source_parent_frame and source_child_frame must not be empty");
  }
  if (output_parent_frame_.empty() || output_child_frame_.empty()) {
    throw std::runtime_error("output_parent_frame and output_child_frame must not be empty");
  }
  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("publish_rate_hz must be > 0");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  init_timer_ = this->create_wall_timer(
    500ms, std::bind(&StaticTfToDynamicPublisher::initialize_transform_, this));

  RCLCPP_INFO(
    get_logger(),
    "Waiting to cache TF %s -> %s; will republish as %s -> %s at %.2f Hz",
    source_parent_frame_.c_str(), source_child_frame_.c_str(),
    output_parent_frame_.c_str(), output_child_frame_.c_str(),
    publish_rate_hz_);
}

void StaticTfToDynamicPublisher::initialize_transform_()
{
  if (transform_cached_) {
    return;
  }

  geometry_msgs::msg::TransformStamped looked_up;
  try {
    looked_up = tf_buffer_->lookupTransform(
      source_parent_frame_,
      source_child_frame_,
      tf2::TimePointZero);

    cached_transform_ = looked_up.transform;
    transform_cached_ = true;

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    pub_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StaticTfToDynamicPublisher::publish_transform_, this));

    init_timer_->cancel();

    RCLCPP_INFO(
      get_logger(),
      "Cached TF %s -> %s: xyz=(%.6f, %.6f, %.6f), quat=(%.6f, %.6f, %.6f, %.6f)",
      source_parent_frame_.c_str(), source_child_frame_.c_str(),
      cached_transform_.translation.x,
      cached_transform_.translation.y,
      cached_transform_.translation.z,
      cached_transform_.rotation.x,
      cached_transform_.rotation.y,
      cached_transform_.rotation.z,
      cached_transform_.rotation.w);

  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Still waiting for TF %s -> %s: %s",
      source_parent_frame_.c_str(),
      source_child_frame_.c_str(),
      e.what());
  }
}

void StaticTfToDynamicPublisher::publish_transform_()
{
  if (!transform_cached_) {
    return;
  }

  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = this->get_clock()->now();
  out.header.frame_id = output_parent_frame_;
  out.child_frame_id = output_child_frame_;
  out.transform = cached_transform_;

  tf_broadcaster_->sendTransform(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTfToDynamicPublisher>());
  rclcpp::shutdown();
  return 0;
}