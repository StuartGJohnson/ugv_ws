#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

class StaticTfToDynamicPublisher : public rclcpp::Node
{
public:
  explicit StaticTfToDynamicPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void initialize_transform_();
  void publish_transform_();

  std::string source_parent_frame_;
  std::string source_child_frame_;
  std::string output_parent_frame_;
  std::string output_child_frame_;
  double publish_rate_hz_;

  bool transform_cached_;
  geometry_msgs::msg::Transform cached_transform_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};