#include <gtest/gtest.h>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ugv/ekf1.hpp"

#include "ugv/gnuplot.hpp"

namespace
{
double stamp_to_sec(const builtin_interfaces::msg::Time & st)
{
  return static_cast<double>(st.sec) + static_cast<double>(st.nanosec) * 1e-9;
}
double bagtime_ns_to_sec(const int64_t bag_time_ns)
{
  return static_cast<double>(bag_time_ns) * 1e-9;
}

double yaw_from_quat_wxyz(nav_msgs::msg::Odometry odom_msg)
{
  double w = odom_msg.pose.pose.orientation.w;
  double x = odom_msg.pose.pose.orientation.x;
  double y = odom_msg.pose.pose.orientation.y;
  double z = odom_msg.pose.pose.orientation.z;
  // Standard yaw (Z) from quaternion.
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

enum class EventType : uint8_t { IMU, ODOM, WS, CV };

using EventMsg = std::variant<
  sensor_msgs::msg::Imu::SharedPtr,
  nav_msgs::msg::Odometry::SharedPtr,
  geometry_msgs::msg::Twist::SharedPtr,
  std_msgs::msg::Float32MultiArray::SharedPtr
>;

struct Event
{
  double t_sec{};
  EventType type{};
  EventMsg msg{};
};

template <typename MsgT>
typename MsgT::SharedPtr deserialize_as_copy(
  const rosbag2_storage::SerializedBagMessage & bag_msg,
  rclcpp::Serialization<MsgT> & serializer)
{
  const auto & src_ptr = bag_msg.serialized_data;          // shared_ptr<rcutils_uint8_array_t>
  if (!src_ptr || !src_ptr->buffer || src_ptr->buffer_length == 0) {
    throw std::runtime_error("Empty serialized_data in bag message");
  }

  // Allocate our own buffer and COPY bytes into it.
  rclcpp::SerializedMessage serialized(src_ptr->buffer_length);
  auto & rcl_ser = serialized.get_rcl_serialized_message();

  std::memcpy(rcl_ser.buffer, src_ptr->buffer, src_ptr->buffer_length);
  rcl_ser.buffer_length = src_ptr->buffer_length;

  auto out = std::make_shared<MsgT>();
  serializer.deserialize_message(&serialized, out.get());
  return out;
}

struct dataPacket {
  bool imu_valid;
  bool ws_valid;
  sensor_msgs::msg::Imu imu;
  std_msgs::msg::Float32MultiArray ws;
  double ws_tsec;
  double imu_tsec;
};

std::vector<Event> load_events_from_bag(const std::string & bag_path, const std::string & storage_id)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = storage_id;

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::Reader reader;
  reader.open(storage_options, converter_options);

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rclcpp::Serialization<nav_msgs::msg::Odometry> odom_ser;
  rclcpp::Serialization<geometry_msgs::msg::Twist> twist_ser;
  rclcpp::Serialization<std_msgs::msg::Float32MultiArray> f32_ser;

  const std::unordered_map<std::string, EventType> topic_to_type = {
    {"/imu/data_raw",  EventType::IMU},
    {"/odom",          EventType::ODOM},
    {"/odom/odom_raw", EventType::WS},
    {"/cmd_vel",       EventType::CV},
  };

  std::vector<Event> events;
  while (reader.has_next()) {
    auto bag_msg_ptr = reader.read_next();
    if (!bag_msg_ptr) continue;

    const auto it = topic_to_type.find(bag_msg_ptr->topic_name);
    if (it == topic_to_type.end()) continue;

    const double bag_t_sec = bagtime_ns_to_sec(bag_msg_ptr->time_stamp);
    const auto & topic = bag_msg_ptr->topic_name;

    if (topic == "/imu/data_raw") {
      auto msg = deserialize_as_copy(*bag_msg_ptr, imu_ser);
      events.push_back(Event{stamp_to_sec(msg->header.stamp), it->second, msg});
    } else if (topic == "/odom") {
      auto msg = deserialize_as_copy(*bag_msg_ptr, odom_ser);
      auto event = Event{stamp_to_sec(msg->header.stamp), it->second, msg};
      events.push_back(event);
    } else if (topic == "/odom/odom_raw") {
      // adjust type as needed; placeholder: nav_msgs/Odometry
      auto msg = deserialize_as_copy(*bag_msg_ptr, f32_ser);
      events.push_back(Event{bag_t_sec, it->second, msg});
    } else if (topic == "/cmd_vel") {
      auto msg = deserialize_as_copy(*bag_msg_ptr, twist_ser);
      events.push_back(Event{bag_t_sec, it->second, msg});
    }
  }

  std::sort(events.begin(), events.end(),
            [](const Event & a, const Event & b) { return a.t_sec < b.t_sec; });

  return events;
}
}  // namespace

TEST(EkfReplay, BagCase01)
{
  // Resolve package share dir, then point to test/data
  const std::string share = ament_index_cpp::get_package_share_directory("ugv");
  const std::string bag_data = share + "/test/data/dc_bag_20260124_193242/dc_bag_20260124_193242_0.mcap";
  //const std::string bag_data = "/home/sjohnson/ugv/trajectory_data_collector/dc_bag_20260124_193242/dc_bag_20260124_193242_0.mcap";

  // If you're using MCAP, pass "mcap" here.
  const std::string storage_id = "mcap";

  auto events = load_events_from_bag(bag_data, storage_id);
  ASSERT_GT(events.size(), 100u) << "No events loaded from bag: " << bag_data;
  printf("events loaded: %ld \n", events.size());

  // Create your EKF core / node-under-test.
  // Prefer a pure C++ class with deterministic stepping.
  // your_pkg::EkfCore ekf;  // <- adapt to your API
  // ekf.reset();
  Ekf1 ekf;

  // Optional: record outputs for assertions (trajectory, final state, etc.)
  // std::vector<double> est_t;
  // std::vector<double> est_x;
  // std::vector<double> est_y;

  // collect packets and process
  dataPacket data_packet;
  data_packet.imu_valid = false;
  data_packet.ws_valid = false;

  // ground truth (/odom)
  std::vector<double> gt_t;
  std::vector<double> gt_x;
  std::vector<double> gt_y;
  std::vector<double> gt_yaw;

  // ekf estimates
  std::vector<double> ekf_t;
  std::vector<double> ekf_x;
  std::vector<double> ekf_y;
  std::vector<double> ekf_yaw;

  double t0 = -1.0;
  double t_min = 7;
  double t_max = 10;
  int ind_min = -1;
  int ind_max = -1;

  ekf1Init ekf_init;
  ekf_init.x = Vector3d({0, 0, 0});
  ekf_init.v_lr_mps = Vector2d({0, 0});
  ekf_init.t = 0;
  bool init_done = false;

  // first pass - collect ground truth odometry and t0
  size_t ind = 0;
  for (const auto & e : events) {
    switch (e.type) {
      case EventType::ODOM: {
        auto msg = *std::get<nav_msgs::msg::Odometry::SharedPtr>(e.msg);
        if (t0 == -1.0)
        {
          t0 = e.t_sec;
        }
        auto t_current = e.t_sec - t0;
        gt_t.push_back(t_current);
        auto x = msg.pose.pose.position.x;
        auto y = msg.pose.pose.position.y;
        auto yaw = yaw_from_quat_wxyz(msg);
        gt_x.push_back(x);
        gt_y.push_back(y);
        gt_yaw.push_back(yaw);
        if (!init_done && t_current >= t_min)
        {
          ind_min = ind;
          ekf_init.x = Vector3d({x, y, yaw});
          ekf_init.v_lr_mps = Vector2d({0, 0});
          ekf_init.t = t_current;
          init_done = true;
        }
        if (ind_max==-1 && t_current > t_max)
        {
          ind_max = ind;
        }
        ind++;
      }
      default:
        break;
    }
  }

  ekf.Init(ekf_init);

  for (const auto & e : events) {
    switch (e.type) {
      case EventType::IMU: {
        data_packet.imu = *std::get<sensor_msgs::msg::Imu::SharedPtr>(e.msg);
        data_packet.imu_valid = true;
        data_packet.imu_tsec = e.t_sec;
        break;
      }
      case EventType::ODOM: {
        auto msg = *std::get<nav_msgs::msg::Odometry::SharedPtr>(e.msg);
        break;
      }
      case EventType::WS: {
        data_packet.ws = *std::get<std_msgs::msg::Float32MultiArray::SharedPtr>(e.msg);
        data_packet.ws_valid = true;
        data_packet.ws_tsec = e.t_sec;
        break;
      }
      case EventType::CV: {
        auto msg = std::get<geometry_msgs::msg::Twist::SharedPtr>(e.msg);
        break;
      }
      default:
        break;
    }

    if (data_packet.imu_valid && data_packet.ws_valid) {
      // we have a packet - note that the way the robot code
      // works, there is never an imu,ws that are unpaired.
      // at least, if you start from the beginning of the bag
      // file.
      auto t_imu = data_packet.imu_tsec;
      auto t_ws = data_packet.ws_tsec;
      auto t_current = std::min(t_imu, t_ws) - t0;
      //printf("t_current: %f \n", t_current);

      if (t_current > ekf_init.t && t_current <= t_max) {
        // process the packet via the ekf
        ekf1Data ekf_data;
        ekf_data.tSec = t_current;
        ekf_data.gz_rps = data_packet.imu.angular_velocity.z;
        ekf_data.v_lr_mps = Vector2d(data_packet.ws.data[0], data_packet.ws.data[1]);
        ekf1Out ekf_out = ekf.Update(ekf_data);
        ekf_t.push_back(t_current);
        ekf_x.push_back(ekf_out.x(0));
        ekf_y.push_back(ekf_out.x(1));
        ekf_yaw.push_back(ekf_out.x(2));
      }

      //printf("here\n");
      // invalidate the packet
      data_packet.imu_valid = false;
      data_packet.ws_valid = false;
    }

  }

  GnuplotPipe gp;
  gp.plot_two_xy(gt_x,gt_y,ekf_x,ekf_y, "GT vs EKF traj.","x","y",0);

  // zoom in on gt in t
  std::vector<double> gt_t_zoom(gt_t.begin() + ind_min, gt_t.begin() + ind_max + 1);
  std::vector<double> gt_yaw_zoom(gt_yaw.begin() + ind_min, gt_yaw.begin() + ind_max + 1);
  // plot yaw - much smoother!
  gp.plot_two_ty(gt_t_zoom,gt_yaw_zoom,ekf_t,ekf_yaw, "GT vs EKF yaw","time(s)","rad", 1);

  // zoom in on x and y (to check time alignment)
  std::vector<double> gt_x_zoom(gt_x.begin() + ind_min, gt_x.begin() + ind_max + 1);
  std::vector<double> gt_y_zoom(gt_y.begin() + ind_min, gt_y.begin() + ind_max + 1);
  gp.plot_two_xy(gt_x_zoom,gt_y_zoom,ekf_x,ekf_y, "GT vs EKF traj. detail","x","y", 2);

}

TEST(EkfTest, PlotSomething) {
  // this makes a plot via gnuplot - which, for me, displays
  // via some remote window magic (which must come down to x11).

  XY traj = make_sinc_data();
  GnuplotPipe gp;
  gp.plot_xy(traj.x, traj.y, "EKF x(t)", "x", "y", 0);
}

TEST(EkfTest, TestGPTcontention)
{
  // double check counter roll-over correction (on this computer)
  std::uint32_t correction = std::uint32_t(std::pow(2,32)-1);
  std::uint32_t x = correction - 22;
  std::uint32_t y = 5;
  std::uint32_t z = y+(correction-x+1);
  std::uint32_t yx = y-x;
  std::cerr << z << "\n";
  std::cerr << yx << "\n";
  std::cerr << x << "\n";
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
