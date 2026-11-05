// Copyright 2026 Ali Pahlevani
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// relocalization_check: plays one window of the depot fixture (simulated time, real-time rate)
// into a running map_server + AMCL + landfall_relocalizer stack, asks the relocalizer for a pose,
// and waits until AMCL, started without any pose, converges on the ground truth.
//
//   relocalization_check BAG START_S END_S
//
// The recorded map->odom (the recording's own AMCL) is dropped, so the AMCL under test is the only
// localization. Exit 0 when AMCL's pose is within 0.20 m / 3 deg of the truth twice in a row.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/eval/trajectory.hpp"
#include "landfall/ros/compat/rosbag2.hpp"
#include "landfall/ros/conversions.hpp"
#include "landfall_msgs/srv/relocalize.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{

using Clock = std::chrono::steady_clock;
using landfall::Pose2;
using Result = landfall_msgs::msg::RelocalizationResult;

template <typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  static const rclcpp::Serialization<MessageT> serialization;
  const rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  MessageT message;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

struct Recorded
{
  int64_t stamp_ns = 0;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> message;
};

int check(int argc, char ** argv)
{
  if (argc != 4) {
    std::fprintf(stderr, "usage: relocalization_check BAG START_S END_S\n");
    return 2;
  }
  std::setvbuf(stdout, nullptr, _IOLBF, 0);  // launch_testing captures a pipe
  const std::string bag = argv[1];
  const auto start_ns = static_cast<int64_t>(std::stod(argv[2]) * 1e9);
  const auto end_ns = static_cast<int64_t>(std::stod(argv[3]) * 1e9);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("relocalization_check");

  // The window's messages in recording order, the static transforms and the ground truth. The
  // reader outlives the messages: their buffers are freed by its storage plugin.
  rosbag2_cpp::Reader reader;
  std::vector<Recorded> messages;
  tf2_msgs::msg::TFMessage statics;
  landfall::eval::Trajectory truth;
  {
    reader.open(bag);
    while (reader.has_next()) {
      auto m = reader.read_next();
      const int64_t t = landfall::ros::compat::receiveTimestamp(*m);
      if (m->topic_name == "/tf_static") {
        const auto s = deserialize<tf2_msgs::msg::TFMessage>(*m);
        statics.transforms.insert(
          statics.transforms.end(), s.transforms.begin(), s.transforms.end());
      } else if (m->topic_name == "/ground_truth/pose") {
        const auto p = deserialize<geometry_msgs::msg::PoseStamped>(*m);
        truth.add(
          landfall::ros::toNanoseconds(p.header.stamp),
          landfall::toPose2(landfall::ros::toPose3(p.pose)));
      } else if (t >= start_ns && t <= end_ns && m->topic_name != "/amcl_pose") {
        messages.push_back({t, m});
      }
    }
  }
  std::sort(messages.begin(), messages.end(), [](const Recorded & a, const Recorded & b) {
    return a.stamp_ns < b.stamp_ns;
  });
  if (messages.empty() || statics.transforms.empty()) {
    std::fprintf(stderr, "relocalization_check: nothing to play in %s\n", bag.c_str());
    return 2;
  }

  auto clock_pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  auto tf_pub = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
  auto tf_static_pub = node->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(1).reliable().transient_local());
  auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  auto image_pub =
    node->create_publisher<sensor_msgs::msg::CompressedImage>("/rgbd_camera/image/compressed", 10);
  auto info_pub =
    node->create_publisher<sensor_msgs::msg::CameraInfo>("/rgbd_camera/camera_info", 10);
  auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("/imu", 50);
  std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> amcl_poses;
  std::mutex amcl_mutex;
  auto amcl_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", rclcpp::QoS(10).reliable().transient_local(),
    [&](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & p) {
      const std::lock_guard<std::mutex> lock(amcl_mutex);
      amcl_poses.push_back(*p);
    });
  auto relocalize =
    node->create_client<landfall_msgs::srv::Relocalize>("/landfall_relocalizer/relocalize");
  auto amcl_state = node->create_client<lifecycle_msgs::srv::GetState>("/amcl/get_state");
  auto reloc_state =
    node->create_client<lifecycle_msgs::srv::GetState>("/landfall_relocalizer/get_state");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::atomic<bool> spinning{true};
  std::thread spinner([&] {
    while (spinning) {
      executor.spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
  const auto publishClock = [&](int64_t ns) {
    rosgraph_msgs::msg::Clock c;
    c.clock = landfall::ros::toStamp(ns);
    clock_pub->publish(c);
  };
  const auto finish = [&](int code) {
    spinning = false;
    spinner.join();
    rclcpp::shutdown();
    return code;
  };
  const auto isActive = [](rclcpp::Client<lifecycle_msgs::srv::GetState> & client) {
    if (!client.service_is_ready()) {
      return false;
    }
    auto f = client.async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (f.future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      client.remove_pending_request(f.request_id);
      return false;
    }
    return f.future.get()->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  };

  // Lead-in: the clock runs up to the window while nav2_lifecycle_manager brings the stack up.
  tf_static_pub->publish(statics);
  const auto lead_in = Clock::now();
  bool ready = false;
  while (!ready && Clock::now() - lead_in < std::chrono::seconds(90)) {
    const int64_t elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - lead_in).count();
    publishClock(std::min(start_ns - 30'000'000'000 + elapsed, start_ns - 1'000'000'000));
    ready = isActive(*amcl_state) && isActive(*reloc_state) && relocalize->service_is_ready();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!ready) {
    std::fprintf(stderr, "relocalization_check: AMCL and the relocalizer did not become active\n");
    return finish(3);
  }
  std::printf(
    "stack active after %.1f s; playing %zu messages\n",
    std::chrono::duration<double>(Clock::now() - lead_in).count(), messages.size());

  // Playback at real-time rate. Three seconds in, ask for a pose.
  const auto play_start = Clock::now();
  std::optional<rclcpp::Client<landfall_msgs::srv::Relocalize>::FutureAndRequestId> request;
  std::optional<Result> result;
  int64_t published_stamp = 0;
  int converged = 0;
  for (const auto & r : messages) {
    const auto due = play_start + std::chrono::nanoseconds(r.stamp_ns - messages.front().stamp_ns);
    while (Clock::now() < due) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    publishClock(r.stamp_ns);
    const auto & m = *r.message;
    if (m.topic_name == "/tf") {
      auto tf = deserialize<tf2_msgs::msg::TFMessage>(m);
      tf.transforms.erase(
        std::remove_if(
          tf.transforms.begin(), tf.transforms.end(),
          [](const geometry_msgs::msg::TransformStamped & t) {
            return t.child_frame_id == "odom";
          }),
        tf.transforms.end());
      if (!tf.transforms.empty()) {
        tf_pub->publish(tf);
      }
    } else if (m.topic_name == "/scan") {
      scan_pub->publish(deserialize<sensor_msgs::msg::LaserScan>(m));
    } else if (m.topic_name == "/rgbd_camera/image/compressed") {
      image_pub->publish(deserialize<sensor_msgs::msg::CompressedImage>(m));
    } else if (m.topic_name == "/rgbd_camera/camera_info") {
      info_pub->publish(deserialize<sensor_msgs::msg::CameraInfo>(m));
    } else if (m.topic_name == "/imu") {
      imu_pub->publish(deserialize<sensor_msgs::msg::Imu>(m));
    }
    if (!request && r.stamp_ns - messages.front().stamp_ns > 3'000'000'000) {
      auto req = std::make_shared<landfall_msgs::srv::Relocalize::Request>();
      req->num_frames = 2;
      req->timeout = 20.0F;
      request = relocalize->async_send_request(req);
      std::printf("relocalize requested at %.2f s\n", static_cast<double>(r.stamp_ns) * 1e-9);
    }
    if (
      request && !result &&
      request->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      result = request->future.get()->result;
      std::printf(
        "relocalizer: status %u, published %d: %s\n", result->status, result->published ? 1 : 0,
        result->message.c_str());
      if (result->status != Result::SUCCESS || !result->published) {
        return finish(4);
      }
      published_stamp = landfall::ros::toNanoseconds(result->pose.header.stamp);
    }
    if (!result) {
      continue;
    }
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> poses;
    {
      const std::lock_guard<std::mutex> lock(amcl_mutex);
      poses.swap(amcl_poses);
    }
    for (const auto & p : poses) {
      const int64_t stamp = landfall::ros::toNanoseconds(p.header.stamp);
      if (stamp < published_stamp) {
        continue;
      }
      const auto t = truth.at(stamp, 250'000'000);
      if (!t) {
        continue;
      }
      const Pose2 expected = landfall::compose({8.0, 0.0, 0.0}, *t);
      const Pose2 got = landfall::toPose2(landfall::ros::toPose3(p.pose.pose));
      const double position = landfall::translationDistance(got, expected);
      const double yaw = landfall::rotationDistance(got, expected) * 180.0 / M_PI;
      std::printf(
        "amcl at %.2f s: %.3f m / %.2f deg from the truth, std %.3f m\n",
        static_cast<double>(stamp) * 1e-9, position, yaw, std::sqrt(p.pose.covariance[0]));
      converged = position <= 0.20 && yaw <= 3.0 ? converged + 1 : 0;
      if (converged >= 2) {
        std::printf("AMCL converged\n");
        return finish(0);
      }
    }
  }
  std::fprintf(
    stderr, "relocalization_check: %s\n",
    result ? "AMCL did not converge before the window ended" : "no answer from the relocalizer");
  return finish(result ? 5 : 6);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return check(argc, argv);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "relocalization_check: %s\n", e.what());
  } catch (...) {
    std::fprintf(stderr, "relocalization_check: unknown exception\n");
  }
  return 2;
}
