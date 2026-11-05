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

// The depot fixture (test/fixtures/depot_small) for node tests: its messages in memory, its map
// database, its occupancy map as map_server publishes it.

#ifndef LANDFALL_ROS_TEST_FIXTURE_DATA_HPP_
#define LANDFALL_ROS_TEST_FIXTURE_DATA_HPP_

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "landfall/build/map_builder.hpp"
#include "landfall/eval/trajectory.hpp"
#include "landfall/grid/map_loader.hpp"
#include "landfall/ros/bag_reader.hpp"
#include "landfall/ros/conversions.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace landfall::test
{

const std::string kFixtureDir = LANDFALL_FIXTURE_DIR;
/// The fixture's map frame is Gazebo's world shifted by this (nav2_bringup's depot).
const Pose2 kWorldToMap{8.0, 0.0, 0.0};

class TempDir
{
public:
  TempDir()
  {
    std::string pattern =
      (std::filesystem::temp_directory_path() / "landfall_ros_test_XXXXXX").string();
    if (mkdtemp(pattern.data()) == nullptr) {
      throw std::runtime_error("mkdtemp failed");
    }
    path_ = pattern;
  }
  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;
  ~TempDir()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }
  std::string file(const std::string & name) const { return (path_ / name).string(); }

private:
  std::filesystem::path path_;
};

struct FixtureMessages
{
  std::vector<tf2_msgs::msg::TFMessage> tf;  ///< in bag order
  tf2_msgs::msg::TFMessage tf_static;
  sensor_msgs::msg::CameraInfo camera_info;
  std::vector<sensor_msgs::msg::CompressedImage> images;
  eval::Trajectory truth;  ///< Gazebo's world frame
};

template <typename MessageT>
MessageT deserializeBagMessage(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  static const rclcpp::Serialization<MessageT> serialization;
  const rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  MessageT message;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

inline FixtureMessages readFixtureMessages()
{
  FixtureMessages m;
  rosbag2_cpp::Reader reader;
  reader.open(kFixtureDir + "/bag");
  while (reader.has_next()) {
    const auto message = reader.read_next();
    const std::string & topic = message->topic_name;
    if (topic == "/tf") {
      m.tf.push_back(deserializeBagMessage<tf2_msgs::msg::TFMessage>(*message));
    } else if (topic == "/tf_static") {
      const auto statics = deserializeBagMessage<tf2_msgs::msg::TFMessage>(*message);
      m.tf_static.transforms.insert(
        m.tf_static.transforms.end(), statics.transforms.begin(), statics.transforms.end());
    } else if (topic == "/rgbd_camera/camera_info") {
      m.camera_info = deserializeBagMessage<sensor_msgs::msg::CameraInfo>(*message);
    } else if (topic == "/rgbd_camera/image/compressed") {
      m.images.push_back(deserializeBagMessage<sensor_msgs::msg::CompressedImage>(*message));
    } else if (topic == "/ground_truth/pose") {
      const auto pose = deserializeBagMessage<geometry_msgs::msg::PoseStamped>(*message);
      m.truth.add(ros::toNanoseconds(pose.header.stamp), toPose2(ros::toPose3(pose.pose)));
    }
  }
  return m;
}

inline std::string readTextFile(const std::string & path)
{
  std::ifstream file(path);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

inline MapServerSemantics distroSemantics()
{
  return std::string(LANDFALL_ROS_DISTRO) == "humble" ? MapServerSemantics::kHumble
                                                      : MapServerSemantics::kJazzy;
}

/// Builds the fixture's map database (as `landfall build --config build.yaml` does).
inline BuildStats buildFixtureDatabase(const std::string & db_path, int threads = 4)
{
  auto config = parseBuildConfig(readTextFile(kFixtureDir + "/build.yaml"));
  config.builder.threads = threads;
  const auto bag = ros::readBagKeyframes(kFixtureDir + "/bag", config);
  BuildInput input;
  input.map = loadMap(kFixtureDir + "/depot.yaml", distroSemantics());
  input.sessions.push_back(bag.session);
  input.cameras.push_back(bag.camera);
  input.keyframes = bag.keyframes;
  return buildMap(input, config, db_path);
}

/// The occupancy grid map_server publishes for the fixture's depot map.
inline nav_msgs::msg::OccupancyGrid fixtureMapMessage()
{
  const auto map = loadMap(kFixtureDir + "/depot.yaml", distroSemantics());
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = "map";
  msg.info.width = static_cast<uint32_t>(map.grid.width);
  msg.info.height = static_cast<uint32_t>(map.grid.height);
  msg.info.resolution = static_cast<float>(map.grid.resolution);
  msg.info.origin = ros::toPoseMsg(toPose3(map.grid.origin));
  msg.data.assign(map.grid.data.begin(), map.grid.data.end());
  return msg;
}

}  // namespace landfall::test

#endif  // LANDFALL_ROS_TEST_FIXTURE_DATA_HPP_
