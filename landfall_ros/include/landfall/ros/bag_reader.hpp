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

#ifndef LANDFALL_ROS_BAG_READER_HPP_
#define LANDFALL_ROS_BAG_READER_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "landfall/build/map_builder.hpp"
#include "landfall/ros/imu_gravity_source.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2/buffer_core.hpp"

namespace landfall::ros
{

/// Everything of a bag except its images: TF (static and dynamic) in a buffer spanning the whole
/// recording, the first camera_info, localization covariances and IMU attitudes.
struct BagSideData
{
  std::shared_ptr<tf2::BufferCore> buffer;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info;
  std::vector<std::pair<int64_t, Mat3>> covariances;  ///< (stamp, x/y/yaw covariance)
  ImuGravitySource imu{std::numeric_limits<std::size_t>::max()};
  std::string imu_frame;               ///< `imu_frame` if given, else the samples' frame_id
  std::size_t invalid_transforms = 0;  ///< TF entries tf2 refused (skipped)
};

/// Reads the side data. Empty topic names are skipped. Throws std::runtime_error for a missing or
/// unreadable bag.
BagSideData readBagSideData(
  const std::string & uri, const std::string & camera_info_topic, const std::string & pose_topic,
  const std::string & imu_topic, const std::string & imu_frame);

/// Number of messages on an Image or CompressedImage topic. Throws std::runtime_error, listing the
/// bag's image topics, if the topic is missing or of another type.
std::size_t countBagImages(const std::string & uri, const std::string & image_topic);

/// Calls `visit(stamp_ns + offset, decode)` for every image on the topic, in bag order, until it
/// returns false.
void forEachBagImage(
  const std::string & uri, const std::string & image_topic, double time_offset,
  const std::function<bool(int64_t, std::function<cv::Mat()>)> & visit);

/// Keyframes selected from one rosbag2 recording.
struct BagKeyframes
{
  BuildSession session;
  BuildCamera camera;
  std::vector<BuildKeyframe> keyframes;  ///< session 0, camera 0 (the caller re-indexes)
  GateCounts counts;
  std::size_t images = 0;
  std::size_t invalid_transforms = 0;  ///< TF entries tf2 refused (skipped)
};

/// (images done, images in the bag)
using BagProgress = std::function<void(std::size_t, std::size_t)>;

/// Replays a bag through the recorder's keyframe gates (`config.bag`, `config.gating`): first all
/// TF, camera_info, localization covariance and IMU messages, then every image. Any storage
/// plugin rosbag2 has (sqlite3, mcap). Throws std::runtime_error with the reason (missing topic,
/// no camera_info, unreadable bag).
BagKeyframes readBagKeyframes(
  const std::string & uri, const BuildConfigFile & config, const BagProgress & progress = {});

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_BAG_READER_HPP_
