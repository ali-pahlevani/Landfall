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
#include <functional>
#include <string>
#include <vector>

#include "landfall/build/map_builder.hpp"

namespace landfall::ros
{

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
