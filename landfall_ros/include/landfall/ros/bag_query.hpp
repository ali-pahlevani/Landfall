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

#ifndef LANDFALL_ROS_BAG_QUERY_HPP_
#define LANDFALL_ROS_BAG_QUERY_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <set>
#include <string>

#include "landfall/build/builder_config.hpp"
#include "landfall/query/localizer.hpp"

namespace landfall::ros
{

struct BagQueryOptions
{
  BagConfig bag;  ///< topics, frames, image time offset and gravity source of the recording
  GravityParams gravity;
  ScanCheckMode scan_mode = ScanCheckMode::kRequired;  ///< as the localizer's
  double scan_max_time_offset = 0.5;  ///< s between an image and the scan that checks it
  std::size_t every = 1;              ///< use every n-th image
  double min_frame_interval = 0.0;    ///< s between the images used
  std::set<int64_t> skip_stamps;      ///< e.g. the database's own keyframes
};

struct BagQueryCounts
{
  std::size_t images = 0;  ///< on the image topic
  std::size_t skipped_stamps = 0;
  std::size_t skipped_tf = 0;
  std::size_t skipped_gravity = 0;  ///< IMU stale, or the robot turning
  std::size_t skipped_scan = 0;     ///< no scan near the image (scan check required)
  std::size_t localized = 0;
};

/// Localizes a bag's images the way the relocalizer does live: extrinsics and odometry from the
/// bag's TF at each image stamp, gravity from its IMU (`bag.gravity_source`), the nearest LiDAR
/// scan (`bag.scan_topic`; frames without one are skipped when the check is required). Calls
/// `visit(stamp_ns, odom_T_base, result)` for every localized frame, in bag order. Throws
/// std::runtime_error when the bag lacks the topics or its camera differs from the database's.
BagQueryCounts queryBagImages(
  const std::string & uri, Localizer & localizer, const BagQueryOptions & options,
  const std::function<void(int64_t, const Pose2 &, FrameResult &&)> & visit);

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_BAG_QUERY_HPP_
