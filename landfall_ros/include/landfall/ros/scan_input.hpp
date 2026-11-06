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

#ifndef LANDFALL_ROS_SCAN_INPUT_HPP_
#define LANDFALL_ROS_SCAN_INPUT_HPP_

#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include "landfall/grid/scan_verifier.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/buffer_core.hpp"

namespace landfall::ros
{

/// The scan nearest to `stamp_ns` among time-ordered scans, if one is within `max_offset_ns`.
const sensor_msgs::msg::LaserScan * nearestScan(
  const std::deque<sensor_msgs::msg::LaserScan> & scans, int64_t stamp_ns, int64_t max_offset_ns);

/// The scan as the localizer takes it (QueryFrame::scan): its beams, with the laser pose in the base
/// frame at `image_stamp_ns`. The base's motion between the scan and the image (odometry) is folded
/// in, so the scan checks the pose the image produced (plan §7.12). Works on a live tf2_ros::Buffer
/// and on a buffer filled from a bag. Nullopt, with the reason in `error`, when TF lacks
/// base -> laser at the scan or odom -> base at either stamp.
std::optional<PlanarScan> scanAtImage(
  const sensor_msgs::msg::LaserScan & scan, const tf2::BufferCore & tf,
  const std::string & odom_frame, const std::string & base_frame, int64_t image_stamp_ns,
  std::string * error);

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_SCAN_INPUT_HPP_
