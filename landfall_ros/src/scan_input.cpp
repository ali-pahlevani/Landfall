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

#include "landfall/ros/scan_input.hpp"

#include <chrono>
#include <cstdlib>

#include "landfall/ros/conversions.hpp"
#include "tf2/exceptions.hpp"

namespace landfall::ros
{

const sensor_msgs::msg::LaserScan * nearestScan(
  const std::deque<sensor_msgs::msg::LaserScan> & scans, int64_t stamp_ns, int64_t max_offset_ns)
{
  const sensor_msgs::msg::LaserScan * best = nullptr;
  int64_t best_offset = max_offset_ns;
  for (const auto & scan : scans) {
    const int64_t offset = std::llabs(toNanoseconds(scan.header.stamp) - stamp_ns);
    if (offset <= best_offset) {
      best = &scan;
      best_offset = offset;
    }
  }
  return best;
}

std::optional<PlanarScan> scanAtImage(
  const sensor_msgs::msg::LaserScan & scan, const tf2::BufferCore & tf,
  const std::string & odom_frame, const std::string & base_frame, int64_t image_stamp_ns,
  std::string * error)
{
  const tf2::TimePoint at_scan{std::chrono::nanoseconds(toNanoseconds(scan.header.stamp))};
  const tf2::TimePoint at_image{std::chrono::nanoseconds(image_stamp_ns)};
  PlanarScan planar;
  try {
    const Pose3 T_base_laser =
      toPose3(tf.lookupTransform(base_frame, scan.header.frame_id, at_scan).transform);
    const Pose3 odom_T_base_scan =
      toPose3(tf.lookupTransform(odom_frame, base_frame, at_scan).transform);
    const Pose3 odom_T_base_image =
      toPose3(tf.lookupTransform(odom_frame, base_frame, at_image).transform);
    planar.T_base_laser = odom_T_base_image.inverse() * odom_T_base_scan * T_base_laser;
  } catch (const tf2::TransformException & e) {
    if (error != nullptr) {
      *error = e.what();
    }
    return std::nullopt;
  }
  planar.ranges = scan.ranges;
  planar.angle_min = scan.angle_min;
  planar.angle_increment = scan.angle_increment;
  planar.range_min = scan.range_min;
  planar.range_max = scan.range_max;
  return planar;
}

}  // namespace landfall::ros
