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

#ifndef LANDFALL_ROS_IMU_GRAVITY_SOURCE_HPP_
#define LANDFALL_ROS_IMU_GRAVITY_SOURCE_HPP_

#include <cstdint>
#include <map>
#include <optional>

#include "landfall/geometry/gravity.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace landfall::ros
{

/// Recent IMU attitudes, answering "which way is down" at an image stamp (core:
/// estimateGravity). Only roll and pitch of the orientation are used.
class ImuGravitySource
{
public:
  explicit ImuGravitySource(std::size_t history = 2000) : history_(history) {}

  /// Stores a sample; messages without orientation (covariance[0] == -1) are ignored. Returns
  /// whether it was stored.
  bool add(const sensor_msgs::msg::Imu & imu);

  /// Gravity at the stamp from the nearest sample (its age relative to the stamp decides
  /// freshness). Nullopt when the frame must wait (see estimateGravity).
  std::optional<GravityEstimate> at(
    int64_t stamp_ns, const Eigen::Quaterniond & q_base_imu, GravitySource source,
    const GravityParams & params) const;

  bool empty() const { return samples_.empty(); }

private:
  std::size_t history_;
  std::map<int64_t, ImuSample> samples_;
};

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_IMU_GRAVITY_SOURCE_HPP_
