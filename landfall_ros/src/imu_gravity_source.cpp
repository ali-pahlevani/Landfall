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

#include "landfall/ros/imu_gravity_source.hpp"

#include <cstdlib>
#include <iterator>

#include "landfall/ros/conversions.hpp"

namespace landfall::ros
{

bool ImuGravitySource::add(const sensor_msgs::msg::Imu & imu)
{
  // REP-145: orientation_covariance[0] == -1 means "no orientation estimate".
  if (imu.orientation_covariance[0] == -1.0) {
    return false;
  }
  const Eigen::Quaterniond q(
    imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z);
  if (q.norm() < 0.5) {
    return false;
  }
  ImuSample sample;
  sample.orientation = q.normalized();
  sample.angular_velocity =
    Vec3(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
  samples_[toNanoseconds(imu.header.stamp)] = sample;
  while (samples_.size() > history_) {
    samples_.erase(samples_.begin());
  }
  return true;
}

std::optional<GravityEstimate> ImuGravitySource::at(
  int64_t stamp_ns, const Eigen::Quaterniond & q_base_imu, GravitySource source,
  const GravityParams & params) const
{
  std::optional<ImuSample> nearest;
  if (!samples_.empty()) {
    auto after = samples_.lower_bound(stamp_ns);
    auto best = after;
    if (
      after == samples_.end() ||
      (after != samples_.begin() &&
       std::llabs(std::prev(after)->first - stamp_ns) < std::llabs(after->first - stamp_ns))) {
      best = std::prev(after);
    }
    nearest = best->second;
    nearest->age = static_cast<double>(std::llabs(best->first - stamp_ns)) * 1e-9;
  }
  return estimateGravity(source, nearest, q_base_imu, params);
}

}  // namespace landfall::ros
