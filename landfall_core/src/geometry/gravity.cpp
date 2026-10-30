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

#include "landfall/geometry/gravity.hpp"

#include <algorithm>
#include <cmath>

namespace landfall
{

Mat3 rotationBetween(const Vec3 & from, const Vec3 & to)
{
  const Vec3 a = from.normalized();
  const Vec3 b = to.normalized();
  const Vec3 axis = a.cross(b);
  const double sin_angle = axis.norm();
  const double cos_angle = a.dot(b);
  if (sin_angle < 1e-12) {
    if (cos_angle > 0.0) {
      return Mat3::Identity();
    }
    // Opposite directions: half turn about any axis perpendicular to `a`.
    const Vec3 helper = std::abs(a.x()) < 0.9 ? Vec3::UnitX() : Vec3::UnitY();
    return Eigen::AngleAxisd(M_PI, a.cross(helper).normalized()).toRotationMatrix();
  }
  return Eigen::AngleAxisd(std::atan2(sin_angle, cos_angle), axis / sin_angle).toRotationMatrix();
}

Vec3 gravityFromImu(const Eigen::Quaterniond & q_world_imu, const Eigen::Quaterniond & q_base_imu)
{
  const Vec3 g_imu = q_world_imu.normalized().conjugate() * (-Vec3::UnitZ());
  return (q_base_imu.normalized() * g_imu).normalized();
}

double tiltDeg(const Vec3 & gravity_base)
{
  const double c = std::clamp(gravity_base.normalized().dot(-Vec3::UnitZ()), -1.0, 1.0);
  return std::acos(c) * 180.0 / M_PI;
}

std::optional<GravitySource> parseGravitySource(const std::string & name)
{
  if (name == "auto") {
    return GravitySource::kAuto;
  }
  if (name == "imu") {
    return GravitySource::kImu;
  }
  if (name == "flat_floor") {
    return GravitySource::kFlatFloor;
  }
  return std::nullopt;
}

std::optional<GravityEstimate> estimateGravity(
  GravitySource source, const std::optional<ImuSample> & imu, const Eigen::Quaterniond & q_base_imu,
  const GravityParams & params)
{
  GravityEstimate estimate;
  if (source == GravitySource::kFlatFloor) {
    return estimate;
  }
  const bool fresh = imu.has_value() && imu->age >= 0.0 && imu->age <= params.max_age;
  if (!fresh) {
    if (source == GravitySource::kImu) {
      return std::nullopt;
    }
    return estimate;
  }
  if (imu->angular_velocity.norm() > params.max_angular_rate) {
    return std::nullopt;
  }
  estimate.gravity_base = gravityFromImu(imu->orientation, q_base_imu);
  estimate.from_imu = true;
  estimate.tilt_deg = tiltDeg(estimate.gravity_base);
  estimate.on_ramp = estimate.tilt_deg > params.ramp_tilt_deg;
  return estimate;
}

}  // namespace landfall
