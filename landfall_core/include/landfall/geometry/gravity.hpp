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

#ifndef LANDFALL_GEOMETRY_GRAVITY_HPP_
#define LANDFALL_GEOMETRY_GRAVITY_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "landfall/types.hpp"

namespace landfall
{

/// Gravity direction (unit vector, pointing down) in the base frame on a flat floor.
inline Vec3 flatFloorGravity() { return -Vec3::UnitZ(); }

/// Smallest rotation that maps the direction `from` onto `to` (both non-zero).
Mat3 rotationBetween(const Vec3 & from, const Vec3 & to);

/// Rotation that levels the base frame (gravity becomes -z) without changing its heading.
inline Mat3 levelingRotation(const Vec3 & gravity_base)
{
  return rotationBetween(gravity_base, -Vec3::UnitZ());
}

/// Gravity in the base frame from an IMU attitude `q_world_imu` (any gravity-aligned world frame
/// with z up, e.g. ENU) and the static base->IMU rotation `q_base_imu`. Only roll and pitch
/// matter, so the IMU's yaw (and a magnetometer) is irrelevant.
Vec3 gravityFromImu(const Eigen::Quaterniond & q_world_imu, const Eigen::Quaterniond & q_base_imu);

/// Angle between gravity and the base's -z axis, degrees.
double tiltDeg(const Vec3 & gravity_base);

enum class GravitySource : uint8_t {
  kAuto,       ///< IMU when a fresh sample exists, flat floor otherwise
  kImu,        ///< IMU only
  kFlatFloor,  ///< ignore the IMU
};

/// Parses the `gravity.source` parameter ("auto", "imu", "flat_floor").
std::optional<GravitySource> parseGravitySource(const std::string & name);

struct GravityParams
{
  double max_age = 0.1;           ///< s; older IMU samples are stale
  double max_angular_rate = 0.1;  ///< rad/s; orientation filters are only trusted while still
  double ramp_tilt_deg = 1.5;     ///< tilt above this means "on a ramp"
};

struct ImuSample
{
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  ///< q_world_imu
  Vec3 angular_velocity = Vec3::Zero();                             ///< rad/s, IMU frame
  double age = 0.0;  ///< s, relative to the query image
};

struct GravityEstimate
{
  Vec3 gravity_base = flatFloorGravity();
  bool from_imu = false;
  double tilt_deg = 0.0;
  bool on_ramp = false;
};

/// Picks the gravity for one query frame. Returns nullopt when the frame must wait: the IMU is
/// required but stale, or it is fresh but the robot is turning or shaking (in kAuto too, since a
/// flat-floor guess would be wrong on a ramp).
std::optional<GravityEstimate> estimateGravity(
  GravitySource source, const std::optional<ImuSample> & imu, const Eigen::Quaterniond & q_base_imu,
  const GravityParams & params);

}  // namespace landfall

#endif  // LANDFALL_GEOMETRY_GRAVITY_HPP_
