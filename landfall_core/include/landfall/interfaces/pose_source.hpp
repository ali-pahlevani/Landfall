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

#ifndef LANDFALL_INTERFACES_POSE_SOURCE_HPP_
#define LANDFALL_INTERFACES_POSE_SOURCE_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include "landfall/types.hpp"

namespace landfall
{

/// Where the recorder's keyframe poses come from, at one image stamp.
struct PoseSample
{
  Pose2 map_base;                        ///< base pose in the global frame
  Pose3 T_base_cam = Pose3::Identity();  ///< camera optical frame in the base frame
  std::optional<Mat3> covariance;        ///< x, y, yaw of the localization estimate, if known
  double covariance_age = 0.0;           ///< s from the covariance estimate to the image
  double linear_speed = 0.0;             ///< m/s of the base (odometry), just before the image
  double angular_speed = 0.0;            ///< rad/s
};

/// Pose of the robot and its camera at a given time: live TF, TF replayed from a bag, and later
/// other sources (e.g. visual-inertial odometry).
class IPoseSource
{
public:
  virtual ~IPoseSource() = default;
  /// `stamp_ns` is on the clock of the image stamps. Returns nullopt, with the reason in `error`,
  /// when the pose isn't available (yet).
  virtual std::optional<PoseSample> poseAt(
    int64_t stamp_ns, const std::string & camera_frame, std::string * error) = 0;
};

}  // namespace landfall

#endif  // LANDFALL_INTERFACES_POSE_SOURCE_HPP_
