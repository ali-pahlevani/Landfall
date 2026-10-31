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

#ifndef LANDFALL_BUILD_POSE_PRIOR_BA_HPP_
#define LANDFALL_BUILD_POSE_PRIOR_BA_HPP_

#include <optional>
#include <string>
#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

/// Builder keys `ba.*`.
struct PosePriorBaParams
{
  bool enabled = true;
  double max_shift_xy = 0.10;      ///< m; hard bound on |dx| and |dy| from the prior
  double max_shift_yaw_deg = 2.0;  ///< hard bound on |dyaw|
  double loss_scale_px = 2.0;      ///< Cauchy scale of the reprojection residuals
  double prior_floor_xy = 0.02;    ///< m added (in quadrature) to the recorded std of x and y
  double prior_floor_yaw_deg = 0.5;
  /// When > 0: the std of every prior's x/y (m) and yaw (deg), replacing the recorded covariance
  /// and the floors (for localization stacks that misreport their covariance).
  double prior_std_xy = 0.0;
  double prior_std_yaw_deg = 0.0;
  int max_iterations = 50;
};

struct BaKeyframe
{
  Pose2 prior;                     ///< recorded base pose
  std::optional<Mat3> covariance;  ///< recorded x, y, yaw covariance (floors only if absent)
  Pose3 T_base_cam = Pose3::Identity();
  double focal_px = 500.0;  ///< reprojection residuals are in pixels
};

struct BaObservation
{
  int keyframe = 0;
  int landmark = 0;
  Vec2 normalized = Vec2::Zero();  ///< undistorted normalized image coordinates
};

struct BaResult
{
  std::vector<Pose2> poses;  ///< refined base poses
  std::vector<Vec3> landmarks;
  std::vector<char> at_bound;  ///< the keyframe's shift reached a hard bound
  double initial_cost = 0.0;
  double final_cost = 0.0;
  int iterations = 0;
  bool usable = false;  ///< false: Ceres failed and the inputs were returned unchanged
  std::string message;
};

/// Pose-prior bundle adjustment for a ground robot: one SE(2) base pose per keyframe (extrinsics
/// fixed) with whitened prior residuals and hard bounds, free landmarks, Cauchy-robust reprojection
/// residuals. The priors anchor the gauge, so the map frame is kept; the bounds make "never move a
/// pose far from its prior" hold by construction. Single-threaded, hence deterministic.
BaResult adjustPosePriors(
  const std::vector<BaKeyframe> & keyframes, const std::vector<Vec3> & landmarks,
  const std::vector<BaObservation> & observations, const PosePriorBaParams & params);

/// T_map_cam of a base pose and its camera extrinsics.
Pose3 cameraPose(const Pose2 & map_base, const Pose3 & T_base_cam);

}  // namespace landfall

#endif  // LANDFALL_BUILD_POSE_PRIOR_BA_HPP_
