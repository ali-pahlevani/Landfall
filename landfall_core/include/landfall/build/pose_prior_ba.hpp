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

/// Builder keys `poses.odom_*`: how much the odometry between consecutive keyframes is trusted.
struct OdometryParams
{
  double std_xy = 0.02;          ///< m per step, plus ...
  double std_xy_per_m = 0.03;    ///< ... this fraction of the distance travelled, plus ...
  double std_xy_per_rad = 0.1;   ///< ... this many metres per radian turned (wheels slip)
  double std_yaw_deg = 0.5;      ///< deg per step, plus ...
  double std_yaw_per_rad = 0.1;  ///< ... this fraction of the rotation
  double max_step = 2.0;         ///< m: farther steps (a gap in the recording) aren't linked
};

/// Builder keys `ba.*` (and `poses.odom_*` for the odometry).
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
  /// Huber scale (in standard deviations) of priors given as `prior_sqrt_information`: a wrong
  /// scan match costs linearly beyond it instead of dragging the solution.
  double prior_loss_scale = 3.0;
  OdometryParams odometry;
};

struct BaKeyframe
{
  Pose2 prior;                     ///< recorded base pose
  std::optional<Mat3> covariance;  ///< recorded x, y, yaw covariance (floors only if absent)
  Pose3 T_base_cam = Pose3::Identity();
  double focal_px = 500.0;  ///< reprojection residuals are in pixels
  /// The prior's square-root information (U with U^T U = covariance^-1, e.g. from scan matching),
  /// replacing `covariance`, the floors and `prior_std_*`. Such a prior gets the robust loss, and
  /// its bounds widen to 3 standard deviations where it is uncertain (along a corridor).
  std::optional<Mat3> prior_sqrt_information;
  /// Odometry pose (odom -> base) at the keyframe: consecutive keyframes of a session that both
  /// have one are tied by their relative motion.
  std::optional<Pose2> odom;
  int session = 0;
  std::optional<Pose2> initial;  ///< where the optimization starts; default: the prior
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
  int odometry_links = 0;      ///< relative-motion constraints used
  double initial_cost = 0.0;
  double final_cost = 0.0;
  int iterations = 0;
  bool usable = false;  ///< false: Ceres failed and the inputs were returned unchanged
  std::string message;
};

/// Pose-prior bundle adjustment for a ground robot: one SE(2) base pose per keyframe (extrinsics
/// fixed) with whitened prior residuals and hard bounds, free landmarks, Cauchy-robust reprojection
/// residuals, and (where keyframes have odometry) Cauchy-robust relative-motion residuals between
/// consecutive keyframes of a session. The priors anchor the gauge, so the map frame is kept; the
/// bounds make "never move a pose far from its prior" hold by construction (3 standard deviations
/// of a `prior_sqrt_information` prior, at least the configured shift). With no landmarks it is a
/// pose graph: priors and odometry only. Single-threaded, hence deterministic.
/// One step between consecutive keyframes: the odometry's relative motion and the same motion
/// from a reference (two scan matches).
struct OdometryStep
{
  Pose2 odometry;
  Pose2 reference;
};

struct OdometryCheck
{
  int straight_steps = 0;       ///< steps turning less than 5 degrees
  double straight_std_m = 0.0;  ///< their position error per axis (the reference's noise included)
  int turns = 0;                ///< steps turning at least min_turn
  double turn_std_per_rad = 0.0;  ///< position error per axis per radian turned, beyond the above
};

/// The odometry's position error against a reference, as OdometryParams model it: on straight
/// steps, and what turning adds per radian. From medians (|e|^2 / sigma^2 of a 2-D error with
/// sigma per axis is chi-square(2), median 2 ln 2), so a few bad references don't count.
OdometryCheck checkOdometry(const std::vector<OdometryStep> & steps, double min_turn_rad);

/// Whether the turns call for a larger odometry turn std than `std_xy_per_rad`
/// (poses.odom_std_xy_per_rad): from `min_turns` turns on, when the measured error is more than a
/// quarter above it (about the measurement's own spread from a few tens of turns), so that a setting
/// at the measured value stays quiet.
bool turnsNeedMore(const OdometryCheck & check, double std_xy_per_rad, int min_turns);

BaResult adjustPosePriors(
  const std::vector<BaKeyframe> & keyframes, const std::vector<Vec3> & landmarks,
  const std::vector<BaObservation> & observations, const PosePriorBaParams & params);

/// T_map_cam of a base pose and its camera extrinsics.
Pose3 cameraPose(const Pose2 & map_base, const Pose3 & T_base_cam);

}  // namespace landfall

#endif  // LANDFALL_BUILD_POSE_PRIOR_BA_HPP_
