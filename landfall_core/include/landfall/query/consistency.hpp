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

#ifndef LANDFALL_QUERY_CONSISTENCY_HPP_
#define LANDFALL_QUERY_CONSISTENCY_HPP_

#include <optional>
#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

/// A frame's accepted hypothesis and the odometry pose (odom -> base TF) at its image stamp.
struct FrameEstimate
{
  Pose2 map_T_base;
  Pose2 odom_T_base;
  /// Of map_T_base (x, y, yaw). Without it in every frame the fused pose is the plain mean and
  /// has none.
  std::optional<Mat3> covariance;
};

struct ConsistencyParams
{
  double max_translation_error = 0.15;  ///< m
  double max_rotation_error_deg = 5.0;
};

struct ConsistencyResult
{
  bool consistent = false;
  double max_translation_error = 0.0;
  double max_rotation_error_deg = 0.0;
  /// The estimates propagated to the last frame's odometry pose and fused: weighted by their
  /// covariances (covariance intersection, plan §7.10), or their mean without covariances.
  Pose2 fused;
  /// Of `fused`: consistent however much the frames' errors are shared (the same landmarks);
  /// frames alike give one frame's covariance.
  std::optional<Mat3> covariance;
};

/// Moves a map pose observed at odometry pose `odom_then` to odometry pose `odom_now`:
/// map_T_base(now) = map_T_base(then) ⊕ odom_then⁻¹ ⊕ odom_now (plan §7.14).
Pose2 propagate(const Pose2 & map_T_base_then, const Pose2 & odom_then, const Pose2 & odom_now);

/// The covariance of map_T_base(then) carried to map_T_base(now) by propagate() (the odometry
/// itself taken as exact over the short time of a request). Exactly symmetric.
Mat3 propagateCovariance(
  const Mat3 & covariance, const Pose2 & map_T_base_then, const Pose2 & odom_then,
  const Pose2 & odom_now);

/// A covariance without negative entries that covers `covariance` (their difference is positive
/// semidefinite, so it is never narrower), as Nav2's AMCL requires of an initial pose from Nav2
/// 1.5 (Lyrical) on: it rejects any negative entry, correlations included. Each negatively
/// correlated pair loses its correlation and both variances grow by 1 + |rho|, the smallest
/// axis-aligned ellipse around the pair's. Unchanged without negative entries; exactly symmetric.
Mat3 coverWithoutNegativeEntries(const Mat3 & covariance);

/// Every pair of frames must agree with the odometry between them (plan §7.10).
ConsistencyResult checkConsistency(
  const std::vector<FrameEstimate> & frames, const ConsistencyParams & params);

}  // namespace landfall

#endif  // LANDFALL_QUERY_CONSISTENCY_HPP_
