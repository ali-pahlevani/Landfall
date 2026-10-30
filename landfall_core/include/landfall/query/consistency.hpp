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

#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

/// A frame's accepted hypothesis and the odometry pose (odom -> base TF) at its image stamp.
struct FrameEstimate
{
  Pose2 map_T_base;
  Pose2 odom_T_base;
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
  Pose2 fused;  ///< mean of all estimates, propagated to the last frame's odometry pose
};

/// Moves a map pose observed at odometry pose `odom_then` to odometry pose `odom_now`:
/// map_T_base(now) = map_T_base(then) ⊕ odom_then⁻¹ ⊕ odom_now (plan §7.14).
Pose2 propagate(const Pose2 & map_T_base_then, const Pose2 & odom_then, const Pose2 & odom_now);

/// Every pair of frames must agree with the odometry between them (plan §7.10).
ConsistencyResult checkConsistency(
  const std::vector<FrameEstimate> & frames, const ConsistencyParams & params);

}  // namespace landfall

#endif  // LANDFALL_QUERY_CONSISTENCY_HPP_
