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

#ifndef LANDFALL_GEOMETRY_TWO_VIEW_HPP_
#define LANDFALL_GEOMETRY_TWO_VIEW_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

struct TwoViewParams
{
  double max_error_px = 4.0;  ///< Sampson (or, rotation-only, angular) error; COLMAP's default
  double focal_px = 500.0;    ///< converts the pixel threshold to normalized units
  int max_iterations = 10000;
  int min_iterations = 100;
  double success_prob = 0.9999;
  uint64_t seed = 42;
};

/// Geometry relating two views: x2 ~ R_21 x1 + t_21 (camera-1 coordinates into camera 2).
struct TwoViewGeometry
{
  Mat3 R_21 = Mat3::Identity();
  Vec3 t_21 = Vec3::Zero();  ///< unit direction; zero for the rotation-only model
  std::vector<int> inliers;  ///< indices into the input correspondences
};

/// Calibrated relative pose (5-point solver in LO-RANSAC with Sampson error, then refined on the
/// inliers) from undistorted normalized coordinates. Nullopt with fewer than 5 correspondences or
/// no model.
std::optional<TwoViewGeometry> estimateRelativePose(
  const std::vector<Vec2> & x1, const std::vector<Vec2> & x2, const TwoViewParams & params);

/// Pure rotation (2-point solver in LO-RANSAC on bearing vectors), for pairs without baseline,
/// where the essential matrix is degenerate.
std::optional<TwoViewGeometry> estimateRelativeRotation(
  const std::vector<Vec2> & x1, const std::vector<Vec2> & x2, const TwoViewParams & params);

}  // namespace landfall

#endif  // LANDFALL_GEOMETRY_TWO_VIEW_HPP_
