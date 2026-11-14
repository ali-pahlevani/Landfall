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

#ifndef LANDFALL_GEOMETRY_FLOOR_REFINEMENT_HPP_
#define LANDFALL_GEOMETRY_FLOOR_REFINEMENT_HPP_

#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

/// What a ground robot knows of its camera's pose besides the image (plan §7.8): its base stands
/// on the floor, tilted as gravity says, each to within a standard deviation.
struct FloorPrior
{
  Pose3 T_base_cam = Pose3::Identity();  ///< extrinsics at the image stamp
  /// Down in the base frame: the IMU's when it has one, else the flat floor's (0, 0, -1).
  Vec3 gravity_base = Vec3(0.0, 0.0, -1.0);
  double height = 0.0;      ///< of the base above the map's floor (z = 0)
  double height_std = 0.0;  ///< m; 0: the height is free
  double tilt_std = 0.0;    ///< rad; 0: the tilt is free
};

struct FloorRefinementOptions
{
  double focal_px = 500.0;    ///< pixels per normalized image unit
  double threshold_px = 6.4;  ///< scale of the Cauchy loss, and the inlier threshold
  /// Lower bound of the reprojection noise estimated from the residuals: a few correspondences
  /// that happen to fit exactly don't make the pose exact.
  double min_noise_px = 0.5;
  int max_iterations = 20;  ///< 0: the covariance at the given pose, which isn't moved
};

struct FloorRefinement
{
  bool valid = false;  ///< false: too few correspondences, or their geometry fixes no pose
  Pose3 T_map_base = Pose3::Identity();
  Pose2 pose;  ///< planar projection of T_map_base
  /// Of x, y and yaw (m², m·rad, rad²), from the reprojection noise, the correspondences'
  /// geometry and the prior: long along the view when the landmarks are far. The map's own error
  /// isn't in it.
  Mat3 covariance = Mat3::Zero();
  double noise_px = 0.0;  ///< reprojection noise, estimated from the residuals at the pose
  int inliers = 0;        ///< correspondences within the threshold at the pose (all of them)
  double mean_reprojection_px = 0.0;  ///< over the refined ones
  int iterations = 0;
};

/// Refines a base pose on the correspondences `indices` (normalized image coordinates and map
/// points) by robust least squares (Cauchy loss), the prior's height and tilt as soft constraints,
/// starting from `T_map_base`. Returns the pose and the covariance of its x, y and yaw.
FloorRefinement refineOnFloor(
  const std::vector<Vec2> & normalized, const std::vector<Vec3> & points,
  const std::vector<int> & indices, const Pose3 & T_map_base, const FloorPrior & prior,
  const FloorRefinementOptions & options);

}  // namespace landfall

#endif  // LANDFALL_GEOMETRY_FLOOR_REFINEMENT_HPP_
