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

#ifndef LANDFALL_QUERY_SANITY_HPP_
#define LANDFALL_QUERY_SANITY_HPP_

#include <string>
#include <vector>

#include "landfall/features/bucketing.hpp"
#include "landfall/geometry/gravity.hpp"
#include "landfall/grid/likelihood_field.hpp"

namespace landfall
{

struct SanityParams
{
  int min_inliers = 40;
  double min_inlier_ratio = 0.25;
  double max_mean_reprojection_px = 3.2;  ///< caller resolves "auto" (half the PnP threshold)
  double max_height_error = 0.10;         ///< m, base height above the floor (flat floor)
  double max_floor_elevation = 0.0;       ///< m, extra height allowed on a ramp; 0 = refuse ramps
  double max_tilt_error_deg = 3.0;        ///< vision roll/pitch vs the gravity estimate
  double min_inlier_coverage = 0.15;      ///< fraction of grid cells with an inlier
  bool require_free_space = true;         ///< base must stand on a known-free map cell
  GridSize coverage_grid;
};

/// One solved hypothesis, in the terms the checks need.
struct SanityInput
{
  Pose3 T_map_cam = Pose3::Identity();   ///< from PnP
  Pose3 T_base_cam = Pose3::Identity();  ///< extrinsics at the image stamp
  GravityEstimate gravity;               ///< gravity in the base frame and ramp state
  int num_correspondences = 0;
  std::vector<Vec2> inlier_pixels;
  double mean_reprojection_px = 0.0;
  cv::Size image_size;
};

struct SanityResult
{
  bool passed = false;
  std::string reason;  ///< first failed check; empty when passed
  Pose3 T_map_base = Pose3::Identity();
  Pose2 pose;  ///< planar projection of T_map_base
  int num_inliers = 0;
  double inlier_ratio = 0.0;
  double mean_reprojection_px = 0.0;
  double height_error_m = 0.0;
  double tilt_error_deg = 0.0;
  double inlier_coverage = 0.0;
};

/// Rejects geometrically implausible poses (plan §7.9). Every metric is computed even when an
/// earlier check fails, so diagnostics show all of them. `field` may be null only when
/// require_free_space is false.
SanityResult checkSanity(
  const SanityInput & input, const SanityParams & params, const LikelihoodField * field);

}  // namespace landfall

#endif  // LANDFALL_QUERY_SANITY_HPP_
