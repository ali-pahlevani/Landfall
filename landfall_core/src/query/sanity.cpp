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

#include "landfall/query/sanity.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace landfall
{

SanityResult checkSanity(
  const SanityInput & input, const SanityParams & params, const LikelihoodField * field)
{
  if (params.require_free_space && field == nullptr) {
    throw std::invalid_argument("the free-space check needs the map");
  }
  SanityResult r;
  r.T_map_base = input.T_map_cam * input.T_base_cam.inverse();
  r.pose = toPose2(r.T_map_base);
  r.num_inliers = static_cast<int>(input.inlier_pixels.size());
  r.inlier_ratio = input.num_correspondences > 0
                     ? static_cast<double>(r.num_inliers) / input.num_correspondences
                     : 0.0;
  r.mean_reprojection_px = input.mean_reprojection_px;
  r.inlier_coverage = gridCoverage(input.inlier_pixels, input.image_size, params.coverage_grid);
  // The base frame lies on the floor: its height is the error.
  r.height_error_m = std::abs(r.T_map_base.translation().z());
  // Gravity as vision sees it in the base frame, vs the IMU (or the flat-floor assumption).
  const Vec3 vision_gravity = r.T_map_base.linear().transpose() * Vec3(0.0, 0.0, -1.0);
  const double c =
    std::clamp(vision_gravity.dot(input.gravity.gravity_base.normalized()), -1.0, 1.0);
  r.tilt_error_deg = std::acos(c) * 180.0 / M_PI;

  std::ostringstream reason;
  const double allowed_height = input.gravity.on_ramp
                                  ? params.max_floor_elevation + params.max_height_error
                                  : params.max_height_error;
  if (r.num_inliers < params.min_inliers) {
    reason << r.num_inliers << " inliers < " << params.min_inliers;
  } else if (r.inlier_ratio < params.min_inlier_ratio) {
    reason << "inlier ratio " << r.inlier_ratio << " < " << params.min_inlier_ratio;
  } else if (r.mean_reprojection_px > params.max_mean_reprojection_px) {
    reason << "mean reprojection " << r.mean_reprojection_px << " px > "
           << params.max_mean_reprojection_px;
  } else if (r.inlier_coverage < params.min_inlier_coverage) {
    reason << "inlier coverage " << r.inlier_coverage << " < " << params.min_inlier_coverage;
  } else if (input.gravity.on_ramp && params.max_floor_elevation <= 0.0) {
    reason << "robot is on a ramp (tilt " << input.gravity.tilt_deg
           << " deg); set sanity.max_floor_elevation to allow ramp queries";
  } else if (r.height_error_m > allowed_height) {
    reason << "base height " << r.height_error_m << " m > " << allowed_height;
  } else if (r.tilt_error_deg > params.max_tilt_error_deg) {
    reason << "tilt error " << r.tilt_error_deg << " deg > " << params.max_tilt_error_deg;
  } else if (
    params.require_free_space &&
    field->classify(r.pose.x, r.pose.y) != LikelihoodField::Cell::kFree) {
    reason << "base pose (" << r.pose.x << ", " << r.pose.y << ") is not on a known-free cell";
  }
  r.reason = reason.str();
  r.passed = r.reason.empty();
  return r;
}

}  // namespace landfall
