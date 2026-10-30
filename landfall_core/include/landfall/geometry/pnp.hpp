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

#ifndef LANDFALL_GEOMETRY_PNP_HPP_
#define LANDFALL_GEOMETRY_PNP_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

/// 2D-3D correspondences: undistorted normalized image coordinates (x/z, y/z) and map points.
struct Correspondences
{
  std::vector<Vec2> normalized;
  std::vector<Vec3> points;
};

struct PnpOptions
{
  double threshold_px = 6.4;  ///< inlier threshold; the caller resolves "auto" (1 % of width)
  double focal_px = 500.0;    ///< converts pixel thresholds to normalized units
  int max_iterations = 10000;
  int min_iterations = 100;
  double success_prob = 0.9999;
  uint64_t seed = 42;
  bool final_refinement = true;  ///< unconstrained 6-DoF Cauchy refinement on the inliers
};

enum class PnpMethod : uint8_t {
  kUpright,  ///< 2-point solver with known gravity (PoseLib up2p) in LO-RANSAC
  kP3P,      ///< PoseLib P3P (Lambda Twist) in LO-RANSAC
  kOpenCv,   ///< OpenCV USAC (cross-check and fallback backend)
};

/// `pnp.solver` parameter values.
enum class PnpStrategy : uint8_t {
  kUpright,
  kP3P,
  kUprightThenP3P,
  kOpenCv,
};
std::optional<PnpStrategy> parsePnpStrategy(const std::string & name);

struct PnpSolution
{
  Pose3 T_cam_map = Pose3::Identity();  ///< maps map points into the camera frame
  std::vector<int> inliers;             ///< correspondence indices
  double mean_reprojection_px = 0.0;    ///< over the inliers
  PnpMethod method = PnpMethod::kP3P;
  int iterations = 0;
};

/// Upright LO-RANSAC: the camera's gravity direction (unit, pointing down, camera frame) fixes
/// roll and pitch; map gravity is -z. Two points per sample, local optimization over yaw and
/// translation. Nullopt with fewer than 4 inliers.
std::optional<PnpSolution> solveUprightPnp(
  const Correspondences & correspondences, const Vec3 & gravity_cam, const PnpOptions & options);

/// Full 6-DoF P3P LO-RANSAC, for when the gravity prior is unreliable.
std::optional<PnpSolution> solveP3P(
  const Correspondences & correspondences, const PnpOptions & options);

/// OpenCV's USAC PnP with the same threshold semantics.
std::optional<PnpSolution> solveOpenCvPnp(
  const Correspondences & correspondences, const PnpOptions & options);

/// Runs the strategy. kUprightThenP3P falls back to P3P when the upright solution keeps fewer
/// than `min_inliers` inliers, and keeps the solution with more inliers.
std::optional<PnpSolution> solvePnp(
  PnpStrategy strategy, const Correspondences & correspondences, const Vec3 & gravity_cam,
  const PnpOptions & options, int min_inliers);

}  // namespace landfall

#endif  // LANDFALL_GEOMETRY_PNP_HPP_
