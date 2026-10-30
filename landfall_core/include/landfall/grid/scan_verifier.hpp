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

#ifndef LANDFALL_GRID_SCAN_VERIFIER_HPP_
#define LANDFALL_GRID_SCAN_VERIFIER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "landfall/grid/likelihood_field.hpp"

namespace landfall
{

struct ScanCheckParams
{
  double max_range = 10.0;            ///< m; longer readings are ignored
  double hit_tolerance = 0.10;        ///< m; endpoint this close to an obstacle is a hit
  double min_hit_ratio = 0.60;        ///< hits / beams ending in known space
  double max_violation_ratio = 0.05;  ///< beams passing through an obstacle / valid beams
  double min_known_ratio = 0.5;       ///< beams ending in known space / valid beams
  int min_valid_beams = 60;
  double violation_margin = 0.15;  ///< m; rays stop this short of their endpoint
  double search_radius = 0.10;     ///< m; local search window (0 disables the search)
  double search_step = 0.025;
  double search_angle_deg = 3.0;
  double search_angle_step_deg = 1.0;
  double floor_clearance = 0.05;  ///< m; leveled endpoints lower than this are floor hits
};

/// A sensor_msgs/LaserScan with the laser pose in the base frame.
struct PlanarScan
{
  std::vector<float> ranges;
  double angle_min = 0.0;
  double angle_increment = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  Pose3 T_base_laser = Pose3::Identity();
};

/// A scan reduced to 2D in the gravity-leveled base frame (x forward, y left).
struct ProjectedScan
{
  Vec2 origin = Vec2::Zero();  ///< laser position
  std::vector<Vec2> endpoints;
  int dropped_floor = 0;  ///< beams that hit the floor or a ramp
};

struct ScanScore
{
  int valid_beams = 0;
  int known_beams = 0;
  int hits = 0;
  int violations = 0;
  double known_ratio = 0.0;
  double hit_ratio = 0.0;
  double violation_ratio = 0.0;
};

struct ScanCheckResult
{
  bool passed = false;
  std::string reason;  ///< why it failed; empty on success
  ScanScore score;
  Pose2 best_pose;  ///< pose of the reported score (refined within the search window)
};

/// "Vision proposes, LiDAR verifies": scores a base pose hypothesis against the map with a scan.
class ScanVerifier
{
public:
  ScanVerifier(std::shared_ptr<const LikelihoodField> field, const ScanCheckParams & params);

  /// Projects valid beams with the gravity direction in the base frame ((0, 0, -1) on a flat
  /// floor; from the IMU on a ramp) and drops the ones that hit the floor.
  ProjectedScan project(const PlanarScan & scan, const Vec3 & gravity_base) const;
  ScanScore score(const ProjectedScan & scan, const Pose2 & map_T_base, bool with_violations) const;
  /// Searches the local window around the hypothesis and accepts the best pose that passes.
  ScanCheckResult verify(const ProjectedScan & scan, const Pose2 & hypothesis) const;

private:
  std::string failure(const ScanScore & score) const;

  std::shared_ptr<const LikelihoodField> field_;
  ScanCheckParams params_;
};

}  // namespace landfall

#endif  // LANDFALL_GRID_SCAN_VERIFIER_HPP_
