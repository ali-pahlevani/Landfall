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

#ifndef LANDFALL_ROS_SCAN_EVALUATION_HPP_
#define LANDFALL_ROS_SCAN_EVALUATION_HPP_

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "landfall/grid/occupancy_grid.hpp"
#include "landfall/grid/scan_verifier.hpp"

namespace landfall::ros
{

struct ScanEvaluationOptions
{
  Pose2 world_to_map{8.0, 0.0, 0.0};  ///< ground truth (simulator world) -> map
  std::string scan_topic = "/scan";
  std::string gt_topic = "/ground_truth/pose";
  std::string base_frame = "base_footprint";
  std::size_t every = 1;        ///< check every n-th scan
  int wrong_per_scan = 5;       ///< wrong poses tried per scan
  double wrong_min = 0.5;       ///< m: wrong poses lie this far ...
  double wrong_max = 5.0;       ///< ... to this far from the truth, on a known-free cell
  double wrong_yaw_deg = 15.0;  ///< and turned by up to this much
  uint64_t seed = 1;
  double max_gt_gap = 0.25;  ///< s; longest ground-truth gap to interpolate across
  ScanCheckParams params;
  /// A symmetry of the map, as the map transform S (a pose S (+) p for every pose p): every scan is
  /// also checked at its true pose moved by S, where it must fail. E.g. a building that looks the
  /// same rotated 180 degrees about a point c: S = (c - R(180) c, 180 degrees).
  std::optional<Pose2> symmetry;
  /// The search window of the symmetric check (m, deg): vision's pose at a look-alike place can be
  /// anywhere near the exact symmetric pose, so its best fit there is what could pass.
  double symmetry_search_radius = 0.1;
  double symmetry_search_angle_deg = 3.0;
  /// Thresholds (min_hit_ratio, max_violation_ratio) to evaluate besides `params`: how many true,
  /// wrong and symmetric poses each pair would accept.
  std::vector<std::pair<double, double>> sweep;
  /// Also search the scan's best fit elsewhere (ScanVerifier::bestAlternative), at the true pose
  /// and at every symmetric pose that passes: how much better it fits than each.
  bool alternatives = false;
};

/// What one threshold pair of a sweep accepts.
struct SweepPoint
{
  double min_hit_ratio = 0.0;
  double max_violation_ratio = 0.0;
  std::size_t true_accepted = 0;
  std::size_t wrong_accepted = 0;
  std::size_t symmetric_accepted = 0;
};

struct WrongAcceptance
{
  double offset = 0.0;  ///< m from the truth
  double yaw_offset_deg = 0.0;
  double hit_ratio = 0.0;
  double violation_ratio = 0.0;
};

struct ScanEvaluation
{
  std::size_t true_checks = 0;
  std::size_t true_accepted = 0;
  std::size_t skipped = 0;                             ///< scans without ground truth or TF
  std::map<std::string, std::size_t> true_rejections;  ///< by reason (numbers left out)
  std::vector<double> true_hit_ratios;
  std::vector<double> true_violation_ratios;
  std::vector<double> true_known_ratios;
  std::vector<double> true_valid_beams;
  std::vector<double> true_refinements;  ///< m the search window moved the true pose
  std::size_t wrong_checks = 0;
  std::size_t wrong_accepted = 0;
  std::vector<WrongAcceptance> wrong_examples;  ///< the first accepted wrong poses
  std::size_t symmetric_checks = 0;             ///< symmetric poses on a free cell
  std::size_t symmetric_accepted = 0;
  std::vector<WrongAcceptance> symmetric_examples;  ///< the first accepted symmetric poses
  std::vector<SweepPoint> sweep;
  /// scanFit of the best alternative less scanFit at the true pose, per scan; and the same at each
  /// accepted symmetric pose (there the true pose is an alternative).
  std::vector<double> true_alternative_gains;
  std::vector<double> symmetric_alternative_gains;
};

/// The LiDAR scan check of every scan in a recording with ground truth (plan §11, M4): at the true
/// base pose, where it must pass, and at wrong poses where it must fail. The flat-floor projection
/// is used; the laser's pose in the base frame comes from the bag's TF.
ScanEvaluation evaluateScanCheck(
  const std::string & bag, const OccupancyGrid & grid, const ScanEvaluationOptions & options);

/// Sets a ScanCheckParams field by its relocalizer parameter name (`scan_check.hit_tolerance`, ...).
/// Throws std::invalid_argument for an unknown name or a value that isn't a number.
void setScanCheckParameter(
  ScanCheckParams * params, const std::string & name, const std::string & value);

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_SCAN_EVALUATION_HPP_
