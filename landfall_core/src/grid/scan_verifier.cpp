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

#include "landfall/grid/scan_verifier.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "landfall/geometry/gravity.hpp"

namespace landfall
{

ScanVerifier::ScanVerifier(
  std::shared_ptr<const LikelihoodField> field, const ScanCheckParams & params)
: field_(std::move(field)), params_(params)
{
}

ProjectedScan ScanVerifier::project(const PlanarScan & scan, const Vec3 & gravity_base) const
{
  // Rotation that levels the base frame: gravity becomes -z, the heading is kept.
  const Mat3 R_level_base = levelingRotation(gravity_base);
  const Pose3 & T_base_laser = scan.T_base_laser;
  ProjectedScan projected;
  projected.origin = (R_level_base * T_base_laser.translation()).head<2>();
  const double max_range = std::min(scan.range_max, params_.max_range);
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > max_range) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const Vec3 endpoint =
      R_level_base * (T_base_laser * Vec3(range * std::cos(angle), range * std::sin(angle), 0.0));
    if (endpoint.z() < params_.floor_clearance) {
      ++projected.dropped_floor;
      continue;
    }
    projected.endpoints.emplace_back(endpoint.head<2>());
  }
  return projected;
}

ScanScore ScanVerifier::score(
  const ProjectedScan & scan, const Pose2 & map_T_base, bool with_violations) const
{
  const double c = std::cos(map_T_base.yaw);
  const double s = std::sin(map_T_base.yaw);
  const auto to_map = [&](const Vec2 & p) {
    return Vec2(map_T_base.x + c * p.x() - s * p.y(), map_T_base.y + s * p.x() + c * p.y());
  };
  const Vec2 origin = to_map(scan.origin);
  ScanScore result;
  result.valid_beams = static_cast<int>(scan.endpoints.size());
  for (const auto & endpoint_base : scan.endpoints) {
    const Vec2 endpoint = to_map(endpoint_base);
    const auto cell = field_->classify(endpoint.x(), endpoint.y());
    if (cell == LikelihoodField::Cell::kFree || cell == LikelihoodField::Cell::kOccupied) {
      ++result.known_beams;
      if (field_->distance(endpoint.x(), endpoint.y()) <= params_.hit_tolerance) {
        ++result.hits;
      }
    }
    if (with_violations) {
      const Vec2 ray = endpoint - origin;
      const double length = ray.norm();
      if (length > params_.violation_margin) {
        const Vec2 stop = origin + ray * ((length - params_.violation_margin) / length);
        if (field_->segmentHitsObstacle(origin, stop)) {
          ++result.violations;
        }
      }
    }
  }
  if (result.valid_beams > 0) {
    result.known_ratio = static_cast<double>(result.known_beams) / result.valid_beams;
    result.violation_ratio = static_cast<double>(result.violations) / result.valid_beams;
  }
  if (result.known_beams > 0) {
    result.hit_ratio = static_cast<double>(result.hits) / result.known_beams;
  }
  return result;
}

std::string ScanVerifier::failure(const ScanScore & score) const
{
  std::ostringstream reason;
  if (score.known_ratio < params_.min_known_ratio) {
    reason << "known ratio " << score.known_ratio << " < " << params_.min_known_ratio;
  } else if (score.hit_ratio < params_.min_hit_ratio) {
    reason << "hit ratio " << score.hit_ratio << " < " << params_.min_hit_ratio;
  } else if (score.violation_ratio > params_.max_violation_ratio) {
    reason << "violation ratio " << score.violation_ratio << " > " << params_.max_violation_ratio;
  }
  return reason.str();
}

ScanCheckResult ScanVerifier::verify(const ProjectedScan & scan, const Pose2 & hypothesis) const
{
  ScanCheckResult result;
  result.best_pose = hypothesis;
  if (static_cast<int>(scan.endpoints.size()) < params_.min_valid_beams) {
    result.score.valid_beams = static_cast<int>(scan.endpoints.size());
    result.reason = "only " + std::to_string(scan.endpoints.size()) + " valid beams (need " +
                    std::to_string(params_.min_valid_beams) + ")";
    return result;
  }

  // Rank the window by hit ratio (cheap), then run the full check on the best few.
  struct Candidate
  {
    Pose2 pose;
    double hit_ratio;
    double offset;
  };
  std::vector<Candidate> candidates;
  const int steps_xy =
    params_.search_step > 0.0
      ? static_cast<int>(std::floor(params_.search_radius / params_.search_step + 1e-9))
      : 0;
  const int steps_yaw = params_.search_angle_step_deg > 0.0
                          ? static_cast<int>(std::floor(
                              params_.search_angle_deg / params_.search_angle_step_deg + 1e-9))
                          : 0;
  for (int i = -steps_xy; i <= steps_xy; ++i) {
    for (int j = -steps_xy; j <= steps_xy; ++j) {
      for (int k = -steps_yaw; k <= steps_yaw; ++k) {
        const Pose2 pose{
          hypothesis.x + i * params_.search_step, hypothesis.y + j * params_.search_step,
          normalizeAngle(hypothesis.yaw + k * params_.search_angle_step_deg * M_PI / 180.0)};
        const double offset = std::hypot(i, j) * params_.search_step + std::abs(k) * 0.01;
        candidates.push_back({pose, score(scan, pose, false).hit_ratio, offset});
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
    return a.hit_ratio != b.hit_ratio ? a.hit_ratio > b.hit_ratio : a.offset < b.offset;
  });
  constexpr std::size_t kFullChecks = 5;
  for (std::size_t i = 0; i < std::min(kFullChecks, candidates.size()); ++i) {
    const ScanScore full = score(scan, candidates[i].pose, true);
    const std::string reason = failure(full);
    if (i == 0) {
      result.score = full;
      result.best_pose = candidates[i].pose;
      result.reason = reason;
    }
    if (reason.empty()) {
      result.passed = true;
      result.reason.clear();
      result.score = full;
      result.best_pose = candidates[i].pose;
      break;
    }
  }
  return result;
}

}  // namespace landfall
