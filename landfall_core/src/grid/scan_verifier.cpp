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
#include <vector>

#include "landfall/geometry/gravity.hpp"

namespace landfall
{

namespace
{

// The alternative search (bestAlternative): candidate positions on a grid of free cells clear of
// obstacles, fewer on large maps; every heading; a coarse score on a subset of the beams that
// falls off slowly with distance, as the grid is coarse; the best coarse poses refined.
constexpr double kAlternativeStep = 0.25;       // m between candidate positions
constexpr double kAlternativeClearance = 0.15;  // m from obstacles
constexpr std::size_t kAlternativeMaxPositions = 12000;
constexpr double kAlternativeAngleStepDeg = 5.0;
constexpr std::size_t kAlternativeBeams = 40;
constexpr double kAlternativeReach = 0.5;  // m where a beam's coarse score reaches 0
constexpr std::size_t kAlternativeRefined = 40;
constexpr std::size_t kRefineBeams = 90;  // the refinement ranks poses on this many beams
constexpr double kRefineStep = 0.05;      // m
constexpr double kRefineAngleStepDeg = 1.0;
constexpr int kRefineAngleSteps = 3;         // +- 3 deg: half the coarse heading step, and more
constexpr double kSamePlace = 1.0;           // m and ...
constexpr double kSamePlaceAngleDeg = 30.0;  // ... deg: closer poses are the same place

bool samePlace(const Pose2 & a, const Pose2 & b)
{
  return translationDistance(a, b) <= kSamePlace &&
         rotationDistance(a, b) <= kSamePlaceAngleDeg * M_PI / 180.0;
}

}  // namespace

double scanFit(const ScanScore & score) { return score.hit_ratio - score.violation_ratio; }

ScanVerifier::ScanVerifier(
  std::shared_ptr<const LikelihoodField> field, const ScanCheckParams & params)
: field_(std::move(field)), params_(params)
{
  const OccupancyGrid & grid = field_->grid();
  if (grid.resolution <= 0.0) {
    return;
  }
  double step = kAlternativeStep;
  for (int pass = 0; pass < 8; ++pass) {
    positions_.clear();
    const int stride = std::max(1, static_cast<int>(std::lround(step / grid.resolution)));
    for (int row = stride / 2; row < grid.height; row += stride) {
      for (int col = stride / 2; col < grid.width; col += stride) {
        const Vec2 c = grid.cellCenter(col, row);
        if (
          field_->classify(c.x(), c.y()) == LikelihoodField::Cell::kFree &&
          field_->distance(c.x(), c.y()) >= kAlternativeClearance) {
          positions_.push_back(c);
        }
      }
    }
    if (positions_.size() <= kAlternativeMaxPositions) {
      break;
    }
    step *= std::sqrt(static_cast<double>(positions_.size()) / kAlternativeMaxPositions);
  }
  alternative_step_ = step;
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
      // Seeing through an obstacle: the beam crossed one and went on through free space. A beam
      // that ends inside an obstacle drawn thicker than the real surface doesn't count.
      const auto clearance = field_->clearanceAfterObstacle(origin, endpoint);
      if (clearance && *clearance >= params_.violation_margin) {
        ++result.violations;
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

ScanAlternative ScanVerifier::bestAlternative(
  const ProjectedScan & scan, const Pose2 & pose, ThreadPool * pool) const
{
  ScanAlternative best;
  if (scan.endpoints.empty() || positions_.empty()) {
    return best;
  }
  // Coarse: every heading at every position, on a subset of the beams, with the distance field
  // read directly.
  const std::size_t stride = std::max<std::size_t>(1, scan.endpoints.size() / kAlternativeBeams);
  std::vector<Vec2> beams;
  for (std::size_t i = 0; i < scan.endpoints.size(); i += stride) {
    beams.push_back(scan.endpoints[i]);
  }
  const int headings = static_cast<int>(std::lround(360.0 / kAlternativeAngleStepDeg));
  std::vector<std::vector<Vec2>> rotated(static_cast<std::size_t>(headings));
  for (int h = 0; h < headings; ++h) {
    const double yaw = -M_PI + h * kAlternativeAngleStepDeg * M_PI / 180.0;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    for (const Vec2 & b : beams) {
      rotated[static_cast<std::size_t>(h)].emplace_back(
        c * b.x() - s * b.y(), s * b.x() + c * b.y());
    }
  }
  const OccupancyGrid & grid = field_->grid();
  const std::vector<float> & distances = field_->distances();
  const double inv_res = 1.0 / grid.resolution;
  const auto max_distance = static_cast<float>(field_->maxDistance());
  const auto lookup = [&](double x, double y) {
    const double fx = (x - grid.origin.x) * inv_res;
    const double fy = (y - grid.origin.y) * inv_res;
    // Written so that NaN is outside.
    const bool inside = fx >= 0.0 && fy >= 0.0 && fx < grid.width && fy < grid.height;
    if (!inside) {
      return max_distance;
    }
    return distances
      [static_cast<std::size_t>(fy) * static_cast<std::size_t>(grid.width) +
       static_cast<std::size_t>(fx)];
  };

  // The best coarse poses of each chunk of positions (a min-heap each), merged in chunk order:
  // the result doesn't depend on the threads.
  using Coarse = std::pair<double, Pose2>;  // score, pose
  const auto worse = [](const Coarse & a, const Coarse & b) { return a.first > b.first; };
  constexpr std::size_t kChunk = 256;
  const std::size_t chunks = (positions_.size() + kChunk - 1) / kChunk;
  std::vector<std::vector<Coarse>> chunk_best(chunks);
  const auto n_beams = static_cast<double>(beams.size());
  const auto run_chunk = [&](std::size_t chunk) {
    std::vector<Coarse> heap;
    const std::size_t first = chunk * kChunk;
    const std::size_t last = std::min(positions_.size(), first + kChunk);
    for (std::size_t i = first; i < last; ++i) {
      const Vec2 & p = positions_[i];
      for (int h = 0; h < headings; ++h) {
        const Pose2 candidate{p.x(), p.y(), -M_PI + h * kAlternativeAngleStepDeg * M_PI / 180.0};
        if (samePlace(candidate, pose)) {
          continue;
        }
        // Stop early once even perfect remaining beams couldn't reach the heap.
        const double floor = heap.size() < kAlternativeRefined ? -1.0 : heap.front().first;
        const auto & set = rotated[static_cast<std::size_t>(h)];
        double sum = 0.0;
        bool pruned = false;
        for (std::size_t b = 0; b < set.size(); ++b) {
          const double d = lookup(p.x() + set[b].x(), p.y() + set[b].y());
          sum += std::max(0.0, 1.0 - d / kAlternativeReach);
          if (
            (b & 7U) == 7U && (sum + static_cast<double>(set.size() - b - 1)) / n_beams <= floor) {
            pruned = true;
            break;
          }
        }
        const double score = sum / n_beams;
        if (pruned || score <= floor) {
          continue;
        }
        if (heap.size() == kAlternativeRefined) {
          std::pop_heap(heap.begin(), heap.end(), worse);
          heap.pop_back();
        }
        heap.emplace_back(score, candidate);
        std::push_heap(heap.begin(), heap.end(), worse);
      }
    }
    chunk_best[chunk] = std::move(heap);
  };
  if (pool != nullptr) {
    pool->parallelFor(chunks, run_chunk);
  } else {
    for (std::size_t c = 0; c < chunks; ++c) {
      run_chunk(c);
    }
  }
  std::vector<Coarse> top;
  for (const auto & heap : chunk_best) {
    top.insert(top.end(), heap.begin(), heap.end());
  }
  const std::size_t keep = std::min(kAlternativeRefined, top.size());
  std::partial_sort(
    top.begin(), top.begin() + static_cast<std::ptrdiff_t>(keep), top.end(),
    [](const Coarse & a, const Coarse & b) {
      return a.first != b.first         ? a.first > b.first
             : a.second.x != b.second.x ? a.second.x < b.second.x
             : a.second.y != b.second.y ? a.second.y < b.second.y
                                        : a.second.yaw < b.second.yaw;
    });
  top.resize(keep);

  // Refine: the best hit ratio within the coarse grid's cell (on a subset of the beams), then the
  // full score.
  const int steps =
    static_cast<int>(std::ceil((0.5 * alternative_step_ + kRefineStep) / kRefineStep));
  ProjectedScan subset;
  subset.origin = scan.origin;
  const std::size_t refine_stride = std::max<std::size_t>(1, scan.endpoints.size() / kRefineBeams);
  for (std::size_t i = 0; i < scan.endpoints.size(); i += refine_stride) {
    subset.endpoints.push_back(scan.endpoints[i]);
  }
  std::vector<ScanAlternative> refined(top.size());
  const auto refine = [&](std::size_t c) {
    const Pose2 & coarse = top[c].second;
    Pose2 pose_r = coarse;
    double best_hit = -1.0;
    for (int i = -steps; i <= steps; ++i) {
      for (int j = -steps; j <= steps; ++j) {
        for (int k = -kRefineAngleSteps; k <= kRefineAngleSteps; ++k) {
          const Pose2 candidate{
            coarse.x + i * kRefineStep, coarse.y + j * kRefineStep,
            normalizeAngle(coarse.yaw + k * kRefineAngleStepDeg * M_PI / 180.0)};
          const double hit = score(subset, candidate, false).hit_ratio;
          if (hit > best_hit) {
            best_hit = hit;
            pose_r = candidate;
          }
        }
      }
    }
    if (!samePlace(pose_r, pose)) {
      refined[c] = {true, pose_r, score(scan, pose_r, true)};
    }
  };
  if (pool != nullptr) {
    pool->parallelFor(refined.size(), refine);
  } else {
    for (std::size_t c = 0; c < refined.size(); ++c) {
      refine(c);
    }
  }
  for (const ScanAlternative & r : refined) {
    if (r.found && (!best.found || scanFit(r.score) > scanFit(best.score))) {
      best = r;
    }
  }
  return best;
}

}  // namespace landfall
