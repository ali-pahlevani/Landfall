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

#include "landfall/benchmark/placements.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

#include "landfall/grid/likelihood_field.hpp"

namespace landfall::benchmark
{

namespace
{

double wrapAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

std::vector<Placement> samplePlacements(
  const OccupancyGrid & grid, const std::vector<Pose2> & keyframes, const PlacementParams & params,
  uint64_t seed, int count)
{
  if (keyframes.empty()) {
    throw std::invalid_argument("no keyframes to place the robot near");
  }
  // Candidate cells: free, clear of obstacles, near the recording.
  const LikelihoodField field(grid, 65, std::max(1.0, 2.0 * params.clearance));
  std::vector<std::pair<int, int>> cells;
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const Vec2 c = grid.cellCenter(col, row);
      if (
        field.classify(c.x(), c.y()) != LikelihoodField::Cell::kFree ||
        field.distance(c.x(), c.y()) < params.clearance) {
        continue;
      }
      for (const Pose2 & k : keyframes) {
        if (std::hypot(k.x - c.x(), k.y - c.y()) <= params.max_keyframe_distance) {
          cells.emplace_back(col, row);
          break;
        }
      }
    }
  }
  if (cells.empty()) {
    throw std::runtime_error("no free cell within reach of the keyframes");
  }

  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<std::size_t> pick(0, cells.size() - 1);
  std::uniform_real_distribution<double> jitter(-0.5, 0.5);
  std::uniform_real_distribution<double> heading(-M_PI, M_PI);
  std::vector<Placement> placements;
  constexpr int kMaxTries = 100000;
  for (int tries = 0; static_cast<int>(placements.size()) < count; ++tries) {
    if (tries >= kMaxTries) {
      throw std::runtime_error("cannot place the robot: min_separation too large for the area");
    }
    const auto [col, row] = cells[pick(rng)];
    const Vec2 c = grid.cellCenter(col, row);
    Placement p;
    p.pose.x = c.x() + jitter(rng) * grid.resolution;
    p.pose.y = c.y() + jitter(rng) * grid.resolution;
    p.pose.yaw = heading(rng);
    if (
      !placements.empty() &&
      std::hypot(p.pose.x - placements.back().pose.x, p.pose.y - placements.back().pose.y) <
        params.min_separation) {
      continue;
    }
    p.keyframe_distance = std::numeric_limits<double>::infinity();
    p.heading_difference = M_PI;
    for (const Pose2 & k : keyframes) {
      const double d = std::hypot(k.x - p.pose.x, k.y - p.pose.y);
      p.keyframe_distance = std::min(p.keyframe_distance, d);
      if (d <= params.max_keyframe_distance) {
        p.heading_difference =
          std::min(p.heading_difference, std::abs(wrapAngle(k.yaw - p.pose.yaw)));
      }
    }
    if (p.keyframe_distance > params.max_keyframe_distance) {
      continue;  // the cell is within reach, this point of it isn't
    }
    placements.push_back(p);
  }
  return placements;
}

}  // namespace landfall::benchmark
