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

#ifndef LANDFALL_BENCHMARK_PLACEMENTS_HPP_
#define LANDFALL_BENCHMARK_PLACEMENTS_HPP_

#include <cstdint>
#include <vector>

#include "landfall/grid/occupancy_grid.hpp"
#include "landfall/types.hpp"

namespace landfall::benchmark
{

struct PlacementParams
{
  double max_keyframe_distance = 1.0;  ///< m from the nearest keyframe position
  double clearance = 0.4;              ///< m from the nearest obstacle cell
  double min_separation = 2.0;         ///< m from the previous placement: a real kidnap
};

/// Where the robot is put, in the map frame.
struct Placement
{
  Pose2 pose;
  double keyframe_distance = 0.0;  ///< m to the nearest keyframe position
  /// rad: the smallest heading difference to a keyframe within max_keyframe_distance, i.e. how
  /// far the robot looks from a direction the recording looked in nearby.
  double heading_difference = 0.0;
};

/// Kidnap placements for the benchmark (plan §11 M7, docs/benchmark.md): positions uniform over the
/// free cells that are at least `clearance` from any obstacle and within `max_keyframe_distance` of
/// a keyframe (the area the map database was recorded in), headings uniform, each at least
/// `min_separation` from the previous one. The same grid, keyframes, parameters and seed give the
/// same placements, so every method meets the same ones.
std::vector<Placement> samplePlacements(
  const OccupancyGrid & grid, const std::vector<Pose2> & keyframes, const PlacementParams & params,
  uint64_t seed, int count);

}  // namespace landfall::benchmark

#endif  // LANDFALL_BENCHMARK_PLACEMENTS_HPP_
