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

#ifndef LANDFALL_BUILD_COVERAGE_HPP_
#define LANDFALL_BUILD_COVERAGE_HPP_

#include <cstdint>
#include <vector>

#include "landfall/grid/occupancy_grid.hpp"
#include "landfall/types.hpp"

namespace landfall
{

/// Heading sectors of the coverage measure, centred on the compass directions (0, 45, 90 deg, ...)
/// so that a robot driving along axis-aligned aisles stays inside one sector.
constexpr int kCoverageSectors = 8;

/// The sector of a heading: sector s is centred on -180 + 45 s deg.
int coverageSector(double yaw);

/// Which floor the measure covers and what counts as a view of it (plan §7.17). In both
/// simulations (warehouse sweep, depot kidnap benchmark) a request with such a view (within 1 m,
/// in its heading's sector, >= 40 landmarks) succeeded 72-83 % of the time, and 4-12 % without
/// one; 3 of 177 successes were farther than 2 m or 45 degrees from every keyframe.
struct CoverageParams
{
  /// m the robot's centre keeps from obstacles. The reachable floor is the free floor at least
  /// this far from obstacles that connects to where the robot drove. 0: from the recording, the
  /// 5th percentile of the views' distance to the nearest obstacle.
  double clearance = 0.0;
  double radius = 1.0;     ///< m: a view covers the positions this close to it
  int min_landmarks = 40;  ///< a view counts with at least this many landmarks (min_inliers)
  double step = 0.25;      ///< m between the positions measured
};

/// A keyframe as the measure sees it.
struct CoverageView
{
  Pose2 pose;  ///< base pose in the map frame
  int landmarks = 0;
};

/// The reachable floor, sampled every `step` metres, and the heading sectors seen from each place.
struct CoverageMap
{
  double clearance = 0.0;        ///< m used
  double step = 0.0;             ///< m between positions
  std::vector<Vec2> positions;   ///< reachable positions (cell centres)
  std::vector<uint8_t> sectors;  ///< per position: bit s set when a view covers sector s
  double reachable_area = 0.0;   ///< m^2: positions x step^2
  double any_direction = 0.0;    ///< share of the positions with a view in at least one sector
  double two_directions = 0.0;   ///< ... in at least two
  double headings = 0.0;         ///< share of the (position, sector) pairs with a view
};

/// Measures how much of the floor the robot can reach has a usable view nearby, and in how many
/// directions. Free cells are those below 65 and not unknown, as the scan check reads the map.
/// Deterministic; empty when there are no views.
CoverageMap measureCoverage(
  const OccupancyGrid & grid, const std::vector<CoverageView> & views,
  const CoverageParams & params);

/// The number of sectors set in a position's mask.
int coveredSectors(uint8_t mask);

}  // namespace landfall

#endif  // LANDFALL_BUILD_COVERAGE_HPP_
