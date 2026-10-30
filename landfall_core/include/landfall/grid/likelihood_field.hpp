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

#ifndef LANDFALL_GRID_LIKELIHOOD_FIELD_HPP_
#define LANDFALL_GRID_LIKELIHOOD_FIELD_HPP_

#include <vector>

#include "landfall/grid/occupancy_grid.hpp"

namespace landfall
{

/// Distance-to-nearest-obstacle field over an occupancy grid (as in AMCL's likelihood-field model),
/// plus cell classes and ray traversal for the "seeing through a wall" test.
class LikelihoodField
{
public:
  enum class Cell : uint8_t {
    kFree,
    kOccupied,
    kUnknown,
    kOutside,
  };

  /// Cells >= occupied_threshold are obstacles, -1 is unknown, the rest is free. Distances are
  /// capped at max_distance (metres).
  explicit LikelihoodField(
    const OccupancyGrid & grid, int occupied_threshold = 65, double max_distance = 1.0);

  const OccupancyGrid & grid() const { return grid_; }
  double maxDistance() const { return max_distance_; }

  Cell classify(double x, double y) const;
  /// Distance (m) to the nearest obstacle cell; max_distance outside the grid or when farther.
  double distance(double x, double y) const;
  /// True if the segment from `from` to `to` passes through an obstacle cell. Cells outside the
  /// grid end the traversal.
  bool segmentHitsObstacle(const Vec2 & from, const Vec2 & to) const;

private:
  OccupancyGrid grid_;
  double max_distance_;
  std::vector<Cell> cells_;
  std::vector<float> distances_;
};

}  // namespace landfall

#endif  // LANDFALL_GRID_LIKELIHOOD_FIELD_HPP_
