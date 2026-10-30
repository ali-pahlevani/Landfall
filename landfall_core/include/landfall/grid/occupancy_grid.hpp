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

#ifndef LANDFALL_GRID_OCCUPANCY_GRID_HPP_
#define LANDFALL_GRID_OCCUPANCY_GRID_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

constexpr int8_t kCellUnknown = -1;
constexpr int8_t kCellFree = 0;
constexpr int8_t kCellOccupied = 100;

/// Content of a nav_msgs/OccupancyGrid: row 0 is the bottom row (lowest y), values are -1 or
/// [0, 100]. The origin's yaw is kept but ignored by the cell lookups, as AMCL and costmap_2d do.
struct OccupancyGrid
{
  int width = 0;
  int height = 0;
  double resolution = 0.0;   ///< metres per cell
  Pose2 origin;              ///< outer corner of cell (0, 0) in the map frame
  std::vector<int8_t> data;  ///< width * height values, row-major

  int8_t at(int col, int row) const { return data[static_cast<std::size_t>(row) * width + col]; }
  bool contains(int col, int row) const
  {
    return col >= 0 && row >= 0 && col < width && row < height;
  }
  /// Cell containing a map-frame point; false if it is outside the grid.
  bool worldToCell(double x, double y, int * col, int * row) const;
  Vec2 cellCenter(int col, int row) const;
};

/// SHA-256 over width, height and cell values. Resolution and origin are compared numerically.
std::string gridSha256(const OccupancyGrid & grid);

/// Lossless PNG of the cell values (stored as value + 1, top row first) for the map database.
std::vector<uint8_t> encodeGridPng(const OccupancyGrid & grid);
/// Inverse of encodeGridPng. Throws std::runtime_error on corrupt data.
OccupancyGrid decodeGridPng(
  const std::vector<uint8_t> & png, double resolution, const Pose2 & origin);

}  // namespace landfall

#endif  // LANDFALL_GRID_OCCUPANCY_GRID_HPP_
