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

#include "landfall/grid/likelihood_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace landfall
{

LikelihoodField::LikelihoodField(
  const OccupancyGrid & grid, int occupied_threshold, double max_distance)
: grid_(grid), max_distance_(max_distance)
{
  const std::size_t count = static_cast<std::size_t>(grid_.width) * grid_.height;
  cells_.resize(count);
  // distanceTransform measures the distance to the nearest zero pixel: obstacles are 0.
  cv::Mat free_mask(grid_.height, grid_.width, CV_8UC1);
  for (int row = 0; row < grid_.height; ++row) {
    for (int col = 0; col < grid_.width; ++col) {
      const int8_t value = grid_.at(col, row);
      Cell cell = Cell::kFree;
      if (value < 0) {
        cell = Cell::kUnknown;
      } else if (value >= occupied_threshold) {
        cell = Cell::kOccupied;
      }
      cells_[static_cast<std::size_t>(row) * grid_.width + col] = cell;
      free_mask.at<uint8_t>(row, col) = cell == Cell::kOccupied ? 0 : 255;
    }
  }
  distances_.assign(count, static_cast<float>(max_distance_));
  if (count == 0) {
    return;
  }
  cv::Mat distance_cells;
  cv::distanceTransform(free_mask, distance_cells, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
  for (int row = 0; row < grid_.height; ++row) {
    const auto * values = distance_cells.ptr<float>(row);
    for (int col = 0; col < grid_.width; ++col) {
      distances_[static_cast<std::size_t>(row) * grid_.width + col] = std::min(
        values[col] * static_cast<float>(grid_.resolution), static_cast<float>(max_distance_));
    }
  }
}

LikelihoodField::Cell LikelihoodField::classify(double x, double y) const
{
  int col = 0;
  int row = 0;
  if (!grid_.worldToCell(x, y, &col, &row)) {
    return Cell::kOutside;
  }
  return cells_[static_cast<std::size_t>(row) * grid_.width + col];
}

double LikelihoodField::distance(double x, double y) const
{
  int col = 0;
  int row = 0;
  if (!grid_.worldToCell(x, y, &col, &row)) {
    return max_distance_;
  }
  return distances_[static_cast<std::size_t>(row) * grid_.width + col];
}

bool LikelihoodField::segmentHitsObstacle(const Vec2 & from, const Vec2 & to) const
{
  // Amanatides-Woo traversal of every cell the segment touches.
  const double res = grid_.resolution;
  const Vec2 start((from.x() - grid_.origin.x) / res, (from.y() - grid_.origin.y) / res);
  const Vec2 end((to.x() - grid_.origin.x) / res, (to.y() - grid_.origin.y) / res);
  int col = static_cast<int>(std::floor(start.x()));
  int row = static_cast<int>(std::floor(start.y()));
  const int end_col = static_cast<int>(std::floor(end.x()));
  const int end_row = static_cast<int>(std::floor(end.y()));
  const Vec2 direction = end - start;
  const int step_col = direction.x() > 0 ? 1 : -1;
  const int step_row = direction.y() > 0 ? 1 : -1;
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const double delta_col = direction.x() != 0.0 ? std::abs(1.0 / direction.x()) : kInf;
  const double delta_row = direction.y() != 0.0 ? std::abs(1.0 / direction.y()) : kInf;
  double next_col = direction.x() != 0.0
                      ? (step_col > 0 ? (col + 1 - start.x()) : (start.x() - col)) * delta_col
                      : kInf;
  double next_row = direction.y() != 0.0
                      ? (step_row > 0 ? (row + 1 - start.y()) : (start.y() - row)) * delta_row
                      : kInf;
  const int max_steps = std::abs(end_col - col) + std::abs(end_row - row) + 1;
  for (int i = 0; i < max_steps; ++i) {
    if (!grid_.contains(col, row)) {
      return false;
    }
    if (cells_[static_cast<std::size_t>(row) * grid_.width + col] == Cell::kOccupied) {
      return true;
    }
    if (col == end_col && row == end_row) {
      break;
    }
    if (next_col < next_row) {
      next_col += delta_col;
      col += step_col;
    } else {
      next_row += delta_row;
      row += step_row;
    }
  }
  return false;
}

}  // namespace landfall
