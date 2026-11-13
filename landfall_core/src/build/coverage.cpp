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

#include "landfall/build/coverage.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <opencv2/imgproc.hpp>
#include <utility>

namespace landfall
{

namespace
{

constexpr int kOccupiedThreshold = 65;  // as the scan check reads the map
constexpr double kClearancePercentile = 0.05;
// Clearances compare with a micrometre of slack: the report measures again with the clearance the
// build wrote to its JSON (10 significant digits), which may have rounded up.
constexpr double kClearanceSlack = 1e-6;

std::pair<long, long> bucketOf(double x, double y, double size)
{
  return {std::lround(std::floor(x / size)), std::lround(std::floor(y / size))};
}

}  // namespace

int coverageSector(double yaw)
{
  const double a = normalizeAngle(yaw + M_PI / kCoverageSectors) + M_PI;  // [0, 2 pi]
  return std::min(kCoverageSectors - 1, static_cast<int>(a / (2.0 * M_PI / kCoverageSectors)));
}

int coveredSectors(uint8_t mask) { return static_cast<int>(std::bitset<8>(mask).count()); }

CoverageMap measureCoverage(
  const OccupancyGrid & grid, const std::vector<CoverageView> & views,
  const CoverageParams & params)
{
  CoverageMap map;
  if (views.empty() || grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0) {
    return map;
  }
  const std::size_t count = static_cast<std::size_t>(grid.width) * grid.height;
  const auto isFree = [&grid](std::size_t index) {
    return grid.data[index] >= 0 && grid.data[index] < kOccupiedThreshold;
  };

  // Distance of every cell to the nearest obstacle, uncapped (row r of the image is grid row r).
  // Without any obstacle every free cell is reachable.
  cv::Mat not_obstacle(grid.height, grid.width, CV_8UC1);
  bool any_obstacle = false;
  for (int row = 0; row < grid.height; ++row) {
    auto * pixels = not_obstacle.ptr<uint8_t>(row);
    for (int col = 0; col < grid.width; ++col) {
      const bool obstacle = grid.at(col, row) >= kOccupiedThreshold;
      pixels[col] = obstacle ? 0 : 255;
      any_obstacle = any_obstacle || obstacle;
    }
  }
  cv::Mat distance;
  if (any_obstacle) {
    cv::distanceTransform(not_obstacle, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
  }
  const auto clearanceAt = [&](std::size_t index) {
    if (!any_obstacle) {
      return std::numeric_limits<double>::infinity();
    }
    const int row = static_cast<int>(index / static_cast<std::size_t>(grid.width));
    const int col = static_cast<int>(index % static_cast<std::size_t>(grid.width));
    return static_cast<double>(distance.at<float>(row, col)) * grid.resolution;
  };

  // Where the robot drove: the free cells under the views.
  std::vector<std::size_t> driven;
  std::vector<double> driven_clearance;
  for (const CoverageView & view : views) {
    int col = 0;
    int row = 0;
    if (!grid.worldToCell(view.pose.x, view.pose.y, &col, &row)) {
      continue;
    }
    const std::size_t index = static_cast<std::size_t>(row) * grid.width + col;
    if (isFree(index)) {
      driven.push_back(index);
      driven_clearance.push_back(clearanceAt(index));
    }
  }
  map.clearance = params.clearance;
  if (map.clearance <= 0.0 && any_obstacle && !driven_clearance.empty()) {
    const auto k = static_cast<std::size_t>(
      kClearancePercentile * static_cast<double>(driven_clearance.size() - 1));
    std::nth_element(
      driven_clearance.begin(), driven_clearance.begin() + static_cast<std::ptrdiff_t>(k),
      driven_clearance.end());
    map.clearance = driven_clearance[k];
  }
  map.clearance = std::max(map.clearance, grid.resolution);

  // The reachable floor: free cells at least the clearance from obstacles, connected (8-way) to
  // a driven cell.
  const auto passable = [&](std::size_t index) {
    return isFree(index) && clearanceAt(index) + kClearanceSlack >= map.clearance;
  };
  std::vector<uint8_t> reachable(count, 0);
  std::vector<std::size_t> stack;
  for (const std::size_t index : driven) {
    if (reachable[index] == 0 && passable(index)) {
      reachable[index] = 1;
      stack.push_back(index);
    }
  }
  while (!stack.empty()) {
    const std::size_t index = stack.back();
    stack.pop_back();
    const int row = static_cast<int>(index / static_cast<std::size_t>(grid.width));
    const int col = static_cast<int>(index % static_cast<std::size_t>(grid.width));
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if ((dr == 0 && dc == 0) || !grid.contains(col + dc, row + dr)) {
          continue;
        }
        const std::size_t next = static_cast<std::size_t>(row + dr) * grid.width + (col + dc);
        if (reachable[next] == 0 && passable(next)) {
          reachable[next] = 1;
          stack.push_back(next);
        }
      }
    }
  }

  // Positions every `step` metres over the reachable floor.
  const int stride = std::max(1, static_cast<int>(std::lround(params.step / grid.resolution)));
  map.step = stride * grid.resolution;
  for (int row = stride / 2; row < grid.height; row += stride) {
    for (int col = stride / 2; col < grid.width; col += stride) {
      if (reachable[static_cast<std::size_t>(row) * grid.width + col] != 0) {
        map.positions.push_back(grid.cellCenter(col, row));
      }
    }
  }

  // The views with enough landmarks, bucketed by the radius.
  std::map<std::pair<long, long>, std::vector<std::size_t>> buckets;
  std::vector<int> sector(views.size(), 0);
  const double radius = std::max(params.radius, 1e-6);
  for (std::size_t i = 0; i < views.size(); ++i) {
    if (views[i].landmarks >= params.min_landmarks) {
      sector[i] = coverageSector(views[i].pose.yaw);
      buckets[bucketOf(views[i].pose.x, views[i].pose.y, radius)].push_back(i);
    }
  }
  map.sectors.assign(map.positions.size(), 0);
  int any = 0;
  int two = 0;
  long sector_sum = 0;
  for (std::size_t p = 0; p < map.positions.size(); ++p) {
    const Vec2 & position = map.positions[p];
    const auto [bx, by] = bucketOf(position.x(), position.y(), radius);
    uint8_t mask = 0;
    for (long dx = -1; dx <= 1; ++dx) {
      for (long dy = -1; dy <= 1; ++dy) {
        const auto it = buckets.find({bx + dx, by + dy});
        if (it == buckets.end()) {
          continue;
        }
        for (const std::size_t i : it->second) {
          const double ex = views[i].pose.x - position.x();
          const double ey = views[i].pose.y - position.y();
          if (ex * ex + ey * ey <= radius * radius) {
            mask = static_cast<uint8_t>(mask | (1U << sector[i]));
          }
        }
      }
    }
    map.sectors[p] = mask;
    const int covered = coveredSectors(mask);
    any += covered >= 1 ? 1 : 0;
    two += covered >= 2 ? 1 : 0;
    sector_sum += covered;
  }
  if (!map.positions.empty()) {
    const auto n = static_cast<double>(map.positions.size());
    map.reachable_area = n * map.step * map.step;
    map.any_direction = any / n;
    map.two_directions = two / n;
    map.headings = static_cast<double>(sector_sum) / (n * kCoverageSectors);
  }
  return map;
}

}  // namespace landfall
