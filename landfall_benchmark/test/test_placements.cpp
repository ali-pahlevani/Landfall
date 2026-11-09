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

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "landfall/benchmark/placements.hpp"

namespace
{

// A 20 m x 10 m room at 0.1 m per cell: walls around, a 2 m x 2 m block in the middle.
landfall::OccupancyGrid room()
{
  landfall::OccupancyGrid grid;
  grid.width = 200;
  grid.height = 100;
  grid.resolution = 0.1;
  grid.origin = landfall::Pose2{-10.0, -5.0, 0.0};
  grid.data.assign(
    static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height),
    landfall::kCellFree);
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      const bool wall = col == 0 || row == 0 || col == grid.width - 1 || row == grid.height - 1;
      const bool block = col >= 90 && col < 110 && row >= 40 && row < 60;
      if (wall || block) {
        grid.data
          [static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.width) +
           static_cast<std::size_t>(col)] = landfall::kCellOccupied;
      }
    }
  }
  return grid;
}

// Keyframes along y = -3 from x = -8 to 8, looking along +x.
std::vector<landfall::Pose2> route()
{
  std::vector<landfall::Pose2> keyframes;
  for (int i = 0; i <= 32; ++i) {
    keyframes.push_back(landfall::Pose2{-8.0 + 0.5 * i, -3.0, 0.0});
  }
  return keyframes;
}

}  // namespace

TEST(Placements, StayNearTheRecordingInFreeSpace)
{
  const auto grid = room();
  landfall::benchmark::PlacementParams params;
  params.max_keyframe_distance = 1.0;
  params.clearance = 0.4;
  params.min_separation = 2.0;
  const auto placements = landfall::benchmark::samplePlacements(grid, route(), params, 7, 200);
  ASSERT_EQ(placements.size(), 200U);
  for (std::size_t i = 0; i < placements.size(); ++i) {
    const auto & p = placements[i];
    EXPECT_LE(p.keyframe_distance, params.max_keyframe_distance) << i;
    EXPECT_NEAR(p.pose.y, -3.0, 1.1) << i;
    EXPECT_GE(p.pose.x, -9.1) << i;  // clear of the wall at x = -10
    EXPECT_LE(p.pose.x, 9.1) << i;
    EXPECT_LE(std::abs(p.pose.yaw), M_PI) << i;
    EXPECT_NEAR(p.heading_difference, std::abs(p.pose.yaw), 1e-9) << i;
    if (i > 0) {
      EXPECT_GE(
        std::hypot(p.pose.x - placements[i - 1].pose.x, p.pose.y - placements[i - 1].pose.y),
        params.min_separation)
        << i;
    }
  }
}

TEST(Placements, TheSameSeedGivesTheSamePlacements)
{
  const auto grid = room();
  const landfall::benchmark::PlacementParams params;
  const auto a = landfall::benchmark::samplePlacements(grid, route(), params, 3, 20);
  const auto b = landfall::benchmark::samplePlacements(grid, route(), params, 3, 20);
  const auto c = landfall::benchmark::samplePlacements(grid, route(), params, 4, 20);
  ASSERT_EQ(a.size(), b.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].pose.x, b[i].pose.x);
    EXPECT_EQ(a[i].pose.yaw, b[i].pose.yaw);
  }
  EXPECT_NE(a.front().pose.x, c.front().pose.x);
}

TEST(Placements, RefusesWithoutRoom)
{
  const auto grid = room();
  landfall::benchmark::PlacementParams params;
  EXPECT_THROW(
    landfall::benchmark::samplePlacements(grid, {}, params, 1, 1), std::invalid_argument);
  // Keyframes inside the block: no free cell within reach and clear of it.
  params.max_keyframe_distance = 0.5;
  EXPECT_THROW(
    landfall::benchmark::samplePlacements(grid, {landfall::Pose2{0.0, 0.0, 0.0}}, params, 1, 1),
    std::runtime_error);
}
