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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "landfall/build/coverage.hpp"
#include "landfall/grid/occupancy_grid.hpp"

namespace
{

using landfall::CoverageMap;
using landfall::CoverageParams;
using landfall::CoverageView;
using landfall::OccupancyGrid;

double deg(double d) { return d * M_PI / 180.0; }

// A free grid of width x height metres at 5 cm, origin (0, 0), its border cells occupied.
OccupancyGrid room(double width, double height)
{
  OccupancyGrid g;
  g.resolution = 0.05;
  g.width = static_cast<int>(std::lround(width / g.resolution));
  g.height = static_cast<int>(std::lround(height / g.resolution));
  g.data.assign(static_cast<std::size_t>(g.width) * g.height, landfall::kCellFree);
  for (int row = 0; row < g.height; ++row) {
    for (int col = 0; col < g.width; ++col) {
      if (row == 0 || col == 0 || row == g.height - 1 || col == g.width - 1) {
        g.data[static_cast<std::size_t>(row) * g.width + col] = landfall::kCellOccupied;
      }
    }
  }
  return g;
}

// Sets the cells whose centres lie in [x0, x1] x [y0, y1].
void fill(OccupancyGrid * g, double x0, double y0, double x1, double y1, int8_t value)
{
  for (int row = 0; row < g->height; ++row) {
    for (int col = 0; col < g->width; ++col) {
      const landfall::Vec2 c = g->cellCenter(col, row);
      if (c.x() >= x0 && c.x() <= x1 && c.y() >= y0 && c.y() <= y1) {
        g->data[static_cast<std::size_t>(row) * g->width + col] = value;
      }
    }
  }
}

// Keyframes every 0.5 m from x0 to x1 along y, all with the same heading.
std::vector<CoverageView> lane(double x0, double x1, double y, double yaw, int landmarks = 100)
{
  std::vector<CoverageView> views;
  const int steps = static_cast<int>(std::lround((x1 - x0) / 0.5));
  for (int k = 0; k <= steps; ++k) {
    views.push_back({{x0 + 0.5 * k, y, yaw}, landmarks});
  }
  return views;
}

std::vector<CoverageView> joined(std::vector<CoverageView> a, const std::vector<CoverageView> & b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

bool anyPositionIn(const CoverageMap & map, double x0, double y0, double x1, double y1)
{
  for (const auto & p : map.positions) {
    if (p.x() >= x0 && p.x() <= x1 && p.y() >= y0 && p.y() <= y1) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(Coverage, SectorsAreCentredOnTheCompassDirections)
{
  EXPECT_EQ(landfall::coverageSector(0.0), 4);
  EXPECT_EQ(landfall::coverageSector(M_PI), 0);
  EXPECT_EQ(landfall::coverageSector(-M_PI), 0);
  EXPECT_EQ(landfall::coverageSector(M_PI / 2), 6);
  EXPECT_EQ(landfall::coverageSector(-M_PI / 2), 2);
  EXPECT_EQ(landfall::coverageSector(deg(22)), 4);
  EXPECT_EQ(landfall::coverageSector(deg(23)), 5);
  EXPECT_EQ(landfall::coverageSector(deg(-22)), 4);
  EXPECT_EQ(landfall::coverageSector(deg(-23)), 3);
  EXPECT_EQ(landfall::coverageSector(deg(170)), 0);
  EXPECT_EQ(landfall::coverageSector(deg(-170)), 0);
  EXPECT_EQ(landfall::coveredSectors(0), 0);
  EXPECT_EQ(landfall::coveredSectors(0b10010001), 3);
}

TEST(Coverage, ALaneDrivenOneWaySeesOneDirectionAndBothWaysTwo)
{
  // A 20 x 6 m hall, a lane along its middle. From the recording, the robot keeps 0.95-1 m from
  // the walls (the lane's ends): the floor it can reach is about 18 x 4 m, and the lane's views
  // cover the 2 m wide band along it.
  const OccupancyGrid hall = room(20.0, 6.0);
  const auto east = lane(1.0, 19.0, 3.0, 0.0);
  const CoverageMap one_way = landfall::measureCoverage(hall, east, CoverageParams{});
  EXPECT_GE(one_way.clearance, 0.95 - 1e-9);
  EXPECT_LE(one_way.clearance, 1.0 + 1e-9);
  EXPECT_NEAR(one_way.step, 0.25, 1e-9);
  EXPECT_NEAR(one_way.reachable_area, 18.0 * 4.0, 4.0);
  EXPECT_NEAR(one_way.any_direction, 0.5, 0.06);
  EXPECT_EQ(one_way.two_directions, 0.0);
  EXPECT_NEAR(one_way.headings, one_way.any_direction / 8.0, 1e-12);
  ASSERT_EQ(one_way.sectors.size(), one_way.positions.size());
  for (std::size_t p = 0; p < one_way.positions.size(); ++p) {
    const uint8_t mask = one_way.sectors[p];
    EXPECT_TRUE(mask == 0 || mask == (1U << 4)) << static_cast<int>(mask);
    // Covered exactly where a view is within 1 m (the lane's views are 0.5 m apart).
    const bool near = std::abs(one_way.positions[p].y() - 3.0) <= 0.95;
    if (near) {
      EXPECT_NE(mask, 0) << one_way.positions[p].transpose();
    }
  }

  const CoverageMap both_ways =
    landfall::measureCoverage(hall, joined(east, lane(1.0, 19.0, 3.0, M_PI)), CoverageParams{});
  EXPECT_EQ(both_ways.positions.size(), one_way.positions.size());
  EXPECT_EQ(both_ways.any_direction, one_way.any_direction);
  EXPECT_EQ(both_ways.two_directions, both_ways.any_direction);
  EXPECT_NEAR(both_ways.headings, 2.0 * one_way.headings, 1e-12);

  // The report measures again with the clearance the build wrote to its JSON, rounded in the tenth
  // digit (up, say): the same floor. Column 22 (x = 1.1 m), a measured position, lies exactly that
  // far from the west wall.
  CoverageParams exact;
  exact.clearance = 22 * hall.resolution;
  CoverageParams rounded;
  rounded.clearance = exact.clearance * (1.0 + 1e-10);
  const CoverageMap built = landfall::measureCoverage(hall, east, exact);
  const CoverageMap reported = landfall::measureCoverage(hall, east, rounded);
  ASSERT_TRUE(anyPositionIn(built, 1.1, 0.0, 1.15, 6.0));
  EXPECT_EQ(reported.positions.size(), built.positions.size());
  EXPECT_EQ(reported.any_direction, built.any_direction);
}

TEST(Coverage, ViewsWithTooFewLandmarksDontCount)
{
  const OccupancyGrid hall = room(20.0, 6.0);
  const auto blank = lane(1.0, 19.0, 3.0, 0.0, 39);
  const CoverageMap map = landfall::measureCoverage(hall, blank, CoverageParams{});
  // The robot drove there, so the floor is reachable, but its views can't localize anything.
  EXPECT_GT(map.reachable_area, 60.0);
  EXPECT_EQ(map.any_direction, 0.0);
  CoverageParams lenient;
  lenient.min_landmarks = 39;
  EXPECT_GT(landfall::measureCoverage(hall, blank, lenient).any_direction, 0.4);
}

TEST(Coverage, TheClearanceDecidesWhatTheRobotCanReach)
{
  // Two 10 x 10 m rooms joined by a door 1 m wide; the robot drove in the west room only.
  OccupancyGrid rooms = room(20.0, 10.0);
  fill(&rooms, 9.95, 0.0, 10.05, 4.5, landfall::kCellOccupied);
  fill(&rooms, 9.95, 5.5, 10.05, 10.0, landfall::kCellOccupied);
  const auto west = lane(1.0, 8.0, 5.0, 0.0);
  CoverageParams narrow_robot;
  narrow_robot.clearance = 0.3;
  const CoverageMap through = landfall::measureCoverage(rooms, west, narrow_robot);
  EXPECT_NEAR(through.clearance, 0.3, 1e-12);
  EXPECT_TRUE(anyPositionIn(through, 11.0, 1.0, 19.0, 9.0));  // the east room
  CoverageParams wide_robot;
  wide_robot.clearance = 0.6;
  const CoverageMap stopped = landfall::measureCoverage(rooms, west, wide_robot);
  EXPECT_FALSE(anyPositionIn(stopped, 10.0, 0.0, 20.0, 10.0));
  EXPECT_TRUE(anyPositionIn(stopped, 1.0, 1.0, 9.0, 9.0));
  EXPECT_GT(through.reachable_area, 2.0 * stopped.reachable_area);
  // No position is ever closer to a wall than the clearance (one cell of slack).
  for (const auto & p : stopped.positions) {
    EXPECT_GE(std::min({p.x(), p.y(), 20.0 - p.x(), 10.0 - p.y()}), 0.6 - 0.05);
  }
}

TEST(Coverage, UnknownFloorAndViewsOffTheMapAreLeftOut)
{
  OccupancyGrid hall = room(20.0, 6.0);
  fill(&hall, 10.0, 0.05, 19.95, 5.95, landfall::kCellUnknown);
  auto views = lane(1.0, 19.0, 3.0, 0.0);
  views.push_back({{-5.0, 3.0, 0.0}, 100});  // off the map
  views.push_back({{30.0, 3.0, 0.0}, 100});
  const CoverageMap map = landfall::measureCoverage(hall, views, CoverageParams{});
  EXPECT_FALSE(map.positions.empty());
  EXPECT_FALSE(anyPositionIn(map, 10.0, 0.0, 20.0, 6.0));
}

TEST(Coverage, WithoutViewsOrObstacles)
{
  const OccupancyGrid hall = room(20.0, 6.0);
  const CoverageMap none = landfall::measureCoverage(hall, {}, CoverageParams{});
  EXPECT_TRUE(none.positions.empty());
  EXPECT_EQ(none.reachable_area, 0.0);
  EXPECT_EQ(none.any_direction, 0.0);

  // A grid without obstacles: every free cell is reachable.
  OccupancyGrid open = room(10.0, 10.0);
  open.data.assign(open.data.size(), landfall::kCellFree);
  const CoverageMap map =
    landfall::measureCoverage(open, lane(1.0, 9.0, 5.0, 0.0), CoverageParams{});
  EXPECT_NEAR(map.clearance, open.resolution, 1e-12);
  EXPECT_NEAR(map.reachable_area, 100.0, 1e-9);
}
