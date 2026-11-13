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

#include <Eigen/Eigenvalues>
#include <cmath>
#include <vector>

#include "landfall/grid/scan_matcher.hpp"
#include "synthetic_map.hpp"

using landfall::LikelihoodField;
using landfall::Mat3;
using landfall::OccupancyGrid;
using landfall::Pose2;
using landfall::Pose3;
using landfall::ScanMatchParams;
using landfall::Vec2;
using landfall::Vec3;

namespace
{

double deg(double degrees) { return degrees * M_PI / 180.0; }

// An unknown grid of `width` x `height` metres (5 cm cells) with its corner at (0, 0).
OccupancyGrid emptyGrid(double width, double height)
{
  OccupancyGrid grid;
  grid.resolution = 0.05;
  grid.width = static_cast<int>(std::lround(width / grid.resolution));
  grid.height = static_cast<int>(std::lround(height / grid.resolution));
  grid.origin = {0.0, 0.0, 0.0};
  grid.data.assign(static_cast<std::size_t>(grid.width) * grid.height, -1);
  return grid;
}

Pose3 laserMount()
{
  Pose3 T = Pose3::Identity();
  T.translation() = Vec3(0.1, 0.0, 0.4);
  return T;
}

// Endpoints in the base frame of a level scan (what ScanVerifier::project gives on a flat floor).
std::vector<Vec2> endpoints(const landfall::PlanarScan & scan)
{
  std::vector<Vec2> points;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r >= scan.range_max) {
      continue;
    }
    const double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    points.emplace_back(
      (scan.T_base_laser * Vec3(r * std::cos(a), r * std::sin(a), 0.0)).head<2>());
  }
  return points;
}

std::vector<Vec2> scanAt(const OccupancyGrid & grid, const Pose2 & truth, int seed)
{
  return endpoints(landfall::test::simulateScan(
    grid, landfall::toPose3(truth), laserMount(), seed, 360, 12.0, 0.01));
}

}  // namespace

TEST(ScanMatcher, RecoversAWrongGuessInARoom)
{
  OccupancyGrid grid = emptyGrid(12.0, 8.0);
  landfall::test::addRoomToGrid(&grid, 1.0, 1.0, 11.0, 7.0);
  // A pillar, so the room isn't symmetric.
  landfall::test::addRoomToGrid(&grid, 7.0, 2.0, 7.6, 2.6);
  const LikelihoodField field(grid);
  const Pose2 truth{3.2, 2.7, deg(20.0)};
  const auto points = scanAt(grid, truth, 1);
  const Pose2 guess{truth.x + 0.4, truth.y - 0.3, truth.yaw + deg(4.0)};
  const auto match = landfall::matchScan(field, points, guess, ScanMatchParams());
  ASSERT_TRUE(match.valid) << match.reason;
  EXPECT_LT(landfall::translationDistance(match.pose, truth), 0.02);
  EXPECT_LT(landfall::rotationDistance(match.pose, truth), deg(0.3));
  EXPECT_GT(match.hit_ratio, 0.9);
  EXPECT_LT(match.guess_hit_ratio, 0.5);
  // Walls on every side: a few centimetres in every direction, well under a degree.
  EXPECT_LT(std::sqrt(match.covariance(0, 0)), 0.05);
  EXPECT_LT(std::sqrt(match.covariance(1, 1)), 0.05);
  EXPECT_LT(std::sqrt(match.covariance(2, 2)), deg(1.0));
}

TEST(ScanMatcher, ReportsALongCorridorAsUncertainAlongItOnly)
{
  // 2 m wide, 40 m long, its ends far beyond the 12 m beams.
  OccupancyGrid grid = emptyGrid(44.0, 6.0);
  landfall::test::addRoomToGrid(&grid, 2.0, 2.0, 42.0, 4.0);
  const LikelihoodField field(grid);
  const Pose2 truth{20.0, 3.1, deg(2.0)};
  const auto points = scanAt(grid, truth, 2);
  const Pose2 guess{truth.x + 0.5, truth.y + 0.2, truth.yaw - deg(3.0)};
  const auto match = landfall::matchScan(field, points, guess, ScanMatchParams());
  ASSERT_TRUE(match.valid) << match.reason;
  // Across the corridor and in heading the scan decides ...
  EXPECT_LT(std::abs(match.pose.y - truth.y), 0.02);
  EXPECT_LT(landfall::rotationDistance(match.pose, truth), deg(0.3));
  // ... along it nothing does: the match stays near the guess, and says so.
  EXPECT_LT(std::abs(match.pose.x - guess.x), 0.2);
  const double along = std::sqrt(match.covariance(0, 0));
  const double across = std::sqrt(match.covariance(1, 1));
  RecordProperty("std_along_m", std::to_string(along));
  RecordProperty("std_across_m", std::to_string(across));
  EXPECT_GT(along, 1.0);
  EXPECT_LT(across, 0.05);
}

TEST(ScanMatcher, ToleratesUnmappedObjects)
{
  OccupancyGrid grid = emptyGrid(12.0, 8.0);
  landfall::test::addRoomToGrid(&grid, 1.0, 1.0, 11.0, 7.0);
  landfall::test::addRoomToGrid(&grid, 7.0, 2.0, 7.6, 2.6);
  const LikelihoodField field(grid);
  const Pose2 truth{4.0, 4.5, deg(-30.0)};
  // A pallet the map doesn't know, 1 m in front of the robot: blocks a quarter of the beams.
  OccupancyGrid with_pallet = grid;
  landfall::test::addRoomToGrid(&with_pallet, 4.6, 3.2, 5.6, 4.2);
  const auto points = scanAt(with_pallet, truth, 3);
  const Pose2 guess{truth.x - 0.25, truth.y + 0.25, truth.yaw + deg(2.0)};
  const auto match = landfall::matchScan(field, points, guess, ScanMatchParams());
  ASSERT_TRUE(match.valid) << match.reason;
  EXPECT_LT(landfall::translationDistance(match.pose, truth), 0.03);
  EXPECT_LT(landfall::rotationDistance(match.pose, truth), deg(0.4));
}

TEST(ScanMatcher, RefusesWhatItCannotMatch)
{
  OccupancyGrid grid = emptyGrid(12.0, 8.0);
  landfall::test::addRoomToGrid(&grid, 1.0, 1.0, 11.0, 7.0);
  landfall::test::addRoomToGrid(&grid, 7.0, 2.0, 7.6, 2.6);
  const LikelihoodField field(grid);
  const Pose2 truth{3.0, 4.0, 0.0};
  const auto points = scanAt(grid, truth, 4);

  // Too few beams.
  const std::vector<Vec2> few(points.begin(), points.begin() + 20);
  const auto sparse = landfall::matchScan(field, few, truth, ScanMatchParams());
  EXPECT_FALSE(sparse.valid);
  EXPECT_NE(sparse.reason.find("beams"), std::string::npos) << sparse.reason;

  // The truth is 3 m from the guess, outside the 1 m window: no good fit anywhere inside it.
  const auto far = landfall::matchScan(field, points, {6.0, 4.0, 0.0}, ScanMatchParams());
  EXPECT_FALSE(far.valid);
  EXPECT_FALSE(far.reason.empty());
}

TEST(ScanMatcher, ReportsASecondPlaceTheScanFitsAlike)
{
  // A 2 m corridor whose only feature along it is a small pillar against its north wall. The map
  // draws the pillar a second time 0.4 m on (as a SLAM map can draw a wall twice): the scan fits
  // there as well, so the matcher reports that place and how close its fit is.
  const auto pillar = [](OccupancyGrid * grid, double x0) {
    for (int row = 0; row < grid->height; ++row) {
      for (int col = 0; col < grid->width; ++col) {
        const Vec2 c = grid->cellCenter(col, row);
        if (c.x() >= x0 && c.x() <= x0 + 0.2 && c.y() >= 3.6 && c.y() <= 3.95) {
          grid->data[static_cast<std::size_t>(row) * grid->width + col] = landfall::kCellOccupied;
        }
      }
    }
  };
  OccupancyGrid world = emptyGrid(44.0, 6.0);
  landfall::test::addRoomToGrid(&world, 2.0, 2.0, 42.0, 4.0);
  pillar(&world, 20.0);
  OccupancyGrid doubled = world;
  pillar(&doubled, 20.4);
  const LikelihoodField once(world);
  const LikelihoodField twice(doubled);
  for (int seed = 1; seed <= 3; ++seed) {
    const Pose2 truth{18.5 + 0.3 * seed, 2.9, 0.03 * seed};
    const auto points = scanAt(world, truth, seed);
    const Pose2 guess{truth.x + 0.1, truth.y, truth.yaw};
    const auto clear = landfall::matchScan(once, points, guess, ScanMatchParams());
    ASSERT_TRUE(clear.valid) << clear.reason;
    EXPECT_LT(landfall::translationDistance(clear.pose, truth), 0.05);
    if (clear.alternative) {  // other places fit clearly worse
      EXPECT_GT(clear.alternative_nats, 2.0) << seed;
    }
    const auto alike = landfall::matchScan(twice, points, guess, ScanMatchParams());
    ASSERT_TRUE(alike.valid) << alike.reason;
    ASSERT_TRUE(alike.alternative.has_value()) << seed;
    EXPECT_LT(std::abs(alike.alternative_nats), 0.5) << seed;
    EXPECT_NEAR(std::abs(alike.alternative->x - alike.pose.x), 0.4, 0.05) << seed;
    EXPECT_NEAR(alike.alternative->y, alike.pose.y, 0.02) << seed;
    // One of the two is the true place.
    EXPECT_LT(
      std::min(
        landfall::translationDistance(alike.pose, truth),
        landfall::translationDistance(*alike.alternative, truth)),
      0.05)
      << seed;
  }
}
