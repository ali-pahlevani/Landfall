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
#include <limits>
#include <memory>
#include <random>

#include "landfall/grid/scan_verifier.hpp"

using landfall::LikelihoodField;
using landfall::OccupancyGrid;
using landfall::Pose2;
using landfall::Vec2;
using landfall::Vec3;

namespace
{

// 12 m x 8 m room (5 cm cells) with outer walls, an interior wall at x = 6 m from y = 0 to 5 m, a
// box, and an unknown patch in one corner.
OccupancyGrid makeRoom()
{
  OccupancyGrid grid;
  grid.resolution = 0.05;
  grid.width = 240;
  grid.height = 160;
  grid.origin = {0.0, 0.0, 0.0};
  grid.data.assign(static_cast<std::size_t>(grid.width) * grid.height, landfall::kCellFree);
  const auto set = [&](int c0, int r0, int c1, int r1, int8_t value) {
    for (int r = r0; r <= r1; ++r) {
      for (int c = c0; c <= c1; ++c) {
        grid.data[static_cast<std::size_t>(r) * grid.width + c] = value;
      }
    }
  };
  set(0, 0, 239, 1, 100);
  set(0, 158, 239, 159, 100);
  set(0, 0, 1, 159, 100);
  set(238, 0, 239, 159, 100);
  set(120, 0, 121, 100, 100);   // interior wall, x = 6.0 m, y in [0, 5] m
  set(40, 100, 60, 120, 100);   // box, x in [2, 3] m, y in [5, 6] m
  set(200, 120, 237, 157, -1);  // unknown corner
  return grid;
}

// Ray-marched scan from the true pose with 1 cm Gaussian noise; 270 degree field of view.
landfall::PlanarScan simulateScan(const LikelihoodField & field, const Pose2 & pose)
{
  landfall::PlanarScan scan;
  scan.angle_min = -0.75 * M_PI;
  scan.angle_increment = (1.5 * M_PI) / 539.0;
  scan.range_min = 0.1;
  scan.range_max = 20.0;
  scan.T_base_laser.translation() = Vec3(0.2, 0.0, 0.3);
  std::mt19937 rng(3);
  std::normal_distribution<double> noise(0.0, 0.01);
  const Vec2 origin(pose.x + 0.2 * std::cos(pose.yaw), pose.y + 0.2 * std::sin(pose.yaw));
  for (int i = 0; i < 540; ++i) {
    const double angle = pose.yaw + scan.angle_min + i * scan.angle_increment;
    float range = std::numeric_limits<float>::infinity();
    for (int step = 10; step * 0.005 < scan.range_max; ++step) {
      const double r = step * 0.005;
      const auto cell =
        field.classify(origin.x() + r * std::cos(angle), origin.y() + r * std::sin(angle));
      if (cell == LikelihoodField::Cell::kOccupied) {
        range = static_cast<float>(r + noise(rng));
        break;
      }
      if (cell == LikelihoodField::Cell::kOutside) {
        break;
      }
    }
    scan.ranges.push_back(range);
  }
  return scan;
}

class ScanVerifierTest : public ::testing::Test
{
protected:
  ScanVerifierTest()
  : field_(std::make_shared<LikelihoodField>(makeRoom())),
    verifier_(field_, landfall::ScanCheckParams())
  {
  }

  std::shared_ptr<LikelihoodField> field_;
  landfall::ScanVerifier verifier_;
  const Vec3 flat_ = Vec3(0, 0, -1);
};

}  // namespace

TEST_F(ScanVerifierTest, LikelihoodFieldDistances)
{
  // 0.5 m right of the interior wall's inner face (x = 6.1 m).
  EXPECT_NEAR(field_->distance(6.6, 2.5), 0.5, 0.05);
  EXPECT_DOUBLE_EQ(field_->distance(6.025, 2.5), 0.0);
  EXPECT_DOUBLE_EQ(field_->distance(4.0, 3.0), 1.0);  // capped
  EXPECT_EQ(field_->classify(11.0, 7.0), LikelihoodField::Cell::kUnknown);
  EXPECT_EQ(field_->classify(-1.0, 7.0), LikelihoodField::Cell::kOutside);
  EXPECT_TRUE(field_->segmentHitsObstacle(Vec2(5.0, 2.0), Vec2(7.0, 2.5)));
  EXPECT_FALSE(field_->segmentHitsObstacle(Vec2(5.0, 6.0), Vec2(7.0, 6.5)));
}

TEST_F(ScanVerifierTest, TruePoseIsAccepted)
{
  for (const Pose2 & truth : {Pose2{3.0, 3.0, 0.3}, Pose2{9.0, 2.0, 2.5}, Pose2{4.5, 7.0, -1.2}}) {
    const auto scan = verifier_.project(simulateScan(*field_, truth), flat_);
    const auto result = verifier_.verify(scan, truth);
    EXPECT_TRUE(result.passed) << result.reason;
    EXPECT_GE(result.score.hit_ratio, 0.9);
    EXPECT_EQ(result.score.violations, 0);
    EXPECT_LE(landfall::translationDistance(result.best_pose, truth), 0.05);
  }
}

TEST_F(ScanVerifierTest, SmallErrorIsRefinedWithinTheWindow)
{
  const Pose2 truth{3.0, 3.0, 0.3};
  const auto scan = verifier_.project(simulateScan(*field_, truth), flat_);
  const auto result = verifier_.verify(scan, Pose2{3.06, 2.95, 0.3 + 2.0 * M_PI / 180.0});
  ASSERT_TRUE(result.passed) << result.reason;
  EXPECT_LE(landfall::translationDistance(result.best_pose, truth), 0.04);
  EXPECT_LE(landfall::rotationDistance(result.best_pose, truth), 1.1 * M_PI / 180.0);
}

TEST_F(ScanVerifierTest, OffsetPosesAreRejected)
{
  const Pose2 truth{3.0, 3.0, 0.3};
  const auto scan = verifier_.project(simulateScan(*field_, truth), flat_);
  for (const Pose2 & wrong :
       {Pose2{3.5, 3.0, 0.3}, Pose2{3.0, 2.5, 0.3}, Pose2{3.35, 3.35, 0.3}, Pose2{3.0, 3.0, 0.5}}) {
    const auto result = verifier_.verify(scan, wrong);
    EXPECT_FALSE(result.passed) << wrong.x << " " << wrong.y << " " << wrong.yaw;
  }
}

TEST_F(ScanVerifierTest, SeeingThroughAWallIsAViolation)
{
  // The robot really stands at x = 8 m looking along -x, with a long clear view to the left wall
  // above the interior wall's end (y = 6.5 m). Placed at y = 4 m instead, the same long beams
  // would pass through the interior wall.
  const Pose2 truth{8.0, 6.5, M_PI};
  const auto scan = verifier_.project(simulateScan(*field_, truth), flat_);
  const auto score = verifier_.score(scan, Pose2{8.0, 4.0, M_PI}, true);
  EXPECT_GT(score.violation_ratio, 0.05);
  EXPECT_FALSE(verifier_.verify(scan, Pose2{8.0, 4.0, M_PI}).passed);
  EXPECT_EQ(verifier_.score(scan, truth, true).violations, 0);
}

TEST_F(ScanVerifierTest, TooFewBeamsFail)
{
  landfall::PlanarScan scan = simulateScan(*field_, Pose2{3.0, 3.0, 0.0});
  scan.ranges.resize(40);
  const auto result = verifier_.verify(verifier_.project(scan, flat_), Pose2{3.0, 3.0, 0.0});
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.reason.find("valid beams"), std::string::npos) << result.reason;
}

TEST_F(ScanVerifierTest, RampTiltDropsFloorHits)
{
  const landfall::PlanarScan scan = simulateScan(*field_, Pose2{3.0, 3.0, 0.0});
  // Nose down by 6 degrees: forward beams from 0.3 m height reach the floor after ~2.9 m.
  const double pitch = 6.0 * M_PI / 180.0;
  const Vec3 gravity_base(std::sin(pitch), 0.0, -std::cos(pitch));
  const auto tilted = verifier_.project(scan, gravity_base);
  const auto flat = verifier_.project(scan, flat_);
  EXPECT_EQ(flat.dropped_floor, 0);
  EXPECT_GT(tilted.dropped_floor, 10);
  EXPECT_EQ(
    static_cast<int>(tilted.endpoints.size()) + tilted.dropped_floor,
    static_cast<int>(flat.endpoints.size()));
}
