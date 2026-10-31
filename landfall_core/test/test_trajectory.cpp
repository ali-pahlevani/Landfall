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
#include <random>
#include <vector>

#include "landfall/eval/trajectory.hpp"

using landfall::Pose2;
using landfall::eval::Trajectory;

TEST(Trajectory, InterpolatesAndRespectsGaps)
{
  Trajectory t;
  t.add(1000, {0.0, 0.0, 3.0});
  t.add(2000, {1.0, 2.0, -3.0});  // across the +-pi seam
  t.add(5000, {4.0, 2.0, 0.0});
  const auto mid = t.at(1500, 1000);
  ASSERT_TRUE(mid.has_value());
  EXPECT_NEAR(mid->x, 0.5, 1e-12);
  EXPECT_NEAR(mid->y, 1.0, 1e-12);
  EXPECT_NEAR(std::abs(mid->yaw), M_PI, 1e-9);  // shorter arc, not through 0
  EXPECT_TRUE(t.at(2000, 0).has_value());
  EXPECT_FALSE(t.at(3000, 1000).has_value());  // 3 us gap
  EXPECT_TRUE(t.at(3000, 3000).has_value());
  EXPECT_FALSE(t.at(999, 1000).has_value());
  EXPECT_FALSE(t.at(5001, 1000).has_value());
}

TEST(Trajectory, AlignmentRecoversTheTransformAndStats)
{
  const Pose2 transform{8.0, -0.5, 0.2};
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-10.0, 10.0);
  std::normal_distribution<double> noise(0.0, 0.01);
  std::vector<Pose2> truth;
  std::vector<Pose2> estimates;
  for (int i = 0; i < 200; ++i) {
    truth.push_back({u(rng), u(rng), u(rng) * 0.3});
    Pose2 e = landfall::compose(transform, truth.back());
    e.x += noise(rng);
    e.y += noise(rng);
    estimates.push_back(e);
  }
  const Pose2 fitted = landfall::eval::alignSE2(truth, estimates);
  EXPECT_NEAR(fitted.x, 8.0, 0.01);
  EXPECT_NEAR(fitted.y, -0.5, 0.01);
  EXPECT_NEAR(fitted.yaw, 0.2, 0.002);
  std::vector<double> position;
  std::vector<double> yaw;
  landfall::eval::poseErrors(truth, estimates, fitted, &position, &yaw);
  const auto stats = landfall::eval::errorStats(position);
  EXPECT_EQ(stats.count, 200U);
  EXPECT_LT(stats.median, 0.02);
  EXPECT_LE(stats.median, stats.p95);
  EXPECT_LE(stats.p95, stats.max);
  EXPECT_THROW(landfall::eval::alignSE2({truth[0]}, {estimates[0]}), std::invalid_argument);
  EXPECT_EQ(landfall::eval::errorStats({}).count, 0U);
}
