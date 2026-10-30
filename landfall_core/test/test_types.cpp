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

#include "landfall/types.hpp"

using landfall::Pose2;

TEST(Types, NormalizeAngleWrapsIntoHalfOpenRange)
{
  EXPECT_NEAR(landfall::normalizeAngle(3 * M_PI / 2), -M_PI / 2, 1e-12);
  EXPECT_NEAR(landfall::normalizeAngle(-3 * M_PI / 2), M_PI / 2, 1e-12);
  EXPECT_NEAR(landfall::normalizeAngle(M_PI), -M_PI, 1e-12);
  EXPECT_NEAR(landfall::normalizeAngle(0.25 + 10 * M_PI), 0.25, 1e-9);
}

TEST(Types, Pose2AlgebraMatchesPose3)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> position(-20.0, 20.0);
  std::uniform_real_distribution<double> angle(-M_PI, M_PI);
  for (int i = 0; i < 100; ++i) {
    const Pose2 a{position(rng), position(rng), angle(rng)};
    const Pose2 b{position(rng), position(rng), angle(rng)};
    const Pose2 ab = landfall::compose(a, b);
    const landfall::Pose3 ab3 = landfall::toPose3(a) * landfall::toPose3(b);
    const Pose2 projected = landfall::toPose2(ab3);
    EXPECT_NEAR(ab.x, projected.x, 1e-9);
    EXPECT_NEAR(ab.y, projected.y, 1e-9);
    EXPECT_NEAR(landfall::rotationDistance(ab, projected), 0.0, 1e-9);

    const Pose2 identity = landfall::compose(a, landfall::inverse(a));
    EXPECT_NEAR(identity.x, 0.0, 1e-9);
    EXPECT_NEAR(identity.y, 0.0, 1e-9);
    EXPECT_NEAR(identity.yaw, 0.0, 1e-12);

    const Pose2 back = landfall::compose(a, landfall::between(a, b));
    EXPECT_NEAR(landfall::translationDistance(back, b), 0.0, 1e-9);
    EXPECT_NEAR(landfall::rotationDistance(back, b), 0.0, 1e-9);
  }
}

TEST(Types, RotationAngle)
{
  const landfall::Mat3 r = Eigen::AngleAxisd(0.3, landfall::Vec3(1, 2, 3).normalized()).matrix();
  EXPECT_NEAR(landfall::rotationAngle(r), 0.3, 1e-12);
  EXPECT_NEAR(landfall::rotationAngle(landfall::Mat3::Identity()), 0.0, 1e-12);
}
