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
#include <random>
#include <set>
#include <string>
#include <vector>

#include "landfall/geometry/two_view.hpp"
#include "synthetic_scene.hpp"

using landfall::Pose3;
using landfall::TwoViewParams;
using landfall::Vec2;
using landfall::Vec3;
using landfall::test::deg;

namespace
{

struct PairData
{
  std::vector<Vec2> x1;
  std::vector<Vec2> x2;
  std::set<int> outliers;
  Pose3 T_2_1;  ///< camera 1 coordinates into camera 2
};

// Points 3..10 m ahead of camera 1, seen by camera 2 = camera 1 moved by `motion`. A fraction of
// the correspondences is replaced by random points.
PairData makePair(const Pose3 & T_2_1, int count, double outlier_ratio, double noise_px, int seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> lateral(-3.0, 3.0);
  std::uniform_real_distribution<double> depth(3.0, 10.0);
  std::uniform_real_distribution<double> image(-0.6, 0.6);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::normal_distribution<double> noise(0.0, noise_px / landfall::test::kFocal);
  PairData data;
  data.T_2_1 = T_2_1;
  while (static_cast<int>(data.x1.size()) < count) {
    const Vec3 X(lateral(rng), lateral(rng) * 0.5, depth(rng));
    const Vec3 X2 = T_2_1 * X;
    if (X2.z() < 0.5) {
      continue;
    }
    Vec2 a = X.hnormalized() + Vec2(noise(rng), noise(rng));
    Vec2 b = X2.hnormalized() + Vec2(noise(rng), noise(rng));
    if (unit(rng) < outlier_ratio) {
      b = Vec2(image(rng), image(rng));
      data.outliers.insert(static_cast<int>(data.x1.size()));
    }
    data.x1.push_back(a);
    data.x2.push_back(b);
  }
  return data;
}

TwoViewParams params()
{
  TwoViewParams p;
  p.focal_px = landfall::test::kFocal;
  return p;
}

// Geometry of one trial: rotation error and translation direction error (degrees), and how many
// true correspondences / outliers ended up as inliers.
struct TrialResult
{
  double rotation_deg = 0.0;
  double direction_deg = 0.0;
  int good_kept = 0;
  int good = 0;
  int outliers_kept = 0;
  int outliers = 0;
};

TrialResult runTrial(int trial, double outlier_ratio)
{
  Pose3 T_2_1 = Pose3::Identity();
  T_2_1.linear() = Eigen::AngleAxisd(deg(-10.0 + trial * 0.4), Vec3::UnitY()).toRotationMatrix();
  T_2_1.translation() = Vec3(0.3, 0.02, -0.5 + 0.01 * trial);
  const PairData data = makePair(T_2_1, 300, outlier_ratio, 0.7, 100 + trial);
  const auto geometry = landfall::estimateRelativePose(data.x1, data.x2, params());
  TrialResult r;
  r.good = 300 - static_cast<int>(data.outliers.size());
  r.outliers = static_cast<int>(data.outliers.size());
  if (!geometry) {
    r.rotation_deg = r.direction_deg = 180.0;
    return r;
  }
  r.rotation_deg =
    landfall::rotationAngle(geometry->R_21 * T_2_1.linear().transpose()) * 180.0 / M_PI;
  r.direction_deg =
    std::acos(std::clamp(geometry->t_21.dot(T_2_1.translation().normalized()), -1.0, 1.0)) * 180.0 /
    M_PI;
  for (int i : geometry->inliers) {
    (data.outliers.count(i) > 0 ? r.outliers_kept : r.good_kept) += 1;
  }
  return r;
}

std::string describe(int trial, const TrialResult & r)
{
  return "trial " + std::to_string(trial) + ": rotation " + std::to_string(r.rotation_deg) +
         " deg, direction " + std::to_string(r.direction_deg) + " deg, good " +
         std::to_string(r.good_kept) + "/" + std::to_string(r.good) + ", outliers " +
         std::to_string(r.outliers_kept) + "/" + std::to_string(r.outliers) + "\n";
}

}  // namespace

TEST(TwoView, RelativePosePrecisionWithoutOutliers)
{
  std::string failures;
  for (int trial = 0; trial < 20; ++trial) {
    const TrialResult r = runTrial(trial, 0.0);
    if (r.rotation_deg > 0.2 || r.direction_deg > 1.5 || r.good_kept < 0.98 * r.good) {
      failures += describe(trial, r);
    }
  }
  EXPECT_TRUE(failures.empty()) << failures;
}

// The builder uses the inliers (tracks are then re-checked against the prior poses by
// triangulation) and the rotation only for a 5 deg check against the prior; random outliers
// that fall within 4 px of their epipolar line legitimately tilt the estimate by a few tenths.
TEST(TwoView, RelativePoseClassifiesOutliers)
{
  std::string failures;
  for (int trial = 0; trial < 50; ++trial) {
    const TrialResult r = runTrial(trial, 0.4);
    if (
      r.rotation_deg > 1.0 || r.direction_deg > 6.0 || r.good_kept < 0.95 * r.good ||
      r.outliers_kept > 0.05 * r.outliers + 3) {
      failures += describe(trial, r);
    }
  }
  EXPECT_TRUE(failures.empty()) << failures;
}

TEST(TwoView, RotationOnlyModel)
{
  Pose3 T_2_1 = Pose3::Identity();
  T_2_1.linear() = Eigen::AngleAxisd(deg(20.0), Vec3::UnitY()).toRotationMatrix();
  const PairData data = makePair(T_2_1, 200, 0.3, 0.5, 7);
  const auto geometry = landfall::estimateRelativeRotation(data.x1, data.x2, params());
  ASSERT_TRUE(geometry.has_value());
  EXPECT_LT(landfall::rotationAngle(geometry->R_21 * T_2_1.linear().transpose()), deg(0.1));
  EXPECT_EQ(geometry->t_21, Vec3::Zero());
  int outliers_kept = 0;
  for (int i : geometry->inliers) {
    outliers_kept += data.outliers.count(i) > 0 ? 1 : 0;
  }
  EXPECT_LE(outliers_kept, 2);
  EXPECT_GE(geometry->inliers.size() - outliers_kept, 0.95 * (200 - data.outliers.size()));
}

TEST(TwoView, DeterministicAndRobustToDegenerateInput)
{
  Pose3 T_2_1 = Pose3::Identity();
  T_2_1.translation() = Vec3(0.5, 0.0, 0.0);
  const PairData data = makePair(T_2_1, 150, 0.3, 0.5, 9);
  const auto a = landfall::estimateRelativePose(data.x1, data.x2, params());
  const auto b = landfall::estimateRelativePose(data.x1, data.x2, params());
  ASSERT_TRUE(a && b);
  EXPECT_EQ(a->inliers, b->inliers);
  EXPECT_EQ(a->R_21, b->R_21);

  EXPECT_FALSE(landfall::estimateRelativePose({Vec2(0, 0)}, {Vec2(0, 0)}, params()).has_value());
  EXPECT_FALSE(landfall::estimateRelativeRotation({Vec2(0, 0)}, {Vec2(0, 0)}, params()));
  EXPECT_THROW(landfall::estimateRelativePose({Vec2(0, 0)}, {}, params()), std::invalid_argument);
}
