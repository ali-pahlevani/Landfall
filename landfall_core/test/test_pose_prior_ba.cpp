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
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "landfall/build/pose_prior_ba.hpp"
#include "synthetic_map.hpp"

using landfall::BaKeyframe;
using landfall::BaObservation;
using landfall::Mat3;
using landfall::Pose2;
using landfall::Vec2;
using landfall::Vec3;
using landfall::test::deg;

namespace
{

struct Problem
{
  std::vector<Pose2> truth;
  std::vector<BaKeyframe> keyframes;
  std::vector<Vec3> landmarks;  ///< initial guesses
  std::vector<Vec3> true_landmarks;
  std::vector<BaObservation> observations;
};

Problem makeProblem(double prior_sigma_xy, double prior_sigma_yaw, int seed)
{
  Problem problem;
  const auto room = landfall::test::makeRoom(1500, -3, -3, 13, 9, seed);
  const auto camera = landfall::test::syntheticCamera();
  problem.truth = landfall::test::loopBothWays(0, 0, 10, 6, 1.0);
  std::mt19937 rng(seed);
  std::normal_distribution<double> xy(0.0, prior_sigma_xy);
  std::normal_distribution<double> yaw(0.0, prior_sigma_yaw);
  std::normal_distribution<double> pixel(0.0, 0.5 / landfall::test::kFocal);
  std::normal_distribution<double> point(0.0, 0.05);
  Mat3 covariance = Mat3::Zero();
  covariance(0, 0) = covariance(1, 1) = prior_sigma_xy * prior_sigma_xy;
  covariance(2, 2) = prior_sigma_yaw * prior_sigma_yaw;
  const auto T_base_cam = landfall::test::baseToCamera();
  std::vector<int> seen(room.landmarks.size(), 0);
  std::vector<std::vector<std::pair<int, Vec2>>> per_landmark(room.landmarks.size());
  for (std::size_t k = 0; k < problem.truth.size(); ++k) {
    const Pose2 & t = problem.truth[k];
    problem.keyframes.push_back(
      {{t.x + xy(rng), t.y + xy(rng), t.yaw + yaw(rng)},
       covariance,
       T_base_cam,
       landfall::test::kFocal});
    const auto T_cam_map = landfall::cameraPose(t, T_base_cam).inverse();
    for (std::size_t l = 0; l < room.landmarks.size(); ++l) {
      Vec2 px;
      if (landfall::test::projectPoint(camera, T_cam_map, room.landmarks[l], &px)) {
        per_landmark[l].emplace_back(
          static_cast<int>(k),
          (T_cam_map * room.landmarks[l]).hnormalized() + Vec2(pixel(rng), pixel(rng)));
      }
    }
  }
  for (std::size_t l = 0; l < room.landmarks.size(); ++l) {
    if (per_landmark[l].size() < 3) {
      continue;
    }
    const int index = static_cast<int>(problem.landmarks.size());
    problem.true_landmarks.push_back(room.landmarks[l]);
    problem.landmarks.emplace_back(room.landmarks[l] + Vec3(point(rng), point(rng), point(rng)));
    for (const auto & [k, x] : per_landmark[l]) {
      problem.observations.push_back({k, index, x});
    }
  }
  return problem;
}

double meanError(const std::vector<Pose2> & poses, const std::vector<Pose2> & truth)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < poses.size(); ++i) {
    sum += landfall::translationDistance(poses[i], truth[i]);
  }
  return sum / static_cast<double>(poses.size());
}

}  // namespace

TEST(PosePriorBa, RecoversPosesFromNoisyPriors)
{
  const Problem problem = makeProblem(0.04, deg(1.0), 11);
  ASSERT_GT(problem.landmarks.size(), 300U);
  landfall::PosePriorBaParams params;
  const auto result =
    landfall::adjustPosePriors(problem.keyframes, problem.landmarks, problem.observations, params);
  ASSERT_TRUE(result.usable) << result.message;
  std::vector<Pose2> priors;
  priors.reserve(problem.keyframes.size());
  for (const auto & k : problem.keyframes) {
    priors.push_back(k.prior);
  }
  const double before = meanError(priors, problem.truth);
  const double after = meanError(result.poses, problem.truth);
  RecordProperty("mean_error_before_mm", std::to_string(1000.0 * before));
  RecordProperty("mean_error_after_mm", std::to_string(1000.0 * after));
  EXPECT_LT(after, 0.5 * before);
  EXPECT_LT(result.final_cost, result.initial_cost);
  for (std::size_t k = 0; k < result.poses.size(); ++k) {
    EXPECT_LE(std::abs(result.poses[k].x - priors[k].x), params.max_shift_xy + 1e-9);
    EXPECT_LE(std::abs(result.poses[k].yaw - priors[k].yaw), deg(params.max_shift_yaw_deg) + 1e-9);
  }
}

TEST(PosePriorBa, BoundsHoldAndFlagAGrossPriorError)
{
  Problem problem = makeProblem(0.01, deg(0.2), 12);
  // One keyframe recorded 0.3 m off: BA may move it only 0.10 m and flags it.
  const int bad = 7;
  problem.keyframes[bad].prior.x += 0.3;
  landfall::PosePriorBaParams params;
  const auto result =
    landfall::adjustPosePriors(problem.keyframes, problem.landmarks, problem.observations, params);
  ASSERT_TRUE(result.usable) << result.message;
  EXPECT_EQ(result.at_bound[bad], 1);
  EXPECT_NEAR(result.poses[bad].x, problem.keyframes[bad].prior.x - 0.10, 1e-6);
  int flagged = 0;
  for (char f : result.at_bound) {
    flagged += f;
  }
  EXPECT_EQ(flagged, 1);
  // The other keyframes stay close to the truth.
  double worst = 0.0;
  for (std::size_t k = 0; k < result.poses.size(); ++k) {
    if (static_cast<int>(k) != bad) {
      worst = std::max(worst, landfall::translationDistance(result.poses[k], problem.truth[k]));
    }
  }
  EXPECT_LT(worst, 0.03);
}

TEST(PosePriorBa, ConfiguredPriorStdOverridesAPessimisticCovariance)
{
  // Priors 1 cm accurate but reported with a 10 cm std: trusting the report lets BA wander; the
  // configured std keeps the keyframes at their priors' accuracy.
  Problem problem = makeProblem(0.01, deg(0.2), 14);
  for (auto & k : problem.keyframes) {
    Mat3 pessimistic = Mat3::Zero();
    pessimistic(0, 0) = pessimistic(1, 1) = 0.1 * 0.1;
    pessimistic(2, 2) = deg(3.0) * deg(3.0);
    k.covariance = pessimistic;
  }
  landfall::PosePriorBaParams params;
  params.prior_std_xy = 0.01;
  params.prior_std_yaw_deg = 0.2;
  const auto result =
    landfall::adjustPosePriors(problem.keyframes, problem.landmarks, problem.observations, params);
  ASSERT_TRUE(result.usable) << result.message;
  double max_shift = 0.0;
  for (std::size_t k = 0; k < result.poses.size(); ++k) {
    max_shift = std::max(
      max_shift, landfall::translationDistance(result.poses[k], problem.keyframes[k].prior));
  }
  EXPECT_LT(max_shift, 0.03);
}

TEST(PosePriorBa, WithoutObservationsThePriorsStay)
{
  Problem problem = makeProblem(0.02, deg(0.5), 13);
  const auto result = landfall::adjustPosePriors(problem.keyframes, {}, {}, {});
  ASSERT_TRUE(result.usable);
  for (std::size_t k = 0; k < result.poses.size(); ++k) {
    EXPECT_NEAR(result.poses[k].x, problem.keyframes[k].prior.x, 1e-9);
  }
  EXPECT_THROW(
    landfall::adjustPosePriors(problem.keyframes, {}, {{0, 0, Vec2::Zero()}}, {}),
    std::out_of_range);
}
