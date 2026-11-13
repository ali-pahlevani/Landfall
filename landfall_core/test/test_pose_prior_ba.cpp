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
    BaKeyframe keyframe;
    keyframe.prior = {t.x + xy(rng), t.y + xy(rng), t.yaw + yaw(rng)};
    keyframe.covariance = covariance;
    keyframe.T_base_cam = T_base_cam;
    keyframe.focal_px = landfall::test::kFocal;
    problem.keyframes.push_back(keyframe);
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
  // BA moves each keyframe toward its truth, so a keyframe's shift is about its prior's error: the
  // worst prior of 64 at 1 cm per axis is 2.8 cm off. Bounds relative to the drawn priors hold on
  // every platform (rounding moves the worst shift across a fixed 3 cm: 2.80 cm on x86, 3.11 cm on
  // arm64).
  double max_shift = 0.0;
  double max_prior_error = 0.0;
  double max_error = 0.0;
  for (std::size_t k = 0; k < result.poses.size(); ++k) {
    max_shift = std::max(
      max_shift, landfall::translationDistance(result.poses[k], problem.keyframes[k].prior));
    max_prior_error = std::max(
      max_prior_error, landfall::translationDistance(problem.keyframes[k].prior, problem.truth[k]));
    max_error =
      std::max(max_error, landfall::translationDistance(result.poses[k], problem.truth[k]));
  }
  EXPECT_LT(max_shift, max_prior_error + 0.01);
  EXPECT_LT(max_error, 0.02);  // 1.5 cm on x86: closer to the truth than the worst prior
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

namespace
{

// U with U^T U = diag(std)^-2.
Mat3 sqrtInformation(double std_x, double std_y, double std_yaw)
{
  return Vec3(1.0 / std_x, 1.0 / std_y, 1.0 / std_yaw).asDiagonal();
}

// A 15 m corridor driven along x, a keyframe every 0.5 m. The scan pins every keyframe across the
// corridor and in heading, but only the two at its ends along it; the priors drift along it by
// up to 0.6 m in the middle (as AMCL does in a long aisle). Odometry: exact up to 2 mm.
std::vector<BaKeyframe> corridor(std::vector<Pose2> * truth, int seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> mm(0.0, 0.002);
  std::vector<BaKeyframe> keyframes;
  const int n = 31;
  for (int k = 0; k < n; ++k) {
    const Pose2 t{0.5 * k, 0.0, 0.0};
    truth->push_back(t);
    const bool end = k == 0 || k == n - 1;
    BaKeyframe kf;
    kf.prior = {t.x + 0.6 * std::sin(M_PI * k / (n - 1)), t.y + mm(rng), t.yaw};
    kf.prior_sqrt_information = sqrtInformation(end ? 0.01 : 2.0, 0.01, deg(0.2));
    // Odometry in its own frame (offset and turned by 0.3 rad), the same motion with 2 mm noise.
    const double c = std::cos(0.3);
    const double s = std::sin(0.3);
    const double along = t.x + mm(rng);
    kf.odom = Pose2{10.0 + c * along, -5.0 + s * along, 0.3};
    keyframes.push_back(kf);
  }
  return keyframes;
}

}  // namespace

TEST(PosePriorBa, OdometryCarriesPosesWhereThePriorsAreWeak)
{
  std::vector<Pose2> truth;
  const auto keyframes = corridor(&truth, 5);
  const auto params = landfall::PosePriorBaParams();
  const auto graph = landfall::adjustPosePriors(keyframes, {}, {}, params);
  ASSERT_TRUE(graph.usable) << graph.message;
  EXPECT_EQ(graph.odometry_links, static_cast<int>(keyframes.size()) - 1);
  double worst = 0.0;
  for (std::size_t k = 0; k < truth.size(); ++k) {
    worst = std::max(worst, landfall::translationDistance(graph.poses[k], truth[k]));
    EXPECT_EQ(graph.at_bound[k], 0) << k;
  }
  RecordProperty("worst_error_m", std::to_string(worst));
  EXPECT_LT(worst, 0.05);

  // Without odometry the middle keeps its drift.
  auto unlinked = keyframes;
  for (auto & k : unlinked) {
    k.odom.reset();
  }
  const auto alone = landfall::adjustPosePriors(unlinked, {}, {}, params);
  ASSERT_TRUE(alone.usable);
  EXPECT_EQ(alone.odometry_links, 0);
  EXPECT_GT(landfall::translationDistance(alone.poses[15], truth[15]), 0.5);
}

TEST(PosePriorBa, OdometrySlipDoesNotOverrideStrongPriors)
{
  // A loop the scan pins down everywhere (2 cm, 0.5 deg); one odometry step slips by 0.5 m.
  const auto truth = landfall::test::loopBothWays(0, 0, 10, 6, 0.5);
  std::vector<BaKeyframe> keyframes;
  Pose2 odom{0.0, 0.0, 0.0};
  for (std::size_t k = 0; k < truth.size(); ++k) {
    if (k > 0) {
      odom = landfall::compose(odom, landfall::between(truth[k - 1], truth[k]));
      if (k == 20) {
        odom.x += 0.5;
      }
    }
    BaKeyframe kf;
    kf.prior = truth[k];
    kf.prior_sqrt_information = sqrtInformation(0.02, 0.02, deg(0.5));
    kf.odom = odom;
    keyframes.push_back(kf);
  }
  const auto result = landfall::adjustPosePriors(keyframes, {}, {}, landfall::PosePriorBaParams());
  ASSERT_TRUE(result.usable) << result.message;
  for (std::size_t k = 0; k < truth.size(); ++k) {
    EXPECT_LT(landfall::translationDistance(result.poses[k], truth[k]), 0.03) << k;
  }
}

TEST(PosePriorBa, ASlippingTurnCountsLessThanAStraightStep)
{
  // An aisle driven east, a U-turn at its end, the aisle driven back west. The scan pins the east
  // leg (1 cm) but the west leg only loosely along the aisle (0.2 m, as where the scan fits two
  // places); the U-turn's odometry slips 0.5 m, as a skid-steer robot's did in the URDF Maker
  // warehouse. Counted like a straight step, that slip drags the west leg; with the per-radian
  // term its pull is a third.
  std::vector<Pose2> truth;
  truth.reserve(16);
  for (int i = 0; i < 8; ++i) {
    truth.push_back({0.5 * i, 0.0, 0.0});
  }
  truth.push_back({3.7, 0.3, M_PI});
  for (int i = 1; i < 8; ++i) {
    truth.push_back({3.7 - 0.5 * i, 0.3, M_PI});
  }
  std::vector<BaKeyframe> keyframes;
  Pose2 odom{0.0, 0.0, 0.0};
  for (std::size_t k = 0; k < truth.size(); ++k) {
    if (k > 0) {
      odom = landfall::compose(odom, landfall::between(truth[k - 1], truth[k]));
      if (k == 8) {
        odom = landfall::compose(odom, Pose2{0.5, 0.0, 0.0});
      }
    }
    BaKeyframe kf;
    kf.prior = truth[k];
    kf.prior_sqrt_information = sqrtInformation(k < 8 ? 0.01 : 0.2, 0.01, deg(0.2));
    kf.odom = odom;
    keyframes.push_back(kf);
  }
  const auto worstWest = [&](const landfall::PosePriorBaParams & params) {
    const auto result = landfall::adjustPosePriors(keyframes, {}, {}, params);
    EXPECT_TRUE(result.usable) << result.message;
    double worst = 0.0;
    for (std::size_t k = 8; k < truth.size(); ++k) {
      worst = std::max(worst, landfall::translationDistance(result.poses[k], truth[k]));
    }
    return worst;
  };
  landfall::PosePriorBaParams params;
  EXPECT_DOUBLE_EQ(params.odometry.std_xy_per_rad, 0.1);
  const double with_turn_term = worstWest(params);
  params.odometry.std_xy_per_rad = 0.0;
  const double without = worstWest(params);
  RecordProperty("west_error_with_turn_term_m", std::to_string(with_turn_term));
  RecordProperty("west_error_without_m", std::to_string(without));
  EXPECT_LT(with_turn_term, 0.035);  // measured 2.0 cm
  EXPECT_GT(without, 0.05);          // measured 6.6 cm
}

TEST(OdometryCheck, MeasuresWhatTurningAddsBeyondAStraightStep)
{
  // Steps of 0.5 m, every fifth turning 20 to 90 degrees; the reference (scan matches) carries
  // 1 cm of noise per axis, the odometry 3 mm, plus `slip` metres sideways per radian turned.
  const auto measure = [](double slip) {
    std::mt19937 rng(3);
    std::normal_distribution<double> noise(0.0, 1.0);
    std::vector<landfall::OdometryStep> steps;
    for (int k = 0; k < 200; ++k) {
      const double turn = k % 5 == 0 ? deg(20.0 + 70.0 * (k % 3) / 2.0) : 0.0;
      const Pose2 truth{0.5, 0.0, turn};
      Pose2 odometry = truth;
      odometry.x += 0.003 * noise(rng);
      odometry.y += 0.003 * noise(rng) + slip * turn;
      Pose2 reference = truth;
      reference.x += 0.01 * noise(rng);
      reference.y += 0.01 * noise(rng);
      steps.push_back({odometry, reference});
    }
    return landfall::checkOdometry(steps, deg(15.0));
  };
  const auto exact = measure(0.0);
  EXPECT_EQ(exact.straight_steps, 160);
  EXPECT_EQ(exact.turns, 40);
  EXPECT_NEAR(exact.straight_std_m, 0.0104, 0.003);  // the reference's 1 cm and the odometry's 3 mm
  EXPECT_LT(exact.turn_std_per_rad, 0.02);
  // A deterministic slip s per radian along one axis reads as s / sqrt(2 ln 2) per axis.
  const auto slipping = measure(0.25);
  EXPECT_NEAR(slipping.turn_std_per_rad, 0.25 / std::sqrt(2.0 * std::log(2.0)), 0.03);
  EXPECT_NEAR(slipping.straight_std_m, exact.straight_std_m, 1e-12);
  EXPECT_EQ(landfall::checkOdometry({}, deg(15.0)).turns, 0);
}

TEST(PosePriorBa, OdometryLinksOnlyConsecutiveKeyframesOfASession)
{
  std::vector<BaKeyframe> keyframes(6);
  for (std::size_t k = 0; k < keyframes.size(); ++k) {
    keyframes[k].prior = {0.4 * static_cast<double>(k), 0.0, 0.0};
    keyframes[k].odom = keyframes[k].prior;
    keyframes[k].session = k < 3 ? 0 : 1;
  }
  keyframes[4].odom.reset();                  // no odometry: no links to 3 or 5
  keyframes[2].odom = Pose2{30.0, 0.0, 0.0};  // a 29 m jump: beyond max_step
  const auto result = landfall::adjustPosePriors(keyframes, {}, {}, landfall::PosePriorBaParams());
  ASSERT_TRUE(result.usable);
  EXPECT_EQ(result.odometry_links, 1);  // 0-1 only
}
