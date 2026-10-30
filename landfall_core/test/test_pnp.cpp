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

#include <chrono>
#include <functional>
#include <random>
#include <string>

#include "landfall/geometry/gravity.hpp"
#include "landfall/geometry/pnp.hpp"
#include "synthetic_scene.hpp"

using landfall::Correspondences;
using landfall::Mat3;
using landfall::PnpOptions;
using landfall::Pose3;
using landfall::Vec2;
using landfall::Vec3;
using landfall::test::deg;

namespace
{

constexpr int kTrials = 1000;
constexpr double kMaxCenterError = 0.02;  // m
constexpr double kMaxRotationErrorDeg = 0.2;

struct Trial
{
  Correspondences correspondences;
  Pose3 T_cam_map;
  Vec3 gravity_cam;  // true gravity direction in the camera frame
};

// 200 correspondences, a fraction of them outliers (random pixels), 1 px noise on the inliers.
// Points are 2-8 m in front of a camera on a robot at a random pose in a 30 m x 30 m area.
Trial makeTrial(std::mt19937 & rng, double outlier_ratio, double ramp_pitch = 0.0)
{
  std::uniform_real_distribution<double> position(-15.0, 15.0);
  std::uniform_real_distribution<double> angle(-M_PI, M_PI);
  std::uniform_real_distribution<double> u(0.0, landfall::test::kWidth);
  std::uniform_real_distribution<double> v(0.0, landfall::test::kHeight);
  std::uniform_real_distribution<double> depth(2.0, 8.0);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::normal_distribution<double> noise(0.0, 1.0 / landfall::test::kFocal);

  const Pose3 T_map_base =
    landfall::test::basePose(position(rng), position(rng), angle(rng), ramp_pitch);
  const Pose3 T_map_cam = T_map_base * landfall::test::baseToCamera();
  Trial trial;
  trial.T_cam_map = T_map_cam.inverse();
  trial.gravity_cam = trial.T_cam_map.linear() * Vec3(0, 0, -1);
  const auto pixel_to_normalized = [](double px, double py) {
    return Vec2(
      (px - landfall::test::kWidth / 2.0) / landfall::test::kFocal,
      (py - landfall::test::kHeight / 2.0) / landfall::test::kFocal);
  };
  for (int i = 0; i < 200; ++i) {
    const Vec2 x = pixel_to_normalized(u(rng), v(rng));
    const Vec3 X = T_map_cam * (Vec3(x.x(), x.y(), 1.0) * depth(rng));
    Vec2 observed = x + Vec2(noise(rng), noise(rng));
    if (unit(rng) < outlier_ratio) {
      observed = pixel_to_normalized(u(rng), v(rng));
    }
    trial.correspondences.normalized.push_back(observed);
    trial.correspondences.points.push_back(X);
  }
  return trial;
}

struct Outcome
{
  int successes = 0;
  double seconds = 0.0;
};

Outcome run(
  const std::function<Trial(std::mt19937 &)> & make,
  const std::function<std::optional<landfall::PnpSolution>(const Trial &)> & solve)
{
  std::mt19937 rng(2026);
  Outcome outcome;
  for (int trial = 0; trial < kTrials; ++trial) {
    const Trial t = make(rng);
    const auto start = std::chrono::steady_clock::now();
    const auto solution = solve(t);
    outcome.seconds +=
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (
      solution &&
      landfall::test::centerError(solution->T_cam_map, t.T_cam_map) <= kMaxCenterError &&
      landfall::test::rotationError(solution->T_cam_map, t.T_cam_map) <=
        deg(kMaxRotationErrorDeg)) {
      ++outcome.successes;
    }
  }
  return outcome;
}

void report(const std::string & name, const Outcome & outcome)
{
  ::testing::Test::RecordProperty(
    name + "_success", std::to_string(outcome.successes) + "/" + std::to_string(kTrials));
  ::testing::Test::RecordProperty(
    name + "_mean_ms", std::to_string(1000.0 * outcome.seconds / kTrials));
}

PnpOptions options()
{
  PnpOptions o;
  o.focal_px = landfall::test::kFocal;
  o.threshold_px = 0.01 * landfall::test::kWidth;  // the "auto" default
  return o;
}

}  // namespace

TEST(Pnp, UprightWithHalfOutliers)
{
  const Outcome outcome = run(
    [](std::mt19937 & rng) { return makeTrial(rng, 0.5); },
    [](const Trial & t) {
      return landfall::solveUprightPnp(t.correspondences, t.gravity_cam, options());
    });
  report("upright", outcome);
  EXPECT_GE(outcome.successes, 995);
}

TEST(Pnp, P3PFallbackWhenGravityIsTwoDegreesOff)
{
  const Outcome outcome = run(
    [](std::mt19937 & rng) {
      Trial t = makeTrial(rng, 0.5);
      // The prior believes the camera is level; it is tilted by 2 degrees about a random axis.
      std::uniform_real_distribution<double> angle(-M_PI, M_PI);
      const double a = angle(rng);
      const Vec3 axis(std::cos(a), 0.0, std::sin(a));
      t.gravity_cam = Eigen::AngleAxisd(deg(2.0), axis) * t.gravity_cam;
      return t;
    },
    [](const Trial & t) {
      return landfall::solvePnp(
        landfall::PnpStrategy::kUprightThenP3P, t.correspondences, t.gravity_cam, options(), 40);
    });
  report("upright_then_p3p_tilted", outcome);
  EXPECT_GE(outcome.successes, 995);
}

TEST(Pnp, P3PAloneWithHalfOutliers)
{
  const Outcome outcome = run(
    [](std::mt19937 & rng) { return makeTrial(rng, 0.5); },
    [](const Trial & t) { return landfall::solveP3P(t.correspondences, options()); });
  report("p3p", outcome);
  EXPECT_GE(outcome.successes, 995);
}

TEST(Pnp, UprightWithImuGravityOnASixDegreeRamp)
{
  const Outcome outcome = run(
    [](std::mt19937 & rng) {
      std::uniform_real_distribution<double> sign(-1.0, 1.0);
      Trial t = makeTrial(rng, 0.5, deg(6.0) * (sign(rng) < 0.0 ? -1.0 : 1.0));
      // IMU roll/pitch error of a consumer AHRS: 0.3 degrees per axis.
      std::normal_distribution<double> imu_error(0.0, deg(0.3));
      const Mat3 error = (Eigen::AngleAxisd(imu_error(rng), Vec3::UnitX()) *
                          Eigen::AngleAxisd(imu_error(rng), Vec3::UnitZ()))
                           .toRotationMatrix();
      t.gravity_cam = error * t.gravity_cam;
      return t;
    },
    [](const Trial & t) {
      return landfall::solvePnp(
        landfall::PnpStrategy::kUprightThenP3P, t.correspondences, t.gravity_cam, options(), 40);
    });
  report("upright_ramp_imu", outcome);
  EXPECT_GE(outcome.successes, 995);
}

TEST(Pnp, OpenCvCrossCheck)
{
  std::mt19937 rng(5);
  int successes = 0;
  for (int i = 0; i < 100; ++i) {
    const Trial t = makeTrial(rng, 0.3);
    const auto solution = landfall::solveOpenCvPnp(t.correspondences, options());
    if (
      solution &&
      landfall::test::centerError(solution->T_cam_map, t.T_cam_map) <= kMaxCenterError &&
      landfall::test::rotationError(solution->T_cam_map, t.T_cam_map) <=
        deg(kMaxRotationErrorDeg)) {
      ++successes;
    }
  }
  RecordProperty("opencv_success", std::to_string(successes) + "/100");
  EXPECT_GE(successes, 95);
}

TEST(Pnp, ReportsInliersAndHandlesDegenerateInput)
{
  std::mt19937 rng(9);
  const Trial t = makeTrial(rng, 0.0);
  const auto solution = landfall::solveUprightPnp(t.correspondences, t.gravity_cam, options());
  ASSERT_TRUE(solution.has_value());
  EXPECT_EQ(solution->method, landfall::PnpMethod::kUpright);
  EXPECT_GE(solution->inliers.size(), 195U);
  EXPECT_LT(solution->mean_reprojection_px, 2.0);

  Correspondences few;
  few.normalized.assign(3, Vec2::Zero());
  few.points.assign(3, Vec3::Ones());
  EXPECT_FALSE(landfall::solveUprightPnp(few, t.gravity_cam, options()).has_value());
  EXPECT_FALSE(landfall::solveP3P(few, options()).has_value());
  EXPECT_FALSE(landfall::solveOpenCvPnp(few, options()).has_value());

  // Pure noise: no consistent pose.
  const Trial noise = makeTrial(rng, 1.0);
  const auto none = landfall::solvePnp(
    landfall::PnpStrategy::kUprightThenP3P, noise.correspondences, noise.gravity_cam, options(),
    40);
  EXPECT_TRUE(!none.has_value() || none->inliers.size() < 40U);

  EXPECT_EQ(landfall::parsePnpStrategy("upright_then_p3p"), landfall::PnpStrategy::kUprightThenP3P);
  EXPECT_FALSE(landfall::parsePnpStrategy("epnp").has_value());
}

TEST(Gravity, ImuOrientationGivesGravityInTheBaseFrame)
{
  // Base pitched nose-down by 6 degrees, IMU mounted rotated 90 degrees about z and with an
  // arbitrary yaw in its world frame: gravity in base still shows the 6 degree pitch.
  const Eigen::Quaterniond q_world_base(
    Eigen::AngleAxisd(1.1, Vec3::UnitZ()) * Eigen::AngleAxisd(deg(6.0), Vec3::UnitY()));
  const Eigen::Quaterniond q_base_imu(Eigen::AngleAxisd(M_PI / 2, Vec3::UnitZ()));
  const Eigen::Quaterniond q_world_imu = q_world_base * q_base_imu;
  const Vec3 g_base = landfall::gravityFromImu(q_world_imu, q_base_imu);
  EXPECT_TRUE(g_base.isApprox(q_world_base.conjugate() * Vec3(0, 0, -1), 1e-12));
  EXPECT_NEAR(landfall::tiltDeg(g_base), 6.0, 1e-9);
  EXPECT_NEAR(landfall::tiltDeg(landfall::flatFloorGravity()), 0.0, 1e-12);

  const Vec3 a = Vec3(0.3, -0.2, 0.9).normalized();
  EXPECT_TRUE((landfall::rotationBetween(a, -a) * a).isApprox(-a, 1e-12));
  EXPECT_TRUE((landfall::rotationBetween(a, Vec3::UnitY()) * a).isApprox(Vec3::UnitY(), 1e-12));
  EXPECT_TRUE(landfall::rotationBetween(a, a).isApprox(Mat3::Identity()));
}

TEST(Gravity, SourceSelection)
{
  landfall::GravityParams params;
  const Eigen::Quaterniond identity = Eigen::Quaterniond::Identity();
  landfall::ImuSample tilted;
  tilted.orientation = Eigen::AngleAxisd(deg(4.0), Vec3::UnitX());
  tilted.age = 0.02;

  const auto flat =
    landfall::estimateGravity(landfall::GravitySource::kFlatFloor, tilted, identity, params);
  ASSERT_TRUE(flat.has_value());
  EXPECT_FALSE(flat->from_imu);

  const auto from_imu =
    landfall::estimateGravity(landfall::GravitySource::kAuto, tilted, identity, params);
  ASSERT_TRUE(from_imu.has_value());
  EXPECT_TRUE(from_imu->from_imu);
  EXPECT_NEAR(from_imu->tilt_deg, 4.0, 1e-9);
  EXPECT_TRUE(from_imu->on_ramp);

  landfall::ImuSample stale = tilted;
  stale.age = 0.5;
  EXPECT_FALSE(
    landfall::estimateGravity(landfall::GravitySource::kAuto, stale, identity, params)->from_imu);
  EXPECT_FALSE(
    landfall::estimateGravity(landfall::GravitySource::kImu, stale, identity, params).has_value());
  EXPECT_FALSE(
    landfall::estimateGravity(landfall::GravitySource::kAuto, std::nullopt, identity, params)
      ->from_imu);

  landfall::ImuSample turning = tilted;
  turning.angular_velocity = Vec3(0.0, 0.0, 0.3);
  EXPECT_FALSE(landfall::estimateGravity(landfall::GravitySource::kAuto, turning, identity, params)
                 .has_value());

  EXPECT_EQ(landfall::parseGravitySource("flat_floor"), landfall::GravitySource::kFlatFloor);
  EXPECT_FALSE(landfall::parseGravitySource("magic").has_value());
}
