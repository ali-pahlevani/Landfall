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
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "landfall/geometry/floor_refinement.hpp"

using landfall::FloorPrior;
using landfall::FloorRefinement;
using landfall::FloorRefinementOptions;
using landfall::Mat3;
using landfall::Pose2;
using landfall::Pose3;
using landfall::Vec2;
using landfall::Vec3;

namespace
{

constexpr double kFocal = 500.0;

double deg(double degrees) { return degrees * M_PI / 180.0; }

// Camera 0.3 m above the floor, 0.1 m ahead of the base, looking forward and 5 degrees down.
Pose3 mount(double extra_pitch_deg = 0.0)
{
  Mat3 optical;  // camera x right, y down, z forward, in a base frame with x forward and z up
  optical << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
  Pose3 T = Pose3::Identity();
  T.linear() =
    Eigen::AngleAxisd(deg(5.0 + extra_pitch_deg), Vec3::UnitY()).toRotationMatrix() * optical;
  T.translation() = Vec3(0.1, 0.0, 0.3);
  return T;
}

struct Scene
{
  std::vector<Vec2> normalized;
  std::vector<Vec3> points;
  std::vector<int> indices;
};

// `count` landmarks between `near` and `far` metres ahead within the field of view (half-angles
// `spread` horizontally, `vertical` vertically, normalized), seen with `noise_px` of pixel noise.
// The camera's true mount may differ from mount() by `pitch_error_deg`.
Scene scene(
  const Pose2 & truth, int count, double near, double far, double spread, double noise_px,
  std::mt19937 * rng, double pitch_error_deg = 0.0, double vertical = 0.4)
{
  const Pose3 T_map_cam = landfall::toPose3(truth) * mount(pitch_error_deg);
  std::uniform_real_distribution<double> depth(near, far);
  std::uniform_real_distribution<double> u(-spread, spread);
  std::uniform_real_distribution<double> v(-vertical, vertical);
  std::normal_distribution<double> pixel(0.0, noise_px / kFocal);
  Scene s;
  for (int i = 0; i < count; ++i) {
    const double d = depth(*rng);
    const Vec3 p_cam(u(*rng) * d, v(*rng) * d, d);
    s.points.push_back(T_map_cam * p_cam);
    s.normalized.emplace_back(p_cam.x() / d + pixel(*rng), p_cam.y() / d + pixel(*rng));
    s.indices.push_back(i);
  }
  return s;
}

// A start like the free solution's: a few centimetres and a degree off, and off the floor.
Pose3 start(const Pose2 & truth)
{
  Pose3 T = landfall::toPose3({truth.x + 0.05, truth.y - 0.04, truth.yaw + deg(1.0)});
  T.translation().z() = 0.04;
  T.linear() = T.linear() * Eigen::AngleAxisd(deg(0.8), Vec3::UnitX()).toRotationMatrix();
  return T;
}

FloorPrior floorPrior()
{
  FloorPrior prior;
  prior.T_base_cam = mount();
  prior.height_std = 0.03;
  prior.tilt_std = deg(1.0);
  return prior;
}

FloorRefinementOptions options()
{
  FloorRefinementOptions o;
  o.focal_px = kFocal;
  o.threshold_px = 6.4;
  return o;
}

Vec3 error(const Pose2 & estimate, const Pose2 & truth)
{
  return {
    estimate.x - truth.x, estimate.y - truth.y, landfall::normalizeAngle(estimate.yaw - truth.yaw)};
}

double nees(const FloorRefinement & r, const Pose2 & truth)
{
  const Vec3 e = error(r.pose, truth);
  return e.dot(r.covariance.ldlt().solve(e));
}

}  // namespace

TEST(FloorRefinement, RecoversThePoseOnTheFloor)
{
  std::mt19937 rng(1);
  const Pose2 truth{2.0, 1.0, 0.3};
  const Scene s = scene(truth, 150, 2.0, 8.0, 0.55, 1.0, &rng);
  const auto r =
    refineOnFloor(s.normalized, s.points, s.indices, start(truth), floorPrior(), options());
  ASSERT_TRUE(r.valid);
  EXPECT_LT(landfall::translationDistance(r.pose, truth), 0.01);
  EXPECT_LT(landfall::rotationDistance(r.pose, truth), deg(0.1));
  EXPECT_NEAR(r.T_map_base.translation().z(), 0.0, 0.01);
  EXPECT_NEAR(r.noise_px, 1.0, 0.25);
  EXPECT_EQ(r.inliers, 150);
  EXPECT_LT(r.mean_reprojection_px, 2.0);
  // Millimetres from 150 landmarks a few metres away.
  EXPECT_LT(std::sqrt(r.covariance(0, 0)), 0.01);
  EXPECT_LT(std::sqrt(r.covariance(2, 2)), deg(0.1));
}

TEST(FloorRefinement, CovarianceMatchesTheErrors)
{
  // Over many noisy scenes, the errors spread as the covariance says: the normalized squared
  // error of x, y and yaw averages 3 (chi-square with 3 degrees of freedom).
  std::mt19937 rng(2);
  const Pose2 truth{-3.0, 4.0, -1.2};
  const int trials = 300;
  double sum = 0.0;
  int within = 0;
  for (int trial = 0; trial < trials; ++trial) {
    const Scene s = scene(truth, 60, 2.0, 10.0, 0.55, 1.5, &rng);
    const auto r =
      refineOnFloor(s.normalized, s.points, s.indices, start(truth), floorPrior(), options());
    ASSERT_TRUE(r.valid) << trial;
    const double value = nees(r, truth);
    sum += value;
    within += value <= 7.81 ? 1 : 0;  // chi-square 3, 95 %
  }
  const double mean = sum / trials;
  RecordProperty("mean_nees", std::to_string(mean));
  EXPECT_GT(mean, 2.4);
  EXPECT_LT(mean, 3.6);
  EXPECT_GE(within, static_cast<int>(0.9 * trials));
}

TEST(FloorRefinement, ACorridorStretchesItAlongTheView)
{
  // Looking down a corridor at its far end: landmarks 5 to 40 m ahead in a narrow cone. Their
  // bearings barely change as the camera moves along the view, so the position is far less
  // certain along it than across it (where near and far landmarks shift apart).
  std::mt19937 rng(3);
  const Pose2 truth{0.0, 0.0, deg(30.0)};
  const Scene s = scene(truth, 120, 5.0, 40.0, 0.1, 1.0, &rng, 0.0, 0.1);
  const auto r =
    refineOnFloor(s.normalized, s.points, s.indices, start(truth), floorPrior(), options());
  ASSERT_TRUE(r.valid);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> xy(r.covariance.topLeftCorner<2, 2>());
  const double minor = std::sqrt(xy.eigenvalues()(0));
  const double major = std::sqrt(xy.eigenvalues()(1));
  const Vec2 axis = xy.eigenvectors().col(1);
  // The major axis against the view, either way along it.
  const double turn =
    std::abs(landfall::normalizeAngle(std::atan2(axis.y(), axis.x()) - truth.yaw));
  const double off_view = std::min(turn, M_PI - turn);
  RecordProperty("std_along_m", std::to_string(major));
  RecordProperty("std_across_m", std::to_string(minor));
  EXPECT_GT(major, 3.0 * minor);
  EXPECT_LT(off_view, deg(10.0));
}

TEST(FloorRefinement, KeepsTheBaseOnTheFloor)
{
  // Far landmarks barely tell the camera's height and tilt: a free 6-DoF refinement lets them
  // wander, the floor prior keeps them, and the planar pose is no worse for it.
  std::mt19937 rng(4);
  const Pose2 truth{5.0, -2.0, 2.0};
  FloorPrior free;
  free.T_base_cam = mount();
  double sq_floor = 0.0;
  double sq_free = 0.0;
  double sq_z_floor = 0.0;
  double sq_z_free = 0.0;
  const int trials = 100;
  for (int trial = 0; trial < trials; ++trial) {
    const Scene s = scene(truth, 50, 8.0, 25.0, 0.4, 1.5, &rng);
    const auto on_floor =
      refineOnFloor(s.normalized, s.points, s.indices, start(truth), floorPrior(), options());
    const auto unconstrained =
      refineOnFloor(s.normalized, s.points, s.indices, start(truth), free, options());
    ASSERT_TRUE(on_floor.valid && unconstrained.valid) << trial;
    sq_floor += error(on_floor.pose, truth).head<2>().squaredNorm();
    sq_free += error(unconstrained.pose, truth).head<2>().squaredNorm();
    sq_z_floor += std::pow(on_floor.T_map_base.translation().z(), 2);
    sq_z_free += std::pow(unconstrained.T_map_base.translation().z(), 2);
  }
  const double rms_floor = std::sqrt(sq_floor / trials);
  const double rms_free = std::sqrt(sq_free / trials);
  const double rms_z_floor = std::sqrt(sq_z_floor / trials);
  const double rms_z_free = std::sqrt(sq_z_free / trials);
  RecordProperty("rms_xy_floor_m", std::to_string(rms_floor));
  RecordProperty("rms_xy_free_m", std::to_string(rms_free));
  RecordProperty("rms_z_floor_m", std::to_string(rms_z_floor));
  RecordProperty("rms_z_free_m", std::to_string(rms_z_free));
  EXPECT_LT(rms_z_floor, 0.5 * rms_z_free);
  EXPECT_LT(rms_floor, 1.1 * rms_free);
}

TEST(FloorRefinement, ToleratesACameraTiltedOnItsMount)
{
  // The camera points 1 degree lower than the extrinsics say: the landmarks still decide.
  std::mt19937 rng(5);
  const Pose2 truth{1.0, 2.0, -0.5};
  const Scene s = scene(truth, 150, 2.0, 12.0, 0.55, 1.0, &rng, 1.0);
  const auto r =
    refineOnFloor(s.normalized, s.points, s.indices, start(truth), floorPrior(), options());
  ASSERT_TRUE(r.valid);
  EXPECT_LT(landfall::translationDistance(r.pose, truth), 0.03);
  EXPECT_LT(landfall::rotationDistance(r.pose, truth), deg(0.3));
  EXPECT_GE(r.inliers, 145);
}

TEST(FloorRefinement, WrongCorrespondencesBarelyMoveIt)
{
  std::mt19937 rng(6);
  const Pose2 truth{-1.0, -1.0, 0.8};
  Scene s = scene(truth, 150, 2.0, 10.0, 0.55, 1.0, &rng);
  // Every tenth correspondence points 20 to 40 px away from its landmark.
  std::uniform_real_distribution<double> offset(20.0, 40.0);
  for (std::size_t i = 0; i < s.normalized.size(); i += 10) {
    s.normalized[i] += Vec2(offset(rng), -offset(rng)) / kFocal;
  }
  const auto r =
    refineOnFloor(s.normalized, s.points, s.indices, start(truth), floorPrior(), options());
  ASSERT_TRUE(r.valid);
  EXPECT_LT(landfall::translationDistance(r.pose, truth), 0.02);
  EXPECT_LT(landfall::rotationDistance(r.pose, truth), deg(0.2));
  EXPECT_NEAR(r.noise_px, 1.0, 0.4);
  EXPECT_EQ(r.inliers, 135);
}

TEST(FloorRefinement, ZeroIterationsKeepThePose)
{
  std::mt19937 rng(7);
  const Pose2 truth{0.5, 0.5, 0.0};
  const Scene s = scene(truth, 80, 2.0, 8.0, 0.55, 1.0, &rng);
  FloorRefinementOptions o = options();
  o.max_iterations = 0;
  const Pose3 from = start(truth);
  const auto r = refineOnFloor(s.normalized, s.points, s.indices, from, floorPrior(), o);
  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.iterations, 0);
  EXPECT_TRUE(r.T_map_base.isApprox(from, 1e-9));
  EXPECT_GT(r.covariance.determinant(), 0.0);
}

TEST(FloorRefinement, RefusesWhatFixesNoPose)
{
  std::mt19937 rng(8);
  const Pose2 truth{0.0, 0.0, 0.0};
  const Scene s = scene(truth, 40, 2.0, 8.0, 0.55, 1.0, &rng);
  const std::vector<int> two = {0, 1};
  EXPECT_FALSE(
    refineOnFloor(s.normalized, s.points, two, start(truth), floorPrior(), options()).valid);
  const std::vector<int> outside = {0, 1, 40};
  EXPECT_FALSE(
    refineOnFloor(s.normalized, s.points, outside, start(truth), floorPrior(), options()).valid);
  // Every landmark behind the camera.
  Pose3 backwards = start(truth);
  backwards.linear() =
    backwards.linear() * Eigen::AngleAxisd(M_PI, Vec3::UnitZ()).toRotationMatrix();
  EXPECT_FALSE(
    refineOnFloor(s.normalized, s.points, s.indices, backwards, floorPrior(), options()).valid);
}
