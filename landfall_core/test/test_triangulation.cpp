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
#include <cstddef>
#include <random>
#include <string>
#include <vector>

#include "landfall/build/triangulation.hpp"
#include "synthetic_scene.hpp"

using landfall::Pose3;
using landfall::TrackObservation;
using landfall::TriangulationParams;
using landfall::TriangulationStatus;
using landfall::Vec2;
using landfall::Vec3;
using landfall::test::deg;

namespace
{

// Ten views from a robot driving 1.8 m along x, weaving slightly in yaw.
std::vector<Pose3> drivingViews()
{
  std::vector<Pose3> T_cam_map;
  for (int i = 0; i < 10; ++i) {
    const Pose3 T_map_base =
      landfall::test::basePose(0.2 * i, 0.05 * (i % 3), deg(3.0 * (i % 4) - 4.5));
    T_cam_map.push_back((T_map_base * landfall::test::baseToCamera()).inverse());
  }
  return T_cam_map;
}

std::vector<TrackObservation> observe(
  const std::vector<Pose3> & views, const Vec3 & X, std::mt19937 & rng, double noise_px)
{
  std::normal_distribution<double> noise(0.0, noise_px / landfall::test::kFocal);
  std::vector<TrackObservation> observations;
  for (int v = 0; v < static_cast<int>(views.size()); ++v) {
    const Vec3 p = views[v] * X;
    observations.push_back({v, p.hnormalized() + Vec2(noise(rng), noise(rng))});
  }
  return observations;
}

TriangulationParams params()
{
  TriangulationParams p;
  p.focal_px = landfall::test::kFocal;
  return p;
}

}  // namespace

TEST(Triangulation, TenViewsHalfPixelNoiseAtThreeMetres)
{
  const auto views = drivingViews();
  std::mt19937 rng(17);
  std::uniform_real_distribution<double> lateral(-1.5, 1.5);
  std::uniform_real_distribution<double> height(0.2, 2.0);
  std::vector<double> errors;
  for (int i = 0; i < 500; ++i) {
    // About 3 m ahead of the middle of the drive (camera centers span x = 0.2 .. 2.0).
    const Vec3 X(1.1 + 3.0, lateral(rng), height(rng));
    const auto result = landfall::triangulateTrack(views, observe(views, X, rng, 0.5), params());
    ASSERT_EQ(result.status, TriangulationStatus::kOk);
    EXPECT_EQ(result.inliers.size(), 10U);
    errors.push_back((result.position - X).norm());
  }
  const auto middle = errors.begin() + static_cast<std::ptrdiff_t>(errors.size() / 2);
  std::nth_element(errors.begin(), middle, errors.end());
  const double median = errors[errors.size() / 2];
  RecordProperty("median_error_mm", std::to_string(1000.0 * median));
  EXPECT_LT(median, 0.01);
}

TEST(Triangulation, OutlierObservationsAreAlwaysRemoved)
{
  // 0.5 px noise, so a 2 px gate (4 sigma) keeps good observations; any observation displaced
  // by more than 4 px must never survive.
  TriangulationParams p = params();
  p.max_reprojection_px = 2.0;
  const auto views = drivingViews();
  std::mt19937 rng(23);
  std::uniform_real_distribution<double> lateral(-1.5, 1.5);
  std::uniform_real_distribution<double> height(0.2, 2.0);
  std::uniform_real_distribution<double> magnitude(4.0, 40.0);
  std::uniform_real_distribution<double> direction(-M_PI, M_PI);
  std::uniform_int_distribution<int> view(0, 9);
  int injected = 0;
  int survived = 0;
  int clean = 0;
  int clean_kept = 0;
  for (int i = 0; i < 500; ++i) {
    const Vec3 X(4.1, lateral(rng), height(rng));
    auto observations = observe(views, X, rng, 0.5);
    std::vector<char> corrupted(observations.size(), 0);
    const int count = i % 3 == 0 ? 2 : (i % 3 == 1 ? 1 : 0);
    for (int k = 0; k < count; ++k) {
      const int v = view(rng);
      if (corrupted[v] != 0) {
        continue;
      }
      const double a = direction(rng);
      observations[v].normalized +=
        magnitude(rng) / landfall::test::kFocal * Vec2(std::cos(a), std::sin(a));
      corrupted[v] = 1;
      ++injected;
    }
    const auto result = landfall::triangulateTrack(views, observations, p);
    ASSERT_EQ(result.status, TriangulationStatus::kOk);
    for (std::size_t v = 0; v < observations.size(); ++v) {
      const bool kept =
        std::find(result.inliers.begin(), result.inliers.end(), static_cast<int>(v)) !=
        result.inliers.end();
      if (corrupted[v] != 0) {
        survived += kept ? 1 : 0;
      } else {
        ++clean;
        clean_kept += kept ? 1 : 0;
      }
    }
  }
  RecordProperty("injected_outliers", std::to_string(injected));
  RecordProperty("clean_kept", std::to_string(clean_kept) + "/" + std::to_string(clean));
  EXPECT_GT(injected, 300);
  EXPECT_EQ(survived, 0);
  EXPECT_GE(clean_kept, 0.95 * clean);
}

TEST(Triangulation, FiltersRejectWeakOrImplausiblePoints)
{
  std::mt19937 rng(31);
  const auto views = drivingViews();

  // Too few observations.
  auto two = observe(views, Vec3(4.0, 0.0, 1.0), rng, 0.0);
  two.resize(2);
  EXPECT_EQ(
    landfall::triangulateTrack(views, two, params()).status,
    TriangulationStatus::kTooFewObservations);

  // Below the floor (a reflection), beyond the depth range, and with too little parallax.
  EXPECT_EQ(
    landfall::triangulateTrack(views, observe(views, Vec3(4.0, 0.3, -0.2), rng, 0.0), params())
      .status,
    TriangulationStatus::kBelowFloor);
  EXPECT_EQ(
    landfall::triangulateTrack(views, observe(views, Vec3(40.0, 0.3, 1.0), rng, 0.0), params())
      .status,
    TriangulationStatus::kDepthOutOfRange);
  std::vector<Pose3> close_views;
  close_views.reserve(5);
  for (int i = 0; i < 5; ++i) {
    close_views.push_back(
      (landfall::test::basePose(0.01 * i, 0.0, 0.0) * landfall::test::baseToCamera()).inverse());
  }
  const auto result = landfall::triangulateTrack(
    close_views, observe(close_views, Vec3(5.0, 0.3, 1.0), rng, 0.0), params());
  EXPECT_EQ(result.status, TriangulationStatus::kSmallAngle);
  EXPECT_LT(result.max_angle_deg, 2.0);
}
