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
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

#include "landfall/camera_model.hpp"
#include "landfall/features/bucketing.hpp"
#include "landfall/features/floor_mask.hpp"
#include "landfall/features/matchers.hpp"
#include "landfall/features/orb.hpp"
#include "landfall/features/sift.hpp"

using landfall::CameraModel;
using landfall::Features;
using landfall::GridSize;
using landfall::Mat3;
using landfall::Pose3;
using landfall::Vec2;
using landfall::Vec3;

namespace
{

// Deterministic cluttered scene: shapes of random gray levels over smooth noise.
cv::Mat texturedImage(int seed = 1)
{
  cv::RNG rng(seed);
  cv::Mat image(480, 640, CV_8UC1);
  rng.fill(image, cv::RNG::UNIFORM, 60, 200);
  cv::GaussianBlur(image, image, cv::Size(0, 0), 3.0);
  for (int i = 0; i < 400; ++i) {
    const cv::Point center(rng.uniform(0, 640), rng.uniform(0, 480));
    const cv::Scalar color(rng.uniform(0, 256));
    if (i % 2 == 0) {
      cv::rectangle(
        image, center, center + cv::Point(rng.uniform(5, 40), rng.uniform(5, 40)), color, -1);
    } else {
      cv::circle(image, center, rng.uniform(3, 20), color, -1);
    }
  }
  return image;
}

std::vector<landfall::Vec2> pixels(const Features & features)
{
  std::vector<landfall::Vec2> result;
  result.reserve(features.keypoints.size());
  for (const auto & k : features.keypoints) {
    result.emplace_back(k.x, k.y);
  }
  return result;
}

CameraModel pinhole()
{
  return {640, 480, 500.0, 500.0, 320.0, 240.0, landfall::DistortionModel::kNone, {}};
}

// A camera `height` above the floor looking along the base's x axis, pitched down and rolled.
Pose3 mountedCamera(double height, double pitch_down_deg = 0.0, double roll_deg = 0.0)
{
  Mat3 base_optical;  // REP 103 optical frame: z forward, x right, y down
  base_optical << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Pose3 pose = Pose3::Identity();
  pose.linear() =
    Eigen::AngleAxisd(roll_deg * M_PI / 180.0, Vec3::UnitX()).toRotationMatrix() *
    Eigen::AngleAxisd(pitch_down_deg * M_PI / 180.0, Vec3::UnitY()).toRotationMatrix() *
    base_optical;
  pose.translation() = Vec3(0.3, 0.0, height);
  return pose;
}

// The mask pixel OpenCV checks for a keypoint (KeyPointsFilter::runByPixelsMask truncates the
// coordinate + 0.5; the same as rounding half up for these non-negative coordinates).
int maskRow(const landfall::Keypoint & k) { return static_cast<int>(std::floor(k.y + 0.5F)); }
int maskCol(const landfall::Keypoint & k) { return static_cast<int>(std::floor(k.x + 0.5F)); }

// Pixels whose hidden state differs from the exact per-pixel answer, ignoring ties (|margin| tiny).
int maskErrors(
  const cv::Mat & mask, const CameraModel & camera, const Pose3 & T_base_cam, double distance)
{
  const double slope = T_base_cam.translation().z() / distance;
  int errors = 0;
  for (int v = 0; v < camera.height(); ++v) {
    for (int u = 0; u < camera.width(); ++u) {
      Vec2 n;
      if (!camera.undistort(Vec2(u, v), &n)) {
        continue;
      }
      const Vec3 r = T_base_cam.rotation() * Vec3(n.x(), n.y(), 1.0);
      const double margin = -r.z() - slope * r.head<2>().norm();
      const bool hidden = !mask.empty() && mask.at<uint8_t>(v, u) == 0;
      if (std::abs(margin) > 1e-9 && hidden != (margin > 0.0)) {
        ++errors;
      }
    }
  }
  return errors;
}

}  // namespace

TEST(FloorMask, HidesTheFloorNearerThanTheDistance)
{
  const CameraModel camera = pinhole();
  const Pose3 level = mountedCamera(0.5);
  const cv::Mat mask = landfall::floorMask(camera, level, 5.0);
  ASSERT_EQ(mask.size(), cv::Size(640, 480));
  ASSERT_EQ(mask.type(), CV_8UC1);
  EXPECT_EQ(maskErrors(mask, camera, level, 5.0), 0);
  // Straight ahead the floor 5 m away is 500 * 0.5 / 5 = 50 rows below the horizon; to the sides
  // the same rows see it farther, so the hidden part starts lower there.
  EXPECT_EQ(mask.at<uint8_t>(289, 320), 255);
  EXPECT_EQ(mask.at<uint8_t>(291, 320), 0);
  EXPECT_EQ(mask.at<uint8_t>(295, 0), 255);
  EXPECT_EQ(mask.at<uint8_t>(479, 0), 0);
  EXPECT_EQ(cv::countNonZero(mask(cv::Rect(0, 0, 640, 241))), 640 * 241);  // above the horizon

  // Pitched down and rolled: still exact (the rays are linear in the pixel without distortion).
  const Pose3 tilted = mountedCamera(1.2, 15.0, 5.0);
  EXPECT_EQ(maskErrors(landfall::floorMask(camera, tilted, 3.0), camera, tilted, 3.0), 0);
}

TEST(FloorMask, FollowsTheLensDistortion)
{
  const CameraModel camera(
    640, 480, 500.0, 500.0, 320.0, 240.0, landfall::DistortionModel::kPlumbBob,
    {-0.25, 0.08, 0.001, -0.0005, 0.0});
  const Pose3 T_base_cam = mountedCamera(0.5, 3.0);
  const cv::Mat mask = landfall::floorMask(camera, T_base_cam, 5.0);
  ASSERT_FALSE(mask.empty());
  // Interpolated between grid rays, the boundary may only be off by a fraction of a pixel.
  EXPECT_LT(maskErrors(mask, camera, T_base_cam, 5.0), 640 * 2);
  // Floor points projected through the lens: hidden at 4 m, visible at 6.25 m.
  const Pose3 T_cam_base = T_base_cam.inverse();
  const Vec3 center = T_base_cam.translation();
  for (int bearing = -30; bearing <= 30; bearing += 5) {
    const double a = bearing * M_PI / 180.0;
    for (const double d : {4.0, 6.25}) {
      const Vec3 floor_point(center.x() + d * std::cos(a), center.y() + d * std::sin(a), 0.0);
      Vec2 pixel;
      ASSERT_TRUE(camera.project(T_cam_base * floor_point, &pixel));
      ASSERT_TRUE(camera.contains(pixel));
      const int u = static_cast<int>(std::lround(pixel.x()));
      const int v = static_cast<int>(std::lround(pixel.y()));
      EXPECT_EQ(mask.at<uint8_t>(v, u), d < 5.0 ? 0 : 255) << bearing << " deg, " << d << " m";
    }
  }
}

TEST(FloorMask, NothingHiddenGivesAnEmptyMask)
{
  const CameraModel camera = pinhole();
  EXPECT_TRUE(landfall::floorMask(camera, mountedCamera(0.5), 0.0).empty());
  EXPECT_TRUE(landfall::floorMask(camera, mountedCamera(0.0), 5.0).empty());         // on the floor
  EXPECT_TRUE(landfall::floorMask(camera, mountedCamera(0.5, -60.0), 5.0).empty());  // looks up
  // The backends' entry point: needs the image's camera only when the floor is hidden.
  const landfall::ImageGeometry unknown;
  EXPECT_TRUE(landfall::detectionMask("orb", 0.0, cv::Size(640, 480), unknown).empty());
  EXPECT_THROW(
    landfall::detectionMask("orb", 5.0, cv::Size(640, 480), unknown), std::invalid_argument);
  const landfall::ImageGeometry geometry{&camera, mountedCamera(0.5)};
  EXPECT_THROW(
    landfall::detectionMask("orb", 5.0, cv::Size(320, 240), geometry), std::invalid_argument);
  EXPECT_FALSE(landfall::detectionMask("orb", 5.0, cv::Size(640, 480), geometry).empty());
}

TEST(Bucketing, SharesFollowTheDetectionMask)
{
  const GridSize grid;  // 8 x 6 cells of 80 x 80 pixels
  const cv::Size size(640, 480);
  EXPECT_EQ(landfall::cellShares(size, grid, 2000), std::vector<int>(48, 41));
  cv::Mat mask(size, CV_8UC1, cv::Scalar(255));
  mask(cv::Rect(0, 240, 640, 240)).setTo(0);  // the bottom three rows of cells
  mask(cv::Rect(0, 200, 640, 40)).setTo(0);   // and half of the row above
  const std::vector<int> shares = landfall::cellShares(size, grid, 2000, mask);
  // 640 * 200 pixels left: 2000 * 6400 / 128000 = 100 per full cell, 50 per half cell.
  for (int cell = 0; cell < 48; ++cell) {
    EXPECT_EQ(shares[cell], cell < 16 ? 100 : (cell < 24 ? 50 : 0)) << cell;
  }
  EXPECT_THROW(
    landfall::cellShares(size, grid, 2000, cv::Mat(10, 10, CV_8UC1)), std::invalid_argument);
}

TEST(Bucketing, SpreadsTheBudgetOverTheGrid)
{
  std::vector<cv::KeyPoint> keypoints;
  keypoints.reserve(5000 + 24 * 32);
  // Many strong keypoints crowded in the top-left cell, weak ones everywhere else.
  for (int i = 0; i < 5000; ++i) {
    const auto x = static_cast<float>(i % 70);
    const auto y = static_cast<float>((i / 70) % 70);
    keypoints.emplace_back(cv::Point2f(x, y), 7.0F, -1.0F, 100.0F + static_cast<float>(i % 13));
  }
  for (int y = 0; y < 480; y += 20) {
    for (int x = 0; x < 640; x += 20) {
      keypoints.emplace_back(
        cv::Point2f(static_cast<float>(x) + 5.0F, static_cast<float>(y) + 5.0F), 7.0F, -1.0F, 1.0F);
    }
  }
  const GridSize grid;
  const auto kept = landfall::bucketKeypoints(keypoints, cv::Size(640, 480), grid, 480);
  ASSERT_EQ(kept.size(), 480U);
  std::vector<landfall::Vec2> points;
  points.reserve(kept.size());
  for (const auto & k : kept) {
    points.emplace_back(k.pt.x, k.pt.y);
  }
  EXPECT_DOUBLE_EQ(landfall::gridCoverage(points, cv::Size(640, 480), grid), 1.0);
  EXPECT_EQ(
    landfall::bucketKeypoints(keypoints, cv::Size(640, 480), grid, 480).size(), kept.size());
  EXPECT_TRUE(landfall::bucketKeypoints(keypoints, cv::Size(640, 480), grid, 0).empty());
}

TEST(Bucketing, GridCoverageCountsOccupiedCells)
{
  const GridSize grid{4, 2};
  const cv::Size size(400, 200);
  EXPECT_DOUBLE_EQ(landfall::gridCoverage({}, size, grid), 0.0);
  EXPECT_DOUBLE_EQ(landfall::gridCoverage({{10, 10}, {20, 20}}, size, grid), 1.0 / 8.0);
  EXPECT_DOUBLE_EQ(
    landfall::gridCoverage({{10, 10}, {390, 190}, {-5, 500}}, size, grid), 3.0 / 8.0);
}

TEST(OrbFeatures, ExtractsBucketedDeterministicFeatures)
{
  const landfall::OrbFeatures orb;
  const cv::Mat image = texturedImage();
  const Features features = orb.extract(image, {});
  EXPECT_GT(features.keypoints.size(), 1500U);
  EXPECT_LE(features.keypoints.size(), 2000U);
  EXPECT_EQ(features.type, landfall::DescriptorType::kBinary);
  EXPECT_EQ(features.descriptors.type(), CV_8UC1);
  EXPECT_EQ(features.descriptors.cols, 32);
  EXPECT_EQ(features.descriptors.rows, static_cast<int>(features.keypoints.size()));
  EXPECT_GE(landfall::gridCoverage(pixels(features), image.size(), GridSize()), 0.95);

  const Features again = orb.extract(image, {});
  ASSERT_EQ(again.keypoints.size(), features.keypoints.size());
  EXPECT_EQ(cv::norm(again.descriptors, features.descriptors, cv::NORM_HAMMING), 0.0);
  EXPECT_EQ(orb.id().name, "orb");
  EXPECT_EQ(landfall::configSha256(orb.id()), landfall::configSha256(landfall::OrbFeatures().id()));
  landfall::OrbParams other;
  other.max_keypoints = 1000;
  EXPECT_NE(
    landfall::configSha256(orb.id()), landfall::configSha256(landfall::OrbFeatures(other).id()));
}

TEST(OrbFeatures, LowTextureCellsGetTheLowerThreshold)
{
  // Left half textured, right half nearly flat: the retry still finds corners on the right.
  cv::Mat image = texturedImage();
  cv::Mat right = image(cv::Rect(320, 0, 320, 480));
  cv::GaussianBlur(right, right, cv::Size(0, 0), 2.5);
  right = right * 0.25 + 90;
  landfall::OrbParams strict;
  strict.min_fast_threshold = strict.fast_threshold;  // no retry
  const auto without = landfall::OrbFeatures(strict).extract(image, {});
  const auto with = landfall::OrbFeatures().extract(image, {});
  const auto count_right = [](const Features & f) {
    int n = 0;
    for (const auto & k : f.keypoints) {
      n += k.x >= 320.0F ? 1 : 0;
    }
    return n;
  };
  EXPECT_GT(count_right(with), count_right(without));
}

TEST(OrbFeatures, HidesTheFloor)
{
  const CameraModel camera = pinhole();
  const landfall::ImageGeometry geometry{&camera, mountedCamera(0.5)};
  const cv::Mat floor = landfall::floorMask(camera, geometry.T_base_cam, 5.0);
  const cv::Mat image = texturedImage();
  landfall::OrbParams params;
  params.floor_mask_distance = 5.0;
  const landfall::OrbFeatures orb(params);
  const Features features = orb.extract(image, geometry);
  ASSERT_GT(features.keypoints.size(), 1000U);
  // ORB applies the mask in each pyramid level: a keypoint of a coarse level may land a few
  // pixels past the boundary, which starts at row 291 (center) to 300 (sides).
  int inside = 0;
  for (const auto & k : features.keypoints) {
    EXPECT_LT(k.y, 312.0F);
    inside += floor.at<uint8_t>(maskRow(k), maskCol(k)) != 0 ? 1 : 0;
  }
  EXPECT_GT(inside, 0.98 * static_cast<double>(features.keypoints.size()));
  // Every cell with room keeps keypoints: the budget is shared by the unmasked area.
  std::vector<Vec2> top;
  for (const auto & p : pixels(features)) {
    if (p.y() < 240.0) {
      top.push_back(p);
    }
  }
  EXPECT_DOUBLE_EQ(landfall::gridCoverage(top, image.size(), GridSize()), 0.5);
  // Stored only when set: databases built before the option keep their configuration.
  EXPECT_EQ(
    landfall::OrbFeatures().id().config,
    "fast_threshold: 20\ngrid: 8x6\nlevels: 8\nmax_keypoints: 2000\nmin_fast_threshold: 7\n"
    "oversample: 2\nscale_factor: 1.2\n");
  EXPECT_NE(orb.id().config.find("floor_mask_distance: 5\n"), std::string::npos);
  EXPECT_THROW(orb.extract(image, {}), std::invalid_argument);  // the camera is needed
  params.floor_mask_distance = -1.0;
  EXPECT_THROW(landfall::OrbFeatures{params}, std::invalid_argument);
}

TEST(SiftFeatures, HidesTheFloor)
{
  const CameraModel camera = pinhole();
  const landfall::ImageGeometry geometry{&camera, mountedCamera(0.5)};
  const cv::Mat floor = landfall::floorMask(camera, geometry.T_base_cam, 5.0);
  landfall::SiftParams params;
  params.floor_mask_distance = 5.0;
  const Features features = landfall::SiftFeatures(params).extract(texturedImage(), geometry);
  ASSERT_GT(features.keypoints.size(), 100U);
  for (const auto & k : features.keypoints) {
    EXPECT_NE(floor.at<uint8_t>(maskRow(k), maskCol(k)), 0) << k.x << ", " << k.y;
  }
  EXPECT_EQ(landfall::SiftFeatures().id().config.find("floor_mask"), std::string::npos);
}

TEST(SiftFeatures, RootSiftRowsHaveUnitNorm)
{
  const Features features = landfall::SiftFeatures().extract(texturedImage(), {});
  ASSERT_GT(features.keypoints.size(), 300U);
  EXPECT_LE(features.keypoints.size(), 2000U);
  EXPECT_EQ(features.type, landfall::DescriptorType::kFloat32);
  ASSERT_EQ(features.descriptors.cols, 128);
  for (int i = 0; i < features.descriptors.rows; ++i) {
    EXPECT_NEAR(cv::norm(features.descriptors.row(i)), 1.0, 1e-4);
  }
}

TEST(Matchers, MatchesAreGeometricallyConsistentUnderAHomography)
{
  const cv::Mat image = texturedImage(5);
  const cv::Mat H =
    (cv::Mat_<double>(3, 3) << 1.03 * std::cos(0.08), -std::sin(0.08), 25.0, std::sin(0.08),
     1.03 * std::cos(0.08), -12.0, 0.0, 0.0, 1.0);
  cv::Mat warped;
  cv::warpPerspective(image, warped, H, image.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);

  const auto check =
    [&](const landfall::ILocalFeatures & extractor, const landfall::IMatcher & matcher) {
      const Features a = extractor.extract(image, {});
      const Features b = extractor.extract(warped, {});
      const landfall::Matches matches = matcher.match(a, b);
      int consistent = 0;
      std::vector<char> used_query(a.keypoints.size(), 0);
      std::vector<char> used_train(b.keypoints.size(), 0);
      for (const auto & m : matches) {
        EXPECT_EQ(used_query[m.query]++, 0);
        EXPECT_EQ(used_train[m.train]++, 0);
        std::vector<cv::Point2f> mapped;
        cv::perspectiveTransform(
          std::vector<cv::Point2f>{{a.keypoints[m.query].x, a.keypoints[m.query].y}}, mapped, H);
        const double error =
          std::hypot(mapped[0].x - b.keypoints[m.train].x, mapped[0].y - b.keypoints[m.train].y);
        consistent += error < 3.0 ? 1 : 0;
      }
      EXPECT_GT(matches.size(), 200U) << matcher.id().name;
      EXPECT_GT(consistent, 0.9 * static_cast<double>(matches.size())) << matcher.id().name;
    };
  check(landfall::OrbFeatures(), landfall::RatioMatcher());
  check(landfall::SiftFeatures(), landfall::RatioMatcher());
  check(
    landfall::SiftFeatures(), landfall::MutualNearestMatcher(landfall::MutualNearestParams{0.9}));
}
