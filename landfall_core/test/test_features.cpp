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
#include <vector>

#include "landfall/features/bucketing.hpp"
#include "landfall/features/matchers.hpp"
#include "landfall/features/orb.hpp"
#include "landfall/features/sift.hpp"

using landfall::Features;
using landfall::GridSize;

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

}  // namespace

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
  const Features features = orb.extract(image);
  EXPECT_GT(features.keypoints.size(), 1500U);
  EXPECT_LE(features.keypoints.size(), 2000U);
  EXPECT_EQ(features.type, landfall::DescriptorType::kBinary);
  EXPECT_EQ(features.descriptors.type(), CV_8UC1);
  EXPECT_EQ(features.descriptors.cols, 32);
  EXPECT_EQ(features.descriptors.rows, static_cast<int>(features.keypoints.size()));
  EXPECT_GE(landfall::gridCoverage(pixels(features), image.size(), GridSize()), 0.95);

  const Features again = orb.extract(image);
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
  const auto without = landfall::OrbFeatures(strict).extract(image);
  const auto with = landfall::OrbFeatures().extract(image);
  const auto count_right = [](const Features & f) {
    int n = 0;
    for (const auto & k : f.keypoints) {
      n += k.x >= 320.0F ? 1 : 0;
    }
    return n;
  };
  EXPECT_GT(count_right(with), count_right(without));
}

TEST(SiftFeatures, RootSiftRowsHaveUnitNorm)
{
  const Features features = landfall::SiftFeatures().extract(texturedImage());
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
      const Features a = extractor.extract(image);
      const Features b = extractor.extract(warped);
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
