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

#include <opencv2/calib3d.hpp>
#include <random>
#include <stdexcept>
#include <vector>

#include "landfall/camera_model.hpp"

using landfall::CameraModel;
using landfall::DistortionModel;
using landfall::Vec2;
using landfall::Vec3;

namespace
{

// Coefficients similar to real calibrations of each model.
CameraModel makeCamera(DistortionModel model)
{
  switch (model) {
    case DistortionModel::kPlumbBob:
      return {640, 480, 525.0, 523.0, 319.5, 241.0, model, {-0.28, 0.07, 0.001, -0.0005, 0.01}};
    case DistortionModel::kRationalPolynomial:
      return {640,   480,   525.0, 523.0,
              319.5, 241.0, model, {0.9, -0.12, 0.0008, 0.0004, 0.01, 1.2, 0.05, 0.002}};
    case DistortionModel::kEquidistant:
      return {640, 480, 285.0, 285.5, 320.0, 240.0, model, {-0.01, 0.04, -0.03, 0.005}};
    case DistortionModel::kNone:
      break;
  }
  return {640, 480, 525.0, 523.0, 319.5, 241.0, DistortionModel::kNone, {}};
}

std::vector<Vec3> randomPoints(int count, double max_slope)
{
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> slope(-max_slope, max_slope);
  std::uniform_real_distribution<double> depth(0.5, 20.0);
  std::vector<Vec3> points;
  for (int i = 0; i < count; ++i) {
    const double z = depth(rng);
    points.emplace_back(slope(rng) * z, slope(rng) * z, z);
  }
  return points;
}

}  // namespace

TEST(CameraModel, ParsesRosModelNames)
{
  EXPECT_EQ(landfall::parseDistortionModel("plumb_bob"), DistortionModel::kPlumbBob);
  EXPECT_EQ(
    landfall::parseDistortionModel("rational_polynomial"), DistortionModel::kRationalPolynomial);
  EXPECT_EQ(landfall::parseDistortionModel("equidistant"), DistortionModel::kEquidistant);
  EXPECT_EQ(landfall::parseDistortionModel(""), DistortionModel::kNone);
  EXPECT_FALSE(landfall::parseDistortionModel("fisheye").has_value());
  for (auto model :
       {DistortionModel::kNone, DistortionModel::kPlumbBob, DistortionModel::kRationalPolynomial,
        DistortionModel::kEquidistant}) {
    EXPECT_EQ(landfall::parseDistortionModel(landfall::toString(model)), model);
  }
}

TEST(CameraModel, RejectsInvalidParameters)
{
  EXPECT_THROW(
    CameraModel(640, 480, 500, 500, 320, 240, DistortionModel::kPlumbBob, {0.1, 0.2, 0.3}),
    std::invalid_argument);
  EXPECT_THROW(
    CameraModel(
      640, 480, 500, 500, 320, 240, DistortionModel::kEquidistant, {0.1, 0.2, 0.3, 0.4, 0.5}),
    std::invalid_argument);
  EXPECT_THROW(
    CameraModel(0, 480, 500, 500, 320, 240, DistortionModel::kNone, {}), std::invalid_argument);
  EXPECT_THROW(
    CameraModel(640, 480, -1.0, 500, 320, 240, DistortionModel::kNone, {}), std::invalid_argument);
  // Empty coefficients mean "no distortion" for every model, as ROS drivers often send.
  EXPECT_NO_THROW(CameraModel(640, 480, 500, 500, 320, 240, DistortionModel::kPlumbBob, {}));
}

TEST(CameraModel, PinholeAndRationalProjectionMatchOpenCv)
{
  for (auto model :
       {DistortionModel::kNone, DistortionModel::kPlumbBob, DistortionModel::kRationalPolynomial}) {
    const CameraModel camera = makeCamera(model);
    const auto points = randomPoints(500, 0.6);
    std::vector<cv::Point3d> object;
    object.reserve(points.size());
    for (const auto & p : points) {
      object.emplace_back(p.x(), p.y(), p.z());
    }
    const cv::Matx33d K(camera.fx(), 0, camera.cx(), 0, camera.fy(), camera.cy(), 0, 0, 1);
    std::vector<cv::Point2d> expected;
    cv::projectPoints(
      object, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, camera.coefficients(), expected);
    for (std::size_t i = 0; i < points.size(); ++i) {
      Vec2 pixel;
      ASSERT_TRUE(camera.project(points[i], &pixel));
      EXPECT_NEAR(pixel.x(), expected[i].x, 1e-8) << landfall::toString(model);
      EXPECT_NEAR(pixel.y(), expected[i].y, 1e-8) << landfall::toString(model);
    }
  }
}

TEST(CameraModel, EquidistantProjectionMatchesOpenCvFisheye)
{
  const CameraModel camera = makeCamera(DistortionModel::kEquidistant);
  const auto points = randomPoints(500, 1.5);
  std::vector<cv::Point3d> object;
  object.reserve(points.size());
  for (const auto & p : points) {
    object.emplace_back(p.x(), p.y(), p.z());
  }
  const cv::Matx33d K(camera.fx(), 0, camera.cx(), 0, camera.fy(), camera.cy(), 0, 0, 1);
  std::vector<cv::Point2d> expected;
  cv::fisheye::projectPoints(
    object, expected, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, camera.coefficients());
  for (std::size_t i = 0; i < points.size(); ++i) {
    Vec2 pixel;
    ASSERT_TRUE(camera.project(points[i], &pixel));
    EXPECT_NEAR(pixel.x(), expected[i].x, 1e-8);
    EXPECT_NEAR(pixel.y(), expected[i].y, 1e-8);
  }
}

TEST(CameraModel, UndistortInvertsProjection)
{
  for (auto model :
       {DistortionModel::kNone, DistortionModel::kPlumbBob, DistortionModel::kRationalPolynomial,
        DistortionModel::kEquidistant}) {
    const CameraModel camera = makeCamera(model);
    int checked = 0;
    for (const auto & point :
         randomPoints(500, model == DistortionModel::kEquidistant ? 1.5 : 0.6)) {
      Vec2 pixel;
      ASSERT_TRUE(camera.project(point, &pixel));
      if (!camera.contains(pixel)) {
        continue;
      }
      Vec2 normalized;
      ASSERT_TRUE(camera.undistort(pixel, &normalized)) << landfall::toString(model);
      EXPECT_NEAR(normalized.x(), point.x() / point.z(), 1e-9) << landfall::toString(model);
      EXPECT_NEAR(normalized.y(), point.y() / point.z(), 1e-9) << landfall::toString(model);
      ++checked;
    }
    EXPECT_GT(checked, 100) << landfall::toString(model);
  }
}

TEST(CameraModel, PointsBehindTheCameraDoNotProject)
{
  const CameraModel camera = makeCamera(DistortionModel::kPlumbBob);
  Vec2 pixel;
  EXPECT_FALSE(camera.project(Vec3(0.1, 0.2, -1.0), &pixel));
  EXPECT_FALSE(camera.project(Vec3(0.1, 0.2, 0.0), &pixel));
}
