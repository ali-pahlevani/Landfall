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

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "landfall/ros/conversions.hpp"

using landfall::DistortionModel;

namespace
{

sensor_msgs::msg::CameraInfo cameraInfo(const std::string & model, std::vector<double> d)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 640;
  info.height = 480;
  info.distortion_model = model;
  info.d = std::move(d);
  info.k = {500.0, 0.0, 319.5, 0.0, 501.0, 239.5, 0.0, 0.0, 1.0};
  return info;
}

cv::Mat colorPattern()
{
  cv::Mat image(48, 64, CV_8UC3);
  cv::RNG(1).fill(image, cv::RNG::UNIFORM, 0, 256);
  return image;
}

}  // namespace

TEST(Conversions, CameraInfoNormalization)
{
  const auto plumb =
    landfall::ros::toCameraModel(cameraInfo("plumb_bob", {0.1, -0.2, 0.0, 0.0, 0.05}));
  EXPECT_EQ(plumb.model(), DistortionModel::kPlumbBob);
  EXPECT_EQ(plumb.fx(), 500.0);
  EXPECT_EQ(plumb.fy(), 501.0);
  EXPECT_EQ(plumb.cx(), 319.5);
  EXPECT_EQ(plumb.coefficients().size(), 5U);

  // OpenCV's 14-term rational model with unused terms, and an all-zero plumb_bob.
  std::vector<double> rational(14, 0.0);
  rational[0] = 0.3;
  rational[7] = 0.01;
  EXPECT_EQ(
    landfall::ros::toCameraModel(cameraInfo("rational_polynomial", rational)).coefficients().size(),
    8U);
  // Zero padding beyond the model's terms is dropped (the model keeps its own 5 terms).
  EXPECT_EQ(
    landfall::ros::toCameraModel(cameraInfo("plumb_bob", std::vector<double>(8, 0.0)))
      .coefficients(),
    std::vector<double>(5, 0.0));
  EXPECT_EQ(
    landfall::ros::toCameraModel(cameraInfo("equidistant", {0.1, 0.0, 0.0, 0.01})).model(),
    DistortionModel::kEquidistant);

  auto uncalibrated = cameraInfo("plumb_bob", {});
  uncalibrated.k = {};
  EXPECT_THROW(landfall::ros::toCameraModel(uncalibrated), std::invalid_argument);
  EXPECT_THROW(landfall::ros::toCameraModel(cameraInfo("fisheye_xyz", {})), std::invalid_argument);
  rational[12] = 0.5;  // a used tilt term: not representable
  EXPECT_THROW(
    landfall::ros::toCameraModel(cameraInfo("rational_polynomial", rational)),
    std::invalid_argument);
}

TEST(Conversions, DecodesRawAndCompressedImages)
{
  const cv::Mat bgr = colorPattern();
  sensor_msgs::msg::Image raw;
  raw.height = static_cast<uint32_t>(bgr.rows);
  raw.width = static_cast<uint32_t>(bgr.cols);
  raw.encoding = "rgb8";
  raw.step = static_cast<uint32_t>(bgr.cols * 3);
  cv::Mat rgb;
  cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
  raw.data.assign(rgb.data, rgb.data + rgb.total() * rgb.elemSize());
  const cv::Mat decoded = landfall::ros::decodeImage(raw);
  EXPECT_EQ(decoded.type(), CV_8UC3);
  EXPECT_EQ(cv::norm(decoded, bgr, cv::NORM_INF), 0.0);  // back to BGR

  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  raw.encoding = "mono8";
  raw.step = static_cast<uint32_t>(gray.cols);
  raw.data.assign(gray.data, gray.data + gray.total());
  EXPECT_EQ(landfall::ros::decodeImage(raw).type(), CV_8UC1);
  raw.encoding = "32FC1";
  raw.step = static_cast<uint32_t>(gray.cols * 4);
  raw.data.assign(gray.total() * 4, 0);
  EXPECT_THROW(landfall::ros::decodeImage(raw), std::runtime_error);

  // Messages' byte fields are std::vector before Lyrical and rosidl::Buffer since: both take a
  // std::vector assignment.
  std::vector<uint8_t> encoded;
  sensor_msgs::msg::CompressedImage png;
  png.format = "bgr8; png compressed bgr8";
  cv::imencode(".png", bgr, encoded);
  png.data = encoded;
  EXPECT_EQ(cv::norm(landfall::ros::decodeImage(png), bgr, cv::NORM_INF), 0.0);
  sensor_msgs::msg::CompressedImage jpeg;
  jpeg.format = "bgr8; jpeg compressed bgr8";
  cv::imencode(".jpg", bgr, encoded);
  jpeg.data = encoded;
  EXPECT_EQ(landfall::ros::decodeImage(jpeg).size(), bgr.size());
  jpeg.data.resize(10);
  EXPECT_THROW(landfall::ros::decodeImage(jpeg), std::runtime_error);
  jpeg.data.clear();
  EXPECT_THROW(landfall::ros::decodeImage(jpeg), std::runtime_error);
}

TEST(Conversions, StampsPosesAndCovariance)
{
  for (const int64_t ns :
       {int64_t{0}, int64_t{1'700'000'000'123'456'789}, int64_t{-1'500'000'000}}) {
    EXPECT_EQ(landfall::ros::toNanoseconds(landfall::ros::toStamp(ns)), ns);
  }
  landfall::Pose3 pose = landfall::Pose3::Identity();
  pose.translation() = landfall::Vec3(1, -2, 0.5);
  pose.linear() = Eigen::AngleAxisd(0.4, landfall::Vec3(1, 2, 3).normalized()).toRotationMatrix();
  EXPECT_TRUE(landfall::ros::toPose3(landfall::ros::toPoseMsg(pose)).isApprox(pose, 1e-12));

  geometry_msgs::msg::PoseWithCovariance::_covariance_type c{};
  c[0] = 1.0;   // xx
  c[1] = 0.1;   // xy
  c[5] = 0.2;   // x yaw
  c[7] = 2.0;   // yy
  c[11] = 0.3;  // y yaw
  c[35] = 3.0;  // yaw yaw
  c[6] = 0.1;
  c[30] = 0.2;
  c[31] = 0.3;
  landfall::Mat3 expected;
  expected << 1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0;
  EXPECT_EQ(landfall::ros::planarCovariance(c), expected);
}
