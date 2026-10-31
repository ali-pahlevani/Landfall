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
#include <filesystem>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "landfall/record/image_quality.hpp"
#include "landfall/record/keyframe_selector.hpp"
#include "landfall/record/staging.hpp"
#include "test_utils.hpp"

using landfall::FrameCandidate;
using landfall::GateResult;
using landfall::ImageQuality;
using landfall::KeyframeGating;
using landfall::KeyframeSelector;
using landfall::Mat3;

namespace
{

cv::Mat texture(int width, int height)
{
  cv::Mat image(height, width, CV_8UC1);
  cv::RNG(3).fill(image, cv::RNG::UNIFORM, 40, 220);
  cv::GaussianBlur(image, image, cv::Size(0, 0), 1.0);
  return image;
}

ImageQuality goodQuality() { return {500.0, 120.0, 0.0}; }

FrameCandidate frameAt(double x, double y, double yaw)
{
  FrameCandidate f;
  f.map_base = {x, y, yaw};
  Mat3 cov = Mat3::Zero();
  cov(0, 0) = cov(1, 1) = 0.02 * 0.02;
  cov(2, 2) = std::pow(0.5 * M_PI / 180.0, 2);
  f.covariance = cov;
  f.covariance_age = 0.5;
  return f;
}

double deg(double d) { return d * M_PI / 180.0; }

}  // namespace

TEST(ImageQuality, BlurDarknessAndSaturation)
{
  const cv::Mat sharp = texture(640, 480);
  cv::Mat blurred;
  cv::GaussianBlur(sharp, blurred, cv::Size(0, 0), 4.0);
  const ImageQuality q_sharp = landfall::measureImageQuality(sharp);
  const ImageQuality q_blur = landfall::measureImageQuality(blurred);
  EXPECT_GT(q_sharp.sharpness, 60.0);
  EXPECT_LT(q_blur.sharpness, 60.0);
  EXPECT_NEAR(q_sharp.mean_intensity, 130.0, 5.0);
  EXPECT_EQ(q_sharp.saturated_fraction, 0.0);

  cv::Mat bright = sharp + 120;  // saturates the upper part of the range
  EXPECT_GT(landfall::measureImageQuality(bright).saturated_fraction, 0.2);

  // The measure is resolution-independent above 640 px: an upscaled copy scores alike.
  cv::Mat large;
  cv::resize(sharp, large, cv::Size(1280, 960), 0, 0, cv::INTER_CUBIC);
  const double ratio = landfall::measureImageQuality(large).sharpness / q_sharp.sharpness;
  EXPECT_GT(ratio, 0.5);
  EXPECT_LT(ratio, 2.0);

  cv::Mat color;
  cv::cvtColor(sharp, color, cv::COLOR_GRAY2BGR);
  EXPECT_EQ(landfall::toGray(color).type(), CV_8UC1);
  EXPECT_THROW(landfall::measureImageQuality(color), std::invalid_argument);
}

TEST(KeyframeSelector, MotionGateAndOrdering)
{
  KeyframeSelector selector{KeyframeGating()};
  int measured = 0;
  const auto measure = [&]() {
    ++measured;
    return goodQuality();
  };
  EXPECT_EQ(selector.evaluate(frameAt(0, 0, 0), measure), GateResult::kAccepted);
  EXPECT_EQ(selector.evaluate(frameAt(0.3, 0, deg(5)), measure), GateResult::kNoMotion);
  EXPECT_EQ(selector.evaluate(frameAt(0.5, 0, 0), measure), GateResult::kAccepted);
  EXPECT_EQ(selector.evaluate(frameAt(0.5, 0, deg(16)), measure), GateResult::kAccepted);
  EXPECT_EQ(measured, 3);  // never measured for a rejected pose

  // Pose gates run before the image gates.
  FrameCandidate fast = frameAt(2, 0, 0);
  fast.linear_speed = 0.8;
  EXPECT_EQ(selector.evaluate(fast, measure), GateResult::kSpeed);
  fast.linear_speed = 0.0;
  fast.angular_speed = -0.7;
  EXPECT_EQ(selector.evaluate(fast, measure), GateResult::kSpeed);
  EXPECT_EQ(measured, 3);

  const auto & c = selector.counts();
  EXPECT_EQ(c.frames_seen, 6U);
  EXPECT_EQ(c.keyframes, 3U);
  EXPECT_EQ(c.no_motion, 1U);
  EXPECT_EQ(c.speed, 2U);
  ASSERT_TRUE(selector.lastKeyframe().has_value());
  EXPECT_NEAR(selector.lastKeyframe()->yaw, deg(16), 1e-12);
}

TEST(KeyframeSelector, CovarianceTiltAndImageGates)
{
  KeyframeGating gating;
  KeyframeSelector selector(gating);
  const auto good = [] { return goodQuality(); };

  FrameCandidate f = frameAt(0, 0, 0);
  f.covariance.reset();
  EXPECT_EQ(selector.evaluate(f, good), GateResult::kPoseUncertain);
  f = frameAt(0, 0, 0);
  (*f.covariance)(0, 0) = 0.06 * 0.06;
  EXPECT_EQ(selector.evaluate(f, good), GateResult::kPoseUncertain);
  f = frameAt(0, 0, 0);
  (*f.covariance)(0, 1) = (*f.covariance)(1, 0) = 0.0012;  // strongly correlated x/y
  (*f.covariance)(0, 0) = (*f.covariance)(1, 1) = 0.04 * 0.04;
  EXPECT_GT(landfall::worstStdXy(*f.covariance), 0.05);
  EXPECT_EQ(selector.evaluate(f, good), GateResult::kPoseUncertain);
  f = frameAt(0, 0, 0);
  (*f.covariance)(2, 2) = std::pow(deg(2.5), 2);
  EXPECT_EQ(selector.evaluate(f, good), GateResult::kPoseUncertain);
  f = frameAt(0, 0, 0);
  f.covariance_age = -6.0;
  EXPECT_EQ(selector.evaluate(f, good), GateResult::kPoseUncertain);

  f = frameAt(0, 0, 0);
  f.tilt_deg = 3.0;  // on a ramp
  EXPECT_EQ(selector.evaluate(f, good), GateResult::kTilt);
  f.tilt_deg = 0.5;
  EXPECT_EQ(selector.evaluate(f, [] { return ImageQuality{20.0, 120.0, 0.0}; }), GateResult::kBlur);
  EXPECT_EQ(
    selector.evaluate(f, [] { return ImageQuality{500.0, 20.0, 0.0}; }), GateResult::kExposure);
  EXPECT_EQ(
    selector.evaluate(f, [] { return ImageQuality{500.0, 120.0, 0.3}; }), GateResult::kExposure);
  EXPECT_EQ(selector.evaluate(f, good), GateResult::kAccepted);
  selector.countNoPose();

  const auto & c = selector.counts();
  EXPECT_EQ(c.pose_uncertain, 5U);
  EXPECT_EQ(c.tilt, 1U);
  EXPECT_EQ(c.blur, 1U);
  EXPECT_EQ(c.exposure, 2U);
  EXPECT_EQ(c.no_pose, 1U);
  EXPECT_EQ(c.keyframes, 1U);
  EXPECT_EQ(c.frames_seen, 11U);

  // tf_only sources have no covariance.
  gating.require_covariance = false;
  KeyframeSelector tf_only(gating);
  f = frameAt(0, 0, 0);
  f.covariance.reset();
  EXPECT_EQ(tf_only.evaluate(f, good), GateResult::kAccepted);
}

TEST(KeyframeSelector, LimitStopsRecordingWithoutCounting)
{
  KeyframeGating gating;
  gating.max_keyframes = 2;
  KeyframeSelector selector(gating);
  const auto good = [] { return goodQuality(); };
  EXPECT_EQ(selector.evaluate(frameAt(0, 0, 0), good), GateResult::kAccepted);
  EXPECT_EQ(selector.evaluate(frameAt(1, 0, 0), good), GateResult::kAccepted);
  EXPECT_EQ(selector.evaluate(frameAt(2, 0, 0), good), GateResult::kLimitReached);
  EXPECT_EQ(selector.counts().frames_seen, 2U);
}

TEST(Staging, RoundTripAndCrashTolerance)
{
  landfall::test::TempDir dir;
  const std::string folder = dir.file("session_a");
  landfall::StagingSession session;
  session.name = "session_a";
  session.created_utc = "2026-10-30T12:00:00Z";
  session.landfall_version = "0.0.0";
  session.pose_source = "tf_with_covariance";
  session.frames.camera = "cam_optical";
  session.camera = landfall::CameraModel(
    640, 480, 500.25, 501.5, 319.5, 239.25, landfall::DistortionModel::kPlumbBob,
    {0.1, -0.05, 0.001, -0.002, 0.01});

  landfall::StagingKeyframe k0;
  k0.stamp_ns = 1'700'000'000'123'456'789;
  k0.map_base = {1.0 / 3.0, -2.5, 0.123456789012345};
  k0.covariance = frameAt(0, 0, 0).covariance;
  k0.T_base_cam.translation() = landfall::Vec3(0.1, 0.0, 0.25);
  k0.T_base_cam.linear() = Eigen::AngleAxisd(0.3, landfall::Vec3::UnitY()).toRotationMatrix();
  k0.quality = {123.5, 99.25, 0.01};
  landfall::StagingKeyframe k1 = k0;
  k1.stamp_ns += 500'000'000;
  k1.covariance.reset();
  k1.tilt_deg = 0.75;
  {
    landfall::StagingWriter writer(folder, session);
    EXPECT_THROW({ landfall::StagingWriter again(folder, session); }, std::runtime_error);
    writer.add(k0, texture(640, 480));
    const auto & added = writer.add(k1, texture(640, 480));
    EXPECT_EQ(added.id, 1);
    EXPECT_EQ(added.image, "images/000001.png");
    landfall::GateCounts counts;
    counts.frames_seen = 42;
    counts.keyframes = 2;
    counts.blur = 3;
    writer.close(counts);
  }

  const landfall::StagingFolder read = landfall::readStaging(folder);
  EXPECT_TRUE(read.session.complete);
  EXPECT_EQ(read.session.counts.frames_seen, 42U);
  EXPECT_EQ(read.session.counts.blur, 3U);
  EXPECT_EQ(read.session.frames.camera, "cam_optical");
  EXPECT_EQ(read.session.camera.fx(), 500.25);
  EXPECT_EQ(read.session.camera.coefficients(), session.camera.coefficients());
  ASSERT_EQ(read.keyframes.size(), 2U);
  const auto & r0 = read.keyframes[0];
  EXPECT_EQ(r0.stamp_ns, k0.stamp_ns);
  EXPECT_EQ(r0.map_base.x, k0.map_base.x);  // bit-exact
  EXPECT_EQ(r0.map_base.yaw, k0.map_base.yaw);
  ASSERT_TRUE(r0.covariance.has_value());
  EXPECT_EQ(*r0.covariance, *k0.covariance);
  EXPECT_TRUE(r0.T_base_cam.isApprox(k0.T_base_cam, 1e-12));
  EXPECT_EQ(r0.quality.sharpness, 123.5);
  EXPECT_FALSE(r0.tilt_deg.has_value());
  EXPECT_FALSE(read.keyframes[1].covariance.has_value());
  EXPECT_EQ(read.keyframes[1].tilt_deg.value_or(-1.0), 0.75);
  const cv::Mat image = landfall::loadStagingImage(read, r0, cv::IMREAD_GRAYSCALE);
  EXPECT_EQ(cv::norm(image, texture(640, 480), cv::NORM_INF), 0.0);  // PNG is lossless

  // A crash while appending leaves a partial last line: it's ignored.
  {
    std::ofstream csv(folder + "/keyframes.csv", std::ios::app);
    csv << "2,1700000001123456789,images/000002.png,0.5,";
  }
  EXPECT_EQ(landfall::readStaging(folder).keyframes.size(), 2U);
  // A complete malformed line is an error.
  {
    std::ofstream csv(folder + "/keyframes.csv", std::ios::app);
    csv << "\n";
  }
  EXPECT_THROW(landfall::readStaging(folder), std::runtime_error);
  EXPECT_THROW(landfall::readStaging(dir.file("missing")), std::runtime_error);
}
