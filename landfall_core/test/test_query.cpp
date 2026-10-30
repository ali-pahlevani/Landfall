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

#include <random>
#include <string>
#include <vector>

#include "landfall/query/clustering.hpp"
#include "landfall/query/decision.hpp"
#include "landfall/query/extrinsics_monitor.hpp"
#include "landfall/query/sanity.hpp"
#include "synthetic_scene.hpp"

using landfall::FrameEstimate;
using landfall::FrameOutcome;
using landfall::Pose2;
using landfall::Pose3;
using landfall::Stage;
using landfall::Status;
using landfall::Vec2;
using landfall::Vec3;
using landfall::test::deg;

namespace
{

// 10 m x 10 m free room with walls, and an unknown band at x > 8 m.
landfall::LikelihoodField room()
{
  landfall::OccupancyGrid grid;
  grid.width = 200;
  grid.height = 200;
  grid.resolution = 0.05;
  constexpr std::size_t kSize = 200;
  grid.data.assign(kSize * kSize, landfall::kCellFree);
  for (std::size_t i = 0; i < kSize; ++i) {
    grid.data[i] = 100;
    grid.data[(kSize - 1) * kSize + i] = 100;
    grid.data[i * kSize] = 100;
    grid.data[i * kSize + kSize - 1] = 100;
    for (std::size_t c = 160; c < kSize - 1; ++c) {
      grid.data[i * kSize + c] = landfall::kCellUnknown;
    }
  }
  return landfall::LikelihoodField(grid);
}

landfall::SanityInput goodInput(const Pose3 & T_map_base)
{
  landfall::SanityInput input;
  input.T_base_cam = landfall::test::baseToCamera();
  input.T_map_cam = T_map_base * input.T_base_cam;
  input.gravity.gravity_base = landfall::flatFloorGravity();
  input.num_correspondences = 200;
  for (int y = 20; y < 480; y += 48) {
    for (int x = 20; x < 640; x += 64) {
      input.inlier_pixels.emplace_back(x, y);
    }
  }
  input.mean_reprojection_px = 1.0;
  input.image_size = cv::Size(640, 480);
  return input;
}

FrameOutcome validFrame(const Pose2 & map_T_base, const Pose2 & odom_T_base)
{
  FrameOutcome frame;
  frame.deepest_stage = Stage::kScanPassed;
  frame.estimate = FrameEstimate{map_T_base, odom_T_base};
  return frame;
}

FrameOutcome failedFrame(Stage stage)
{
  FrameOutcome frame;
  frame.deepest_stage = stage;
  return frame;
}

}  // namespace

TEST(Sanity, AcceptsAPlausiblePoseAndReportsMetrics)
{
  const auto field = room();
  const auto result = landfall::checkSanity(
    goodInput(landfall::test::basePose(3.0, 4.0, 0.4)), landfall::SanityParams(), &field);
  EXPECT_TRUE(result.passed) << result.reason;
  EXPECT_NEAR(result.pose.x, 3.0, 1e-9);
  EXPECT_NEAR(result.pose.yaw, 0.4, 1e-9);
  EXPECT_NEAR(result.height_error_m, 0.0, 1e-9);
  EXPECT_NEAR(result.tilt_error_deg, 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(result.inlier_coverage, 1.0);
  EXPECT_DOUBLE_EQ(result.inlier_ratio, 0.5);
}

TEST(Sanity, EachCheckRejects)
{
  const auto field = room();
  const landfall::SanityParams params;
  const Pose3 base = landfall::test::basePose(3.0, 4.0, 0.4);
  const auto expect_reason = [&](
                               const landfall::SanityInput & input, const std::string & fragment) {
    const auto result = landfall::checkSanity(input, params, &field);
    EXPECT_FALSE(result.passed) << fragment;
    EXPECT_NE(result.reason.find(fragment), std::string::npos) << result.reason;
  };

  auto input = goodInput(base);
  input.inlier_pixels.resize(30);
  expect_reason(input, "inliers <");

  input = goodInput(base);
  input.num_correspondences = 1000;
  expect_reason(input, "inlier ratio");

  input = goodInput(base);
  input.mean_reprojection_px = 5.0;
  expect_reason(input, "reprojection");

  input = goodInput(base);
  for (auto & p : input.inlier_pixels) {
    p = Vec2(10.0 + p.x() / 20.0, 10.0 + p.y() / 20.0);  // all in one corner
  }
  expect_reason(input, "coverage");

  input = goodInput(base);
  input.T_map_cam.translation().z() += 0.2;
  expect_reason(input, "base height");

  input = goodInput(base);
  input.T_map_cam.linear() = input.T_map_cam.linear() * Eigen::AngleAxisd(deg(4.0), Vec3::UnitX());
  expect_reason(input, "tilt error");

  expect_reason(goodInput(landfall::test::basePose(9.0, 4.0, 0.0)), "known-free");   // unknown
  expect_reason(goodInput(landfall::test::basePose(0.02, 4.0, 0.0)), "known-free");  // wall
}

TEST(Sanity, RampsNeedImuGravityAndAConfiguredElevation)
{
  const auto field = room();
  // On a 6 degree ramp, 0.3 m above the floor level.
  Pose3 T_map_base = landfall::test::basePose(3.0, 4.0, 0.0, deg(6.0));
  T_map_base.translation().z() = 0.3;
  landfall::SanityInput input = goodInput(T_map_base);
  input.gravity.gravity_base = T_map_base.linear().transpose() * Vec3(0, 0, -1);
  input.gravity.from_imu = true;
  input.gravity.tilt_deg = 6.0;
  input.gravity.on_ramp = true;

  landfall::SanityParams params;
  auto result = landfall::checkSanity(input, params, &field);
  EXPECT_FALSE(result.passed);
  EXPECT_NE(result.reason.find("max_floor_elevation"), std::string::npos) << result.reason;

  params.max_floor_elevation = 0.5;
  result = landfall::checkSanity(input, params, &field);
  EXPECT_TRUE(result.passed) << result.reason;
  EXPECT_NEAR(result.tilt_error_deg, 0.0, 1e-6);

  // The same pose with flat-floor gravity (no IMU): vision's 6 degree pitch is a tilt error.
  input.gravity = landfall::GravityEstimate();
  result = landfall::checkSanity(input, params, &field);
  EXPECT_FALSE(result.passed);
}

TEST(Clustering, GroupsNearbyPosesAndJudgesAmbiguity)
{
  const std::vector<Pose2> poses = {
    {0.0, 0.0, 0.0}, {0.2, 0.1, 0.05}, {5.0, 0.0, 0.0}, {0.3, 0.0, deg(30.0)}};
  const std::vector<double> scores = {100, 90, 60, 50};
  const auto clusters = landfall::clusterPoses(poses, scores, landfall::ClusterParams());
  ASSERT_EQ(clusters.size(), 3U);
  EXPECT_EQ(clusters[0].members, (std::vector<int>{0, 1}));
  EXPECT_DOUBLE_EQ(clusters[0].score, 100);
  EXPECT_EQ(clusters[1].members, (std::vector<int>{2}));
  EXPECT_EQ(clusters[2].members, (std::vector<int>{3}));

  const landfall::AmbiguityParams params;
  EXPECT_TRUE(landfall::checkAmbiguity(clusters, params).ambiguous);  // 60 / 100
  EXPECT_DOUBLE_EQ(landfall::checkAmbiguity(clusters, params).score_ratio, 0.6);
  EXPECT_TRUE(landfall::checkAmbiguity({clusters[0], clusters[2]}, params).ambiguous);  // 50 / 100
  const landfall::PoseCluster weak{{9}, Pose2{}, 30.0};
  EXPECT_FALSE(landfall::checkAmbiguity({clusters[0], weak}, params).ambiguous);  // 30 / 100
  landfall::AmbiguityParams strict;
  strict.min_inliers = 70;  // the runner-up is too weak to count
  EXPECT_FALSE(landfall::checkAmbiguity(clusters, strict).ambiguous);
  EXPECT_FALSE(landfall::checkAmbiguity({clusters[0]}, params).ambiguous);
}

TEST(Consistency, PropagationAndPairwiseAgreement)
{
  // Facing +y in the map and 1 m forward in odometry: 1 m along +y in the map.
  const Pose2 now = landfall::propagate({1.0, 0.0, M_PI / 2}, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
  EXPECT_NEAR(now.x, 1.0, 1e-12);
  EXPECT_NEAR(now.y, 1.0, 1e-12);

  // The robot turns in place and creeps forward; vision and odometry agree.
  const Pose2 map_T_odom{4.0, -2.0, 0.7};
  std::vector<FrameEstimate> frames;
  for (int i = 0; i < 3; ++i) {
    const Pose2 odom{0.05 * i, 0.0, 0.3 * i};
    frames.push_back({landfall::compose(map_T_odom, odom), odom});
  }
  auto result = landfall::checkConsistency(frames, landfall::ConsistencyParams());
  EXPECT_TRUE(result.consistent);
  EXPECT_LT(result.max_translation_error, 1e-9);
  const Pose2 expected = landfall::compose(map_T_odom, frames.back().odom_T_base);
  EXPECT_NEAR(landfall::translationDistance(result.fused, expected), 0.0, 1e-9);

  frames[1].map_T_base.x += 0.3;
  result = landfall::checkConsistency(frames, landfall::ConsistencyParams());
  EXPECT_FALSE(result.consistent);
  EXPECT_NEAR(result.max_translation_error, 0.3, 1e-9);
}

TEST(Decision, CoversEveryStatus)
{
  const landfall::ConsistencyParams consistency;
  const auto decide = [&](const landfall::DecisionInput & input) {
    return landfall::decide(input, consistency);
  };
  const Pose2 map_T_odom{1.0, 2.0, 0.3};
  const auto agreeing = [&](int i) {
    const Pose2 odom{0.1 * i, 0.0, 0.2 * i};
    return validFrame(landfall::compose(map_T_odom, odom), odom);
  };

  landfall::DecisionInput input;
  input.ready = false;
  input.not_ready_reason = "no camera_info";
  EXPECT_EQ(decide(input).status, Status::kNotReady);
  EXPECT_EQ(decide(input).message, "no camera_info");

  input = {};
  input.busy = true;
  EXPECT_EQ(decide(input).status, Status::kBusy);

  input = {};
  input.frames = {agreeing(0), agreeing(1), agreeing(2)};
  auto decision = decide(input);
  ASSERT_EQ(decision.status, Status::kSuccess);
  ASSERT_TRUE(decision.pose.has_value());
  EXPECT_NEAR(
    landfall::translationDistance(*decision.pose, landfall::compose(map_T_odom, {0.2, 0.0, 0.4})),
    0.0, 1e-9);
  EXPECT_EQ(decision.frames_used, 3);

  input.frames.push_back(failedFrame(Stage::kSanityPassed));
  input.frames.back().ambiguous = true;
  EXPECT_EQ(decide(input).status, Status::kAmbiguous);

  input.frames = {agreeing(0), agreeing(1), agreeing(2)};
  input.frames[2].estimate->map_T_base.y += 0.5;
  EXPECT_EQ(decide(input).status, Status::kRejectedConsistency);

  input.frames = {agreeing(0), agreeing(1), failedFrame(Stage::kSolved)};
  EXPECT_EQ(decide(input).status, Status::kRejectedSanity);

  input.frames = {agreeing(0), failedFrame(Stage::kSanityPassed), failedFrame(Stage::kSolved)};
  EXPECT_EQ(decide(input).status, Status::kRejectedScanCheck);

  input.frames = {failedFrame(Stage::kRetrieved), failedFrame(Stage::kRetrieved)};
  EXPECT_EQ(decide(input).status, Status::kNoMatch);

  input.frames = {};
  EXPECT_EQ(decide(input).status, Status::kNoMatch);

  input.frames = {agreeing(0)};
  input.required_frames = 1;
  EXPECT_EQ(decide(input).status, Status::kSuccess);

  // Values shared with landfall_msgs.
  EXPECT_EQ(static_cast<int>(Status::kRejectedConsistency), 5);
  EXPECT_EQ(static_cast<int>(Status::kBusy), 7);
  EXPECT_EQ(static_cast<int>(Stage::kScanPassed), 3);
  EXPECT_STREQ(landfall::toString(Status::kRejectedScanCheck), "REJECTED_SCAN_CHECK");
}

TEST(ExtrinsicsMonitor, FlagsAMovedCameraAndStaysQuietOtherwise)
{
  const Pose3 T_base_cam = landfall::test::baseToCamera();
  // The camera turned 2 degrees to the left (about base z) and slid 3 cm on its mount.
  Pose3 moved = T_base_cam;
  moved.linear() = Eigen::AngleAxisd(deg(2.0), Vec3::UnitZ()) * T_base_cam.linear();
  moved.translation() += Vec3(0.0, 0.03, 0.0);

  const auto run = [&](const Pose3 & actual_base_cam, int count) {
    std::mt19937 rng(12);
    std::normal_distribution<double> xy(0.0, 0.02);
    std::normal_distribution<double> yaw(0.0, deg(0.5));
    std::uniform_real_distribution<double> place(-10.0, 10.0);
    landfall::ExtrinsicsMonitor monitor;
    for (int i = 0; i < count; ++i) {
      const Pose3 truth = landfall::test::basePose(place(rng), place(rng), place(rng));
      // Localization is a little off in every sample.
      const Pose3 localized = truth * landfall::test::basePose(xy(rng), xy(rng), yaw(rng));
      monitor.addSample(truth * actual_base_cam, localized * T_base_cam);
    }
    return monitor.estimate();
  };

  const auto quiet = run(T_base_cam, 30);
  EXPECT_EQ(quiet.level, landfall::HealthLevel::kOk)
    << quiet.translation_norm << " m, " << quiet.rotation_deg << " deg";

  const auto flagged = run(moved, 30);
  EXPECT_EQ(flagged.level, landfall::HealthLevel::kWarn)
    << flagged.translation_norm << " m, " << flagged.rotation_deg << " deg";
  EXPECT_NEAR(flagged.rotation_deg, 2.0, 0.4);

  Pose3 broken = T_base_cam;
  broken.linear() = Eigen::AngleAxisd(deg(5.0), Vec3::UnitZ()) * T_base_cam.linear();
  EXPECT_EQ(run(broken, 30).level, landfall::HealthLevel::kError);
  EXPECT_EQ(run(moved, 5).level, landfall::HealthLevel::kInsufficientData);

  const auto difference = landfall::compareExtrinsics(T_base_cam, moved);
  EXPECT_NEAR(difference.rotation_deg, 2.0, 1e-9);
  EXPECT_GT(difference.translation, 0.02);
}
