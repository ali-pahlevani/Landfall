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
#include <fstream>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

#include "landfall/build/builder_config.hpp"
#include "landfall/build/map_builder.hpp"
#include "landfall/build/pose_prior_ba.hpp"
#include "landfall/query/localization_map.hpp"
#include "landfall/query/localizer.hpp"
#include "synthetic_map.hpp"
#include "test_utils.hpp"

using landfall::Features;
using landfall::Pose2;
using landfall::Pose3;
using landfall::Vec2;
using landfall::test::deg;

namespace
{

// The synthetic room of the map-builder test, seen from a loop driven both ways (1 m spacing).
struct World
{
  landfall::test::SyntheticRoom room;
  landfall::CameraModel camera = landfall::test::syntheticCamera();
  Pose3 T_base_cam = landfall::test::baseToCamera();
};

// Projections (0.5 px noise) of the visible landmarks with a few flipped descriptor bits, plus
// random distractors.
Features observe(const World & world, const Pose3 & T_base_cam_used, const Pose2 & pose, int seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.5);
  std::uniform_int_distribution<int> bit(0, 255);
  std::uniform_real_distribution<float> u(0.0F, 1.0F);
  const Pose3 T_cam_map = landfall::cameraPose(pose, T_base_cam_used).inverse();
  Features f;
  f.type = landfall::DescriptorType::kBinary;
  for (std::size_t l = 0; l < world.room.landmarks.size(); ++l) {
    Vec2 px;
    if (!landfall::test::projectPoint(world.camera, T_cam_map, world.room.landmarks[l], &px)) {
      continue;
    }
    px += Vec2(noise(rng), noise(rng));
    f.keypoints.push_back(
      {static_cast<float>(px.x()), static_cast<float>(px.y()), 31.0F, 0.0F, 1.0F});
    cv::Mat row = world.room.descriptors.row(static_cast<int>(l)).clone();
    for (int flip = 0; flip < 6; ++flip) {
      const int b = bit(rng);
      row.at<uint8_t>(0, b / 8) ^= static_cast<uint8_t>(1U << (b % 8));
    }
    f.descriptors.push_back(row);
  }
  for (int d = 0; d < 150; ++d) {
    f.keypoints.push_back({u(rng) * 639.0F, u(rng) * 479.0F, 31.0F, 0.0F, 0.5F});
    cv::Mat row(1, 32, CV_8UC1);
    for (int c = 0; c < 32; ++c) {
      row.at<uint8_t>(0, c) = static_cast<uint8_t>(bit(rng));
    }
    f.descriptors.push_back(row);
  }
  return f;
}

// Builds the map database once for every test of this file.
class LocalizerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    dir_ = new landfall::test::TempDir();
    world_ = new World();
    world_->room = landfall::test::makeRoom(4000, -3, -3, 13, 9, 7);
    landfall::BuildInput input;
    input.map.grid.width = 400;
    input.map.grid.height = 300;
    input.map.grid.resolution = 0.05;
    input.map.grid.origin = {-4.25, -4.25, 0.0};
    input.map.grid.data.assign(std::size_t{400} * 300, landfall::kCellFree);
    // An obstacle the free-space check must refuse (3.5 .. 4.5, 2.5 .. 3.5).
    for (int row = 135; row < 155; ++row) {
      for (int col = 155; col < 175; ++col) {
        input.map.grid.data[static_cast<std::size_t>(row) * 400 + col] = landfall::kCellOccupied;
      }
    }
    input.map.yaml_text = "image: room.pgm\n";
    input.map.yaml_sha256 = "yaml";
    input.map.image_sha256 = "image";
    input.sessions.push_back({"synthetic", "test", "tf_with_covariance", 0, ""});
    input.cameras.push_back({"camera_optical", world_->camera, world_->T_base_cam});
    landfall::Mat3 covariance = landfall::Mat3::Zero();
    covariance(0, 0) = covariance(1, 1) = 0.01 * 0.01;
    covariance(2, 2) = deg(0.2) * deg(0.2);
    const auto path = landfall::test::loopBothWays(0, 0, 10, 6, 1.0);
    for (std::size_t k = 0; k < path.size(); ++k) {
      landfall::BuildKeyframe keyframe;
      keyframe.stamp_ns = static_cast<int64_t>(k) * 1'000'000'000;
      keyframe.prior = path[k];
      keyframe.covariance = covariance;
      keyframe.T_base_cam = world_->T_base_cam;
      keyframe.features = observe(*world_, world_->T_base_cam, path[k], static_cast<int>(k));
      input.keyframes.push_back(keyframe);
    }
    auto config = landfall::parseBuildConfig("");
    config.builder.vocabulary.depth = 3;
    db_path_ = new std::string(dir_->file("room.landfall.db"));
    landfall::buildMap(input, config, *db_path_);
  }

  static void TearDownTestSuite()
  {
    delete db_path_;
    delete world_;
    delete dir_;
  }

  static landfall::QueryFrame query(const Pose2 & truth, const Pose3 & T_base_cam_used, int seed)
  {
    landfall::QueryFrame frame;
    frame.features = observe(*world_, T_base_cam_used, truth, seed);
    frame.camera = world_->camera;
    frame.T_base_cam = world_->T_base_cam;
    return frame;
  }

  static landfall::test::TempDir * dir_;
  static World * world_;
  static std::string * db_path_;
};

landfall::test::TempDir * LocalizerTest::dir_ = nullptr;
World * LocalizerTest::world_ = nullptr;
std::string * LocalizerTest::db_path_ = nullptr;

TEST_F(LocalizerTest, LoadsTheDatabase)
{
  const auto map = landfall::LocalizationMap::load(*db_path_);
  EXPECT_EQ(map->keyframes().size(), 64U);
  EXPECT_GT(map->numLandmarks(), 1000U);
  EXPECT_EQ(map->camera().width(), 640);
  EXPECT_EQ(map->descriptorType(), landfall::DescriptorType::kBinary);
  EXPECT_FALSE(map->contentSha256().empty());
  EXPECT_EQ(map->grid().width, 400);
  // Features come from the database once, then from the cache.
  const int64_t id = map->keyframes().front().id;
  const auto a = map->features(id);
  EXPECT_EQ(a.get(), map->features(id).get());
  EXPECT_EQ(a->features.keypoints.size(), a->landmarks.size());
  EXPECT_THROW(map->features(-5), std::runtime_error);
}

TEST_F(LocalizerTest, RefusesMissingAndForeignFiles)
{
  EXPECT_THROW(landfall::LocalizationMap::load(dir_->file("missing.db")), std::runtime_error);
  landfall::test::writeText(dir_->file("text.db"), "not a database");
  EXPECT_THROW(landfall::LocalizationMap::load(dir_->file("text.db")), std::runtime_error);
}

TEST_F(LocalizerTest, LocalizesPosesBetweenKeyframes)
{
  landfall::Localizer localizer(landfall::LocalizationMap::load(*db_path_), {});
  // Off the recorded loop: shifted along and across the path, turned by up to 8 degrees.
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> lateral(-0.3, 0.3);
  std::uniform_real_distribution<double> turn(-deg(8.0), deg(8.0));
  const auto path = landfall::test::loopBothWays(0, 0, 10, 6, 1.0);
  int localized = 0;
  int wrong = 0;
  double worst_position = 0.0;
  double worst_yaw = 0.0;
  for (std::size_t k = 0; k < path.size(); k += 2) {
    const Pose2 & p = path[k];
    const Pose2 truth = landfall::compose(p, {0.5, lateral(rng), turn(rng)});
    const auto result =
      localizer.localize(query(truth, world_->T_base_cam, 1000 + static_cast<int>(k)));
    ASSERT_FALSE(result.ambiguity.ambiguous) << "query " << k;
    if (!result.pose) {
      continue;
    }
    ++localized;
    const double position = landfall::translationDistance(*result.pose, truth);
    const double yaw = landfall::rotationDistance(*result.pose, truth);
    worst_position = std::max(worst_position, position);
    worst_yaw = std::max(worst_yaw, yaw);
    wrong += position > 0.25 ? 1 : 0;
    EXPECT_EQ(result.deepest_stage, landfall::Stage::kSanityPassed);
    ASSERT_GE(result.best, 0);
    EXPECT_GE(result.hypotheses[static_cast<std::size_t>(result.best)].sanity.num_inliers, 40);
  }
  RecordProperty("localized", localized);
  RecordProperty("worst_position_cm", std::to_string(100.0 * worst_position));
  RecordProperty("worst_yaw_deg", std::to_string(worst_yaw * 180.0 / M_PI));
  EXPECT_EQ(wrong, 0);
  EXPECT_EQ(localized, 32);
  EXPECT_LT(worst_position, 0.05);
  EXPECT_LT(worst_yaw, deg(0.5));
}

TEST_F(LocalizerTest, UnknownPlaceIsNoMatch)
{
  landfall::Localizer localizer(landfall::LocalizationMap::load(*db_path_), {});
  World elsewhere;
  elsewhere.room = landfall::test::makeRoom(4000, -3, -3, 13, 9, 99);  // other descriptors
  landfall::QueryFrame frame;
  frame.features = observe(elsewhere, elsewhere.T_base_cam, {2.0, 0.0, 0.0}, 5);
  frame.camera = world_->camera;
  frame.T_base_cam = world_->T_base_cam;
  const auto result = localizer.localize(frame);
  EXPECT_FALSE(result.pose);
  EXPECT_EQ(result.deepest_stage, landfall::Stage::kRetrieved);
  EXPECT_EQ(result.best, -1);
  for (const auto & h : result.hypotheses) {
    EXPECT_FALSE(h.rejection.empty());
  }
}

TEST_F(LocalizerTest, SanityRejectsWrongCameraHeight)
{
  // The camera really sits 0.3 m higher than TF says: vision solves the camera, and the base it
  // implies floats 0.3 m above the floor.
  landfall::Localizer localizer(landfall::LocalizationMap::load(*db_path_), {});
  Pose3 raised = world_->T_base_cam;
  raised.translation().z() += 0.3;
  const auto result = localizer.localize(query({4.0, 0.2, 0.05}, raised, 21));
  EXPECT_FALSE(result.pose);
  EXPECT_EQ(result.deepest_stage, landfall::Stage::kSolved);
  ASSERT_FALSE(result.hypotheses.empty());
  EXPECT_NE(result.hypotheses.front().rejection.find("height"), std::string::npos)
    << result.hypotheses.front().rejection;
}

TEST_F(LocalizerTest, SanityRejectsPosesInsideObstacles)
{
  // A camera 1 m back from a base pose inside the obstacle block: vision is right, the map says
  // a robot can't stand there.
  landfall::Localizer localizer(landfall::LocalizationMap::load(*db_path_), {});
  const auto result = localizer.localize(query({4.0, 3.0, 0.0}, world_->T_base_cam, 31));
  EXPECT_FALSE(result.pose);
  EXPECT_EQ(result.deepest_stage, landfall::Stage::kSolved);
  ASSERT_FALSE(result.hypotheses.empty());
  EXPECT_NE(result.hypotheses.front().rejection.find("free"), std::string::npos)
    << result.hypotheses.front().rejection;
}

TEST_F(LocalizerTest, SameResultWithAnyThreadCount)
{
  const auto map = landfall::LocalizationMap::load(*db_path_);
  landfall::LocalizerParams one;
  one.threads = 1;
  landfall::LocalizerParams four;
  four.threads = 4;
  landfall::Localizer a(map, one);
  landfall::Localizer b(map, four);
  const auto frame = query({6.0, 0.1, 0.02}, world_->T_base_cam, 41);
  const auto ra = a.localize(frame);
  const auto rb = b.localize(frame);
  ASSERT_TRUE(ra.pose);
  ASSERT_TRUE(rb.pose);
  EXPECT_EQ(ra.pose->x, rb.pose->x);
  EXPECT_EQ(ra.pose->y, rb.pose->y);
  EXPECT_EQ(ra.pose->yaw, rb.pose->yaw);
  EXPECT_EQ(ra.hypotheses.size(), rb.hypotheses.size());
}

TEST_F(LocalizerTest, RejectsAnEmptyImage)
{
  landfall::Localizer localizer(landfall::LocalizationMap::load(*db_path_), {});
  landfall::QueryFrame frame;
  frame.camera = world_->camera;
  EXPECT_THROW(localizer.localize(frame), std::invalid_argument);
}

}  // namespace
