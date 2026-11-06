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
#include "landfall/geometry/gravity.hpp"
#include "landfall/query/localization_map.hpp"
#include "landfall/query/localizer.hpp"
#include "synthetic_map.hpp"
#include "test_utils.hpp"

using landfall::Features;
using landfall::Pose2;
using landfall::Pose3;
using landfall::Vec2;
using landfall::Vec3;
using landfall::test::deg;

namespace
{

constexpr double kX0 = -3.0;
constexpr double kY0 = -3.0;
constexpr double kX1 = 13.0;
constexpr double kY1 = 9.0;

// The synthetic room of the map-builder test (wall landmarks, a walled occupancy grid), possibly
// repeated: every copy has the same landmarks and descriptors, shifted by its offset.
struct World
{
  landfall::test::SyntheticRoom room;
  std::vector<Vec2> copies{Vec2::Zero()};
  landfall::CameraModel camera = landfall::test::syntheticCamera();
  Pose3 T_base_cam = landfall::test::baseToCamera();
  Pose3 T_base_laser = [] {
    Pose3 T = Pose3::Identity();
    T.translation() = Vec3(0.1, 0.0, 0.3);
    return T;
  }();
  landfall::OccupancyGrid grid;
};

World makeWorld(const std::vector<Vec2> & copies, int width_cells)
{
  World world;
  world.room = landfall::test::makeRoom(4000, kX0, kY0, kX1, kY1, 7);
  world.copies = copies;
  world.grid.width = width_cells;
  world.grid.height = 300;
  world.grid.resolution = 0.05;
  world.grid.origin = {-4.25, -4.25, 0.0};
  world.grid.data.assign(std::size_t(width_cells) * 300, landfall::kCellUnknown);
  for (const Vec2 & c : copies) {
    landfall::test::addRoomToGrid(&world.grid, kX0 + c.x(), kY0 + c.y(), kX1 + c.x(), kY1 + c.y());
  }
  // An obstacle in the first room the free-space check must refuse (3.5 .. 4.5, 2.5 .. 3.5).
  for (int row = 135; row < 155; ++row) {
    for (int col = 155; col < 175; ++col) {
      world.grid.data[static_cast<std::size_t>(row) * width_cells + col] = landfall::kCellOccupied;
    }
  }
  return world;
}

// Projections (0.5 px noise) of the visible landmarks with a few flipped descriptor bits, plus
// random distractors. The rooms are closed: a camera sees the walls of the room it is in only.
Features observe(const World & world, const Pose3 & T_map_cam, int seed)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, 0.5);
  std::uniform_int_distribution<int> bit(0, 255);
  std::uniform_real_distribution<float> u(0.0F, 1.0F);
  const Pose3 T_cam_map = T_map_cam.inverse();
  const Vec3 center = T_map_cam.translation();
  Features f;
  f.type = landfall::DescriptorType::kBinary;
  for (const Vec2 & copy : world.copies) {
    if (
      center.x() < kX0 + copy.x() || center.x() > kX1 + copy.x() || center.y() < kY0 + copy.y() ||
      center.y() > kY1 + copy.y()) {
      continue;
    }
    for (std::size_t l = 0; l < world.room.landmarks.size(); ++l) {
      Vec2 px;
      const Vec3 X = world.room.landmarks[l] + Vec3(copy.x(), copy.y(), 0.0);
      if (!landfall::test::projectPoint(world.camera, T_cam_map, X, &px)) {
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

std::string buildDatabase(
  const World & world, const std::vector<Pose2> & path, const std::string & db_path)
{
  landfall::BuildInput input;
  input.map.grid = world.grid;
  input.map.yaml_text = "image: room.pgm\n";
  input.map.yaml_sha256 = "yaml";
  input.map.image_sha256 = "image";
  input.sessions.push_back({"synthetic", "test", "tf_with_covariance", 0, ""});
  input.cameras.push_back({"camera_optical", world.camera, world.T_base_cam});
  landfall::Mat3 covariance = landfall::Mat3::Zero();
  covariance(0, 0) = covariance(1, 1) = 0.01 * 0.01;
  covariance(2, 2) = deg(0.2) * deg(0.2);
  for (std::size_t k = 0; k < path.size(); ++k) {
    landfall::BuildKeyframe keyframe;
    keyframe.stamp_ns = static_cast<int64_t>(k) * 1'000'000'000;
    keyframe.prior = path[k];
    keyframe.covariance = covariance;
    keyframe.T_base_cam = world.T_base_cam;
    keyframe.features =
      observe(world, landfall::cameraPose(path[k], world.T_base_cam), static_cast<int>(k));
    input.keyframes.push_back(keyframe);
  }
  auto config = landfall::parseBuildConfig("");
  config.builder.vocabulary.depth = 3;
  landfall::buildMap(input, config, db_path);
  return db_path;
}

// A query: the camera sees the world from `seen_from`, the LiDAR scans it from `scanned_from` (the
// same pose unless a test makes them disagree), both on a flat floor unless the base is tilted.
landfall::QueryFrame query(
  const World & world, const Pose3 & T_map_base_seen, const Pose3 & T_map_base_scanned, int seed,
  const Pose3 & T_base_cam_used)
{
  landfall::QueryFrame frame;
  frame.features = observe(world, T_map_base_seen * T_base_cam_used, seed);
  frame.camera = world.camera;
  frame.T_base_cam = world.T_base_cam;
  frame.scan =
    landfall::test::simulateScan(world.grid, T_map_base_scanned, world.T_base_laser, seed + 7);
  return frame;
}

landfall::QueryFrame query(const World & world, const Pose2 & truth, int seed)
{
  return query(world, landfall::toPose3(truth), landfall::toPose3(truth), seed, world.T_base_cam);
}

// Builds the one-room database once for the tests of this fixture.
class LocalizerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    dir_ = new landfall::test::TempDir();
    world_ = new World(makeWorld({Vec2::Zero()}, 400));
    db_path_ = new std::string(buildDatabase(
      *world_, landfall::test::loopBothWays(0, 0, 10, 6, 1.0), dir_->file("room.landfall.db")));
  }

  static void TearDownTestSuite()
  {
    delete db_path_;
    delete world_;
    delete dir_;
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
    const auto result = localizer.localize(query(*world_, truth, 1000 + static_cast<int>(k)));
    ASSERT_FALSE(result.ambiguity.ambiguous) << "query " << k;
    EXPECT_TRUE(result.scan_checked);
    if (!result.pose) {
      continue;
    }
    ++localized;
    const double position = landfall::translationDistance(*result.pose, truth);
    const double yaw = landfall::rotationDistance(*result.pose, truth);
    worst_position = std::max(worst_position, position);
    worst_yaw = std::max(worst_yaw, yaw);
    wrong += position > 0.25 ? 1 : 0;
    EXPECT_EQ(result.deepest_stage, landfall::Stage::kScanPassed);
    ASSERT_GE(result.best, 0);
    const auto & best = result.hypotheses[static_cast<std::size_t>(result.best)];
    EXPECT_GE(best.sanity.num_inliers, 40);
    EXPECT_GE(best.scan.score.hit_ratio, 0.9);
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
  World elsewhere = *world_;
  elsewhere.room = landfall::test::makeRoom(4000, kX0, kY0, kX1, kY1, 99);  // other descriptors
  const auto result = localizer.localize(query(elsewhere, Pose2{2.0, 0.0, 0.0}, 5));
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
  const Pose3 truth = landfall::toPose3(Pose2{4.0, 0.2, 0.05});
  const auto result = localizer.localize(query(*world_, truth, truth, 21, raised));
  EXPECT_FALSE(result.pose);
  EXPECT_EQ(result.deepest_stage, landfall::Stage::kSolved);
  ASSERT_FALSE(result.hypotheses.empty());
  EXPECT_NE(result.hypotheses.front().rejection.find("height"), std::string::npos)
    << result.hypotheses.front().rejection;
}

TEST_F(LocalizerTest, SanityRejectsPosesInsideObstacles)
{
  // A base pose inside the obstacle block: vision is right, the map says a robot can't stand there.
  landfall::Localizer localizer(landfall::LocalizationMap::load(*db_path_), {});
  const auto result = localizer.localize(query(*world_, Pose2{4.0, 3.0, 0.0}, 31));
  EXPECT_FALSE(result.pose);
  EXPECT_EQ(result.deepest_stage, landfall::Stage::kSolved);
  ASSERT_FALSE(result.hypotheses.empty());
  EXPECT_NE(result.hypotheses.front().rejection.find("free"), std::string::npos)
    << result.hypotheses.front().rejection;
}

TEST_F(LocalizerTest, ScanRejectsAPlaceThatOnlyLooksRight)
{
  // The camera sees place A, the LiDAR scans from B, 0.5 to 5 m away (and up to 15 degrees
  // turned): vision proposes A, and the scan must refuse it every time.
  landfall::Localizer localizer(landfall::LocalizationMap::load(*db_path_), {});
  const landfall::LikelihoodField field(world_->grid);
  std::mt19937 rng(17);
  std::uniform_real_distribution<double> distance(0.5, 5.0);
  std::uniform_real_distribution<double> direction(-M_PI, M_PI);
  std::uniform_real_distribution<double> turn(-deg(15.0), deg(15.0));
  const auto path = landfall::test::loopBothWays(0, 0, 10, 6, 1.0);
  int vision_found = 0;
  int published = 0;
  for (int trial = 0; trial < 40; ++trial) {
    const Pose2 seen = path[static_cast<std::size_t>(trial * 3) % path.size()];
    Pose2 scanned;
    do {
      const double d = distance(rng);
      const double a = direction(rng);
      scanned = {seen.x + d * std::cos(a), seen.y + d * std::sin(a), seen.yaw + turn(rng)};
    } while (field.classify(scanned.x, scanned.y) != landfall::LikelihoodField::Cell::kFree ||
             field.distance(scanned.x, scanned.y) < 0.5);
    const auto result = localizer.localize(query(
      *world_, landfall::toPose3(seen), landfall::toPose3(scanned), 300 + trial,
      world_->T_base_cam));
    vision_found += result.deepest_stage == landfall::Stage::kSanityPassed ? 1 : 0;
    published += result.pose ? 1 : 0;
  }
  EXPECT_EQ(published, 0);
  EXPECT_GE(vision_found, 36);  // the scan, not vision, is what refused them
}

TEST_F(LocalizerTest, ScanModes)
{
  const Pose2 truth{6.0, 0.1, 0.02};
  auto frame = query(*world_, truth, 41);
  const auto map = landfall::LocalizationMap::load(*db_path_);
  // Required: no scan, no pose.
  landfall::Localizer required(map, {});
  auto no_scan = frame;
  no_scan.scan.reset();
  auto r = required.localize(no_scan);
  EXPECT_FALSE(r.pose);
  ASSERT_FALSE(r.hypotheses.empty());
  EXPECT_NE(r.hypotheses.front().rejection.find("no LiDAR scan"), std::string::npos);
  // Optional: checked when there is a scan, vision only otherwise.
  landfall::LocalizerParams optional;
  optional.scan_mode = landfall::ScanCheckMode::kOptional;
  landfall::Localizer optional_localizer(map, optional);
  r = optional_localizer.localize(frame);
  ASSERT_TRUE(r.pose);
  EXPECT_TRUE(r.scan_checked);
  r = optional_localizer.localize(no_scan);
  ASSERT_TRUE(r.pose);
  EXPECT_FALSE(r.scan_checked);
  EXPECT_EQ(r.deepest_stage, landfall::Stage::kSanityPassed);
  // Disabled: a scan from elsewhere doesn't matter.
  landfall::LocalizerParams disabled;
  disabled.scan_mode = landfall::ScanCheckMode::kDisabled;
  landfall::Localizer disabled_localizer(map, disabled);
  auto elsewhere = query(
    *world_, landfall::toPose3(truth), landfall::toPose3(Pose2{2.0, 4.0, 1.0}), 43,
    world_->T_base_cam);
  r = disabled_localizer.localize(elsewhere);
  ASSERT_TRUE(r.pose);
  EXPECT_LT(landfall::translationDistance(*r.pose, truth), 0.05);
  EXPECT_FALSE(landfall::parseScanCheckMode("sometimes"));
}

TEST_F(LocalizerTest, RampQueriesAreRightOrRefused)
{
  // On a 6 degree ramp, 0.2 m up: the IMU knows the tilt, the camera and the laser are tilted, and
  // beams aimed down hit the floor.
  const auto map = landfall::LocalizationMap::load(*db_path_);
  landfall::LocalizerParams allow;
  allow.sanity.max_floor_elevation = 0.3;
  landfall::Localizer refusing(map, {});
  landfall::Localizer accepting(map, allow);
  const auto path = landfall::test::loopBothWays(0, 0, 10, 6, 1.0);
  int refused_by_default = 0;
  int localized = 0;
  int wrong = 0;
  for (std::size_t k = 1; k < path.size(); k += 4) {
    const Pose2 & p = path[k];
    Pose3 T_map_base = landfall::test::basePose(p.x, p.y, p.yaw, -deg(6.0));
    T_map_base.translation().z() = 0.2;
    auto frame =
      query(*world_, T_map_base, T_map_base, 500 + static_cast<int>(k), world_->T_base_cam);
    landfall::ImuSample imu;
    imu.orientation = Eigen::Quaterniond(T_map_base.linear());
    const auto gravity = landfall::estimateGravity(
      landfall::GravitySource::kImu, imu, Eigen::Quaterniond::Identity(), {});
    ASSERT_TRUE(gravity);
    ASSERT_TRUE(gravity->on_ramp);
    frame.gravity = *gravity;
    const auto by_default = refusing.localize(frame);
    refused_by_default += by_default.pose ? 0 : 1;
    const auto result = accepting.localize(frame);
    if (result.pose) {
      ++localized;
      wrong += landfall::translationDistance(*result.pose, p) > 0.25 ? 1 : 0;
    }
  }
  EXPECT_EQ(refused_by_default, 16);  // sanity.max_floor_elevation 0 refuses ramp queries
  EXPECT_EQ(wrong, 0);
  EXPECT_GE(localized, 12);
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
  const auto frame = query(*world_, Pose2{6.0, 0.1, 0.02}, 41);
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

// Two identical rooms 20 m apart, both mapped: every image and every scan fits both.
class DuplicatedRoomTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    dir_ = new landfall::test::TempDir();
    world_ = new World(makeWorld({Vec2::Zero(), Vec2(20.0, 0.0)}, 800));
    // Only the first room's copy of the obstacle is in the grid: keep the queries away from it.
    auto path = landfall::test::loopBothWays(0, 0, 10, 6, 1.0);
    const auto second = landfall::test::loopBothWays(20, 0, 30, 6, 1.0);
    path.insert(path.end(), second.begin(), second.end());
    db_path_ = new std::string(buildDatabase(*world_, path, dir_->file("twins.landfall.db")));
  }

  static void TearDownTestSuite()
  {
    delete db_path_;
    delete world_;
    delete dir_;
  }

  static landfall::test::TempDir * dir_;
  static World * world_;
  static std::string * db_path_;
};

landfall::test::TempDir * DuplicatedRoomTest::dir_ = nullptr;
World * DuplicatedRoomTest::world_ = nullptr;
std::string * DuplicatedRoomTest::db_path_ = nullptr;

TEST_F(DuplicatedRoomTest, IsAmbiguousAndNeverWrong)
{
  landfall::Localizer localizer(landfall::LocalizationMap::load(*db_path_), {});
  // The obstacle block makes the first room's grid differ from the second's around (4, 3): stay on
  // the loop, where both rooms look and scan alike.
  const auto path = landfall::test::loopBothWays(0, 0, 10, 6, 1.0);
  int ambiguous = 0;
  int wrong = 0;
  int queries = 0;
  for (std::size_t k = 0; k < path.size(); k += 4) {
    for (const double offset : {0.0, 20.0}) {
      const Pose2 truth = landfall::compose({offset, 0.0, 0.0}, path[k]);
      const auto result = localizer.localize(query(*world_, truth, 700 + static_cast<int>(k)));
      ++queries;
      ambiguous += result.ambiguity.ambiguous ? 1 : 0;
      if (result.pose && !result.ambiguity.ambiguous) {
        wrong += landfall::translationDistance(*result.pose, truth) > 0.25 ? 1 : 0;
      }
    }
  }
  RecordProperty("ambiguous", ambiguous);
  EXPECT_EQ(wrong, 0);
  EXPECT_GE(ambiguous, queries * 9 / 10);
}

}  // namespace
