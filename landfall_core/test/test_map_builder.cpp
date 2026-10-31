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
#include <filesystem>
#include <map>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "landfall/build/map_builder.hpp"
#include "landfall/build/report.hpp"
#include "landfall/db/codec.hpp"
#include "landfall/db/database.hpp"
#include "synthetic_map.hpp"
#include "test_utils.hpp"

using landfall::BuildInput;
using landfall::Features;
using landfall::Mat3;
using landfall::Pose2;
using landfall::Vec2;
using landfall::Vec3;
using landfall::test::deg;

namespace
{

// The synthetic room seen from a loop driven both ways. Each keyframe's features are the
// projections (0.5 px noise) of the visible wall landmarks, with their descriptor and a few
// flipped bits, plus random distractors. Priors carry 2 cm / 0.4 deg noise.
struct Scenario
{
  BuildInput input;
  std::vector<Pose2> truth;
  landfall::test::SyntheticRoom room;
  /// Per keyframe: the true landmark of each keypoint, -1 for distractors.
  std::vector<std::vector<int>> truth_ids;
};

Scenario makeScenario(int seed)
{
  Scenario s;
  s.room = landfall::test::makeRoom(4000, -3, -3, 13, 9, seed);
  s.truth = landfall::test::loopBothWays(0, 0, 10, 6, 0.5);
  const auto camera = landfall::test::syntheticCamera();
  const auto T_base_cam = landfall::test::baseToCamera();
  std::mt19937 rng(seed);
  std::normal_distribution<double> pixel(0.0, 0.5);
  std::normal_distribution<double> xy(0.0, 0.02);
  std::normal_distribution<double> yaw(0.0, deg(0.4));
  std::uniform_int_distribution<int> bit(0, 255);
  std::uniform_real_distribution<float> u(0.0F, 1.0F);

  s.input.map.grid.width = 400;
  s.input.map.grid.height = 300;
  s.input.map.grid.resolution = 0.05;
  // Off the 1 m report cells' boundaries, so pose noise doesn't split a cell's two directions.
  s.input.map.grid.origin = {-4.25, -4.25, 0.0};
  s.input.map.grid.data.assign(std::size_t{400} * 300, landfall::kCellFree);
  s.input.map.yaml_text = "image: room.pgm\n";
  s.input.map.yaml_sha256 = "yaml";
  s.input.map.image_sha256 = "image";
  s.input.sessions.push_back({"synthetic", "test", "tf_with_covariance", 0, ""});
  s.input.cameras.push_back({"camera_optical", camera, T_base_cam});

  Mat3 covariance = Mat3::Zero();
  covariance(0, 0) = covariance(1, 1) = 0.02 * 0.02;
  covariance(2, 2) = deg(0.4) * deg(0.4);
  for (std::size_t k = 0; k < s.truth.size(); ++k) {
    const Pose2 & t = s.truth[k];
    const auto T_cam_map = landfall::cameraPose(t, T_base_cam).inverse();
    Features f;
    f.type = landfall::DescriptorType::kBinary;
    cv::Mat descriptors;
    std::vector<int> ids;
    for (std::size_t l = 0; l < s.room.landmarks.size(); ++l) {
      Vec2 px;
      if (!landfall::test::projectPoint(camera, T_cam_map, s.room.landmarks[l], &px)) {
        continue;
      }
      px += Vec2(pixel(rng), pixel(rng));
      f.keypoints.push_back(
        {static_cast<float>(px.x()), static_cast<float>(px.y()), 31.0F, 0.0F, 1.0F});
      cv::Mat row = s.room.descriptors.row(static_cast<int>(l)).clone();
      for (int flip = 0; flip < 6; ++flip) {
        const int b = bit(rng);
        row.at<uint8_t>(0, b / 8) ^= static_cast<uint8_t>(1U << (b % 8));
      }
      descriptors.push_back(row);
      ids.push_back(static_cast<int>(l));
    }
    for (int d = 0; d < 150; ++d) {  // distractors
      f.keypoints.push_back({u(rng) * 639.0F, u(rng) * 479.0F, 31.0F, 0.0F, 0.5F});
      cv::Mat row(1, 32, CV_8UC1);
      cv::randu(row, 0, 256);
      descriptors.push_back(row);
      ids.push_back(-1);
    }
    f.descriptors = descriptors;
    s.truth_ids.push_back(ids);
    landfall::BuildKeyframe keyframe;
    keyframe.stamp_ns = static_cast<int64_t>(k) * 1'000'000'000;
    keyframe.prior = {t.x + xy(rng), t.y + xy(rng), t.yaw + yaw(rng)};
    keyframe.covariance = covariance;
    keyframe.T_base_cam = T_base_cam;
    keyframe.features = f;
    s.input.keyframes.push_back(keyframe);
  }
  return s;
}

landfall::BuildConfigFile config(int threads)
{
  landfall::BuildConfigFile c = landfall::parseBuildConfig("");
  c.builder.threads = threads;
  c.builder.vocabulary.depth = 3;  // ~200 k descriptors would make depth 5 sparse
  return c;
}

}  // namespace

TEST(MapBuilder, SyntheticLoopBuildsAccurateDeterministicMap)
{
  cv::setRNGSeed(1);
  const Scenario s = makeScenario(21);
  landfall::test::TempDir dir;
  const std::string a = dir.file("a.landfall.db");
  const std::string b = dir.file("b.landfall.db");
  std::vector<std::string> stages;
  const auto stats = landfall::buildMap(
    s.input, config(1), a, [&](landfall::BuildStage stage, std::size_t done, std::size_t total) {
      if (done == total) {
        stages.emplace_back(landfall::toString(stage));
      }
    });
  RecordProperty("landmarks", std::to_string(stats.landmarks));
  RecordProperty("mean_track_length", std::to_string(stats.mean_track_length));
  RecordProperty("reprojection_rms_px", std::to_string(stats.reprojection_rms_px));
  RecordProperty("ba_max_shift_m", std::to_string(stats.ba_max_shift_m));
  EXPECT_EQ(stats.keyframes, static_cast<int>(s.truth.size()));
  EXPECT_GT(stats.pairs_verified, stats.keyframes * 5);
  EXPECT_GT(stats.landmarks, 1500);
  EXPECT_GE(stats.mean_track_length, 3.5);
  EXPECT_LE(stats.reprojection_rms_px, 1.0);
  EXPECT_TRUE(stats.ba_usable);
  EXPECT_LE(stats.ba_max_shift_m, 0.10 + 1e-9);
  EXPECT_TRUE(stats.warnings.empty()) << stats.warnings.front();
  EXPECT_EQ(stages.back(), "writing");

  // Same input, another thread count: the same database content.
  landfall::buildMap(s.input, config(4), b);
  const auto db_a = landfall::db::Database::open(a, landfall::db::Database::Access::kReadOnly);
  const auto db_b = landfall::db::Database::open(b, landfall::db::Database::Access::kReadOnly);
  EXPECT_EQ(db_a.contentSha256(), db_b.contentSha256());
  EXPECT_EQ(db_a.meta("content_sha256").value_or(""), db_a.contentSha256());
  EXPECT_EQ(db_a.integrityCheck(), "ok");

  // BA brings the keyframes closer to the truth than the recorded priors.
  const auto keyframes = db_a.keyframes();
  ASSERT_EQ(keyframes.size(), s.truth.size());
  double prior_error = 0.0;
  double refined_error = 0.0;
  for (std::size_t k = 0; k < keyframes.size(); ++k) {
    prior_error += landfall::translationDistance(keyframes[k].prior, s.truth[k]);
    refined_error += landfall::translationDistance(keyframes[k].refined, s.truth[k]);
  }
  const auto count = static_cast<double>(keyframes.size());
  RecordProperty("mean_prior_error_mm", std::to_string(1000.0 * prior_error / count));
  RecordProperty("mean_refined_error_mm", std::to_string(1000.0 * refined_error / count));
  EXPECT_LT(refined_error, 0.6 * prior_error);

  // Landmark accuracy against the true landmark each one was built from (found through the pixels
  // of its observations, which pruning keeps), and wrong associations.
  const auto backends = db_a.backends();
  int64_t local_id = 0;
  for (const auto & backend : backends) {
    if (backend.role == "local") {
      local_id = backend.id;
    }
  }
  ASSERT_GT(local_id, 0);
  std::map<int64_t, std::size_t> keyframe_index;
  for (std::size_t k = 0; k < keyframes.size(); ++k) {
    keyframe_index[keyframes[k].id] = k;
  }
  std::map<int64_t, std::vector<landfall::db::ObservationRecord>> by_landmark;
  for (const auto & o : db_a.allObservations()) {
    by_landmark[o.landmark_id].push_back(o);
  }
  std::map<int64_t, landfall::db::LocalFeaturesRecord> stored;
  for (const auto & k : keyframes) {
    stored[k.id] = db_a.localFeatures(k.id, local_id).value();
  }
  std::vector<double> errors;
  int wrong = 0;
  for (const auto & l : db_a.landmarks()) {
    std::set<int> sources;
    for (const auto & o : by_landmark[l.id]) {
      const std::size_t k = keyframe_index.at(o.keyframe_id);
      const auto & kp = stored.at(o.keyframe_id).keypoints[o.keypoint_index];
      const auto & original = s.input.keyframes[k].features->keypoints;
      for (std::size_t i = 0; i < original.size(); ++i) {
        if (original[i].x == kp.x && original[i].y == kp.y) {
          sources.insert(s.truth_ids[k][i]);
          break;
        }
      }
    }
    if (sources.size() != 1 || *sources.begin() < 0) {
      ++wrong;
      continue;
    }
    errors.push_back((s.room.landmarks[*sources.begin()] - l.position).norm());
  }
  ASSERT_FALSE(errors.empty());
  std::nth_element(
    errors.begin(), errors.begin() + static_cast<std::ptrdiff_t>(errors.size() / 2), errors.end());
  RecordProperty("median_landmark_error_mm", std::to_string(1000.0 * errors[errors.size() / 2]));
  RecordProperty("wrong_associations", std::to_string(wrong));
  EXPECT_LT(errors[errors.size() / 2], 0.05);
  EXPECT_LT(wrong, stats.landmarks / 100);

  // Pruned features: every stored keypoint observes a landmark, and indices are consistent.
  const auto record = db_a.localFeatures(keyframes[3].id, local_id);
  ASSERT_TRUE(record.has_value());
  const auto observations = db_a.observations(keyframes[3].id);
  EXPECT_EQ(observations.size(), record->keypoints.size());
  for (std::size_t i = 0; i < observations.size(); ++i) {
    EXPECT_EQ(observations[i].keypoint_index, static_cast<int>(i));
  }
  EXPECT_EQ(
    landfall::db::decodeLocalFeatures(*record).descriptors.rows,
    static_cast<int>(observations.size()));
  EXPECT_FALSE(db_a.covisibility().empty());
  ASSERT_TRUE(db_a.meta("builder_config").has_value());
  EXPECT_NE(db_a.meta("build_report")->find("\"landmarks\":"), std::string::npos);

  // Report: summary numbers agree with the builder's, files are written.
  const auto report = landfall::makeReport(db_a);
  EXPECT_EQ(report.summary.landmarks, stats.landmarks);
  EXPECT_NEAR(report.summary.mean_track_length, stats.mean_track_length, 1e-9);
  // The loop is driven both ways: two opposite sectors per cell (corners aside).
  EXPECT_GT(report.summary.both_directions_fraction, 0.8);
  EXPECT_NEAR(report.summary.heading_sector_fraction, 0.25, 0.05);
  EXPECT_NE(
    report.markdown.find("| Landmarks | " + std::to_string(stats.landmarks)), std::string::npos);
  landfall::writeReport(report, dir.file("report"));
  EXPECT_TRUE(std::filesystem::exists(dir.file("report/coverage.png")));
  EXPECT_TRUE(std::filesystem::exists(dir.file("report/report.json")));
}

TEST(MapBuilder, RejectsInvalidInput)
{
  Scenario s = makeScenario(5);
  landfall::test::TempDir dir;
  BuildInput two_cameras = s.input;
  two_cameras.cameras.push_back(two_cameras.cameras.front());
  EXPECT_THROW(landfall::buildMap(two_cameras, config(1), dir.file("x.db")), std::runtime_error);
  BuildInput one = s.input;
  one.keyframes.resize(1);
  EXPECT_THROW(landfall::buildMap(one, config(1), dir.file("y.db")), std::runtime_error);
  BuildInput no_image = s.input;
  no_image.keyframes[2].features.reset();
  EXPECT_THROW(landfall::buildMap(no_image, config(1), dir.file("z.db")), std::runtime_error);
  // Never overwrites an existing database.
  landfall::test::writeText(dir.file("exists.db"), "x");
  EXPECT_THROW(landfall::buildMap(s.input, config(1), dir.file("exists.db")), std::runtime_error);
}
