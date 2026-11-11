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
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "landfall/build/map_builder.hpp"
#include "landfall/build/report.hpp"
#include "landfall/db/codec.hpp"
#include "landfall/db/database.hpp"
#include "landfall/grid/likelihood_field.hpp"
#include "landfall/grid/scan_matcher.hpp"
#include "landfall/grid/scan_verifier.hpp"
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

TEST(MapBuilder, ScansAndOdometryCorrectDriftingPriors)
{
  // The recording's localization drifts as AMCL does in a warehouse: a bias wandering up to 25 cm
  // and 1.5 deg plus 3 cm of noise, while reporting 2 cm. The keyframes carry their LiDAR scan
  // (the room's walls are in the map) and their odometry (3 mm noise per step).
  Scenario s = makeScenario(31);
  landfall::test::addRoomToGrid(&s.input.map.grid, -3.0, -3.0, 13.0, 9.0);
  BuildInput drifting = s.input;
  std::mt19937 rng(31);
  std::normal_distribution<double> noise(0.0, 0.03);
  std::normal_distribution<double> step(0.0, 0.003);
  landfall::Pose3 T_base_laser = landfall::Pose3::Identity();
  T_base_laser.translation() = Vec3(0.0, 0.0, 0.4);
  Pose2 odom{2.0, -1.0, 0.5};
  for (std::size_t k = 0; k < s.truth.size(); ++k) {
    const Pose2 & t = s.truth[k];
    const auto phase = static_cast<double>(k);
    auto & keyframe = drifting.keyframes[k];
    keyframe.prior = {
      t.x + 0.25 * std::sin(phase / 7.0) + noise(rng),
      t.y + 0.15 * std::cos(phase / 11.0) + noise(rng), t.yaw + deg(1.5) * std::sin(phase / 5.0)};
    if (k > 0) {
      Pose2 motion = landfall::between(s.truth[k - 1], t);
      motion.x += step(rng);
      motion.y += step(rng);
      odom = landfall::compose(odom, motion);
    }
    keyframe.odom = odom;
  }
  // The scans come from the true surfaces, which lie inside the map's occupied cells (as in a map
  // made from the same LiDAR): the walls of a 1 cm grid, at the centres of the 5 cm map's
  // innermost occupied cells.
  landfall::OccupancyGrid surfaces;
  surfaces.resolution = 0.01;
  surfaces.origin = s.input.map.grid.origin;
  surfaces.width = 2000;
  surfaces.height = 1500;
  surfaces.data.assign(std::size_t{2000} * 1500, landfall::kCellFree);
  landfall::test::addRoomToGrid(&surfaces, -3.0 + 0.015, -3.0 + 0.015, 13.0 - 0.015, 9.0 - 0.015);
  BuildInput with_scans = drifting;
  for (std::size_t k = 0; k < s.truth.size(); ++k) {
    with_scans.keyframes[k].scan = landfall::test::simulateScan(
      surfaces, landfall::toPose3(s.truth[k]), T_base_laser, static_cast<int>(k), 360, 10.0, 0.01);
  }
  for (auto & keyframe : drifting.keyframes) {
    keyframe.odom.reset();  // the recorded poses alone, as before scans and odometry
  }

  landfall::test::TempDir dir;
  const auto plain = landfall::buildMap(drifting, config(0), dir.file("plain.landfall.db"));
  const auto fixed = landfall::buildMap(with_scans, config(0), dir.file("fixed.landfall.db"));
  EXPECT_EQ(plain.poses_scan_matched, 0);
  EXPECT_EQ(fixed.poses_scan_matched, static_cast<int>(s.truth.size()));
  EXPECT_EQ(fixed.poses_scan_unmatched, 0);
  // Every step is linked but the jump from the end of the first loop to the start of the second.
  EXPECT_EQ(fixed.poses_odometry_links, static_cast<int>(s.truth.size()) - 2);

  // The scan matches alone, as the builder computes them.
  {
    const auto field = std::make_shared<const landfall::LikelihoodField>(s.input.map.grid);
    landfall::ScanCheckParams projection;
    projection.max_range = 100.0;
    projection.check_alternatives = false;
    const landfall::ScanVerifier projector(field, projection);
    double sum = 0.0;
    double worst = 0.0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (std::size_t k = 0; k < s.truth.size(); ++k) {
      auto scan = *with_scans.keyframes[k].scan;
      scan.range_max -= 1e-3;
      const auto projected = projector.project(scan, Vec3(0.0, 0.0, -1.0));
      const auto match = landfall::matchScan(
        *field, projected.endpoints, with_scans.keyframes[k].prior, landfall::ScanMatchParams());
      const double e = landfall::translationDistance(match.pose, s.truth[k]);
      sum += e;
      worst = std::max(worst, e);
      sum_x += match.pose.x - s.truth[k].x;
      sum_y += match.pose.y - s.truth[k].y;
    }
    const auto count = static_cast<double>(s.truth.size());
    RecordProperty("scan_mean_error_mm", std::to_string(1000.0 * sum / count));
    RecordProperty("scan_worst_error_mm", std::to_string(1000.0 * worst));
    RecordProperty(
      "scan_mean_offset_mm",
      std::to_string(1000.0 * sum_x / count) + ", " + std::to_string(1000.0 * sum_y / count));
    EXPECT_LT(sum / count, 0.03);
    EXPECT_LT(worst, 0.06);
  }
  EXPECT_TRUE(fixed.ba_usable);

  const auto error = [&](const std::string & path) {
    const auto db = landfall::db::Database::open(path, landfall::db::Database::Access::kReadOnly);
    double sum = 0.0;
    double worst = 0.0;
    for (const auto & k : db.keyframes()) {
      const double e =
        landfall::translationDistance(k.refined, s.truth[static_cast<std::size_t>(k.id - 1)]);
      sum += e;
      worst = std::max(worst, e);
    }
    return std::make_pair(sum / static_cast<double>(s.truth.size()), worst);
  };
  double prior_error = 0.0;
  for (std::size_t k = 0; k < s.truth.size(); ++k) {
    prior_error += landfall::translationDistance(drifting.keyframes[k].prior, s.truth[k]);
  }
  prior_error /= static_cast<double>(s.truth.size());
  const auto [plain_mean, plain_worst] = error(dir.file("plain.landfall.db"));
  const auto [fixed_mean, fixed_worst] = error(dir.file("fixed.landfall.db"));
  RecordProperty("mean_prior_error_mm", std::to_string(1000.0 * prior_error));
  RecordProperty("plain_mean_error_mm", std::to_string(1000.0 * plain_mean));
  RecordProperty("fixed_mean_error_mm", std::to_string(1000.0 * fixed_mean));
  RecordProperty("fixed_worst_error_mm", std::to_string(1000.0 * fixed_worst));
  RecordProperty("plain_landmarks", std::to_string(plain.landmarks));
  RecordProperty("fixed_landmarks", std::to_string(fixed.landmarks));
  RecordProperty("plain_rms_px", std::to_string(plain.reprojection_rms_px));
  RecordProperty("fixed_rms_px", std::to_string(fixed.reprojection_rms_px));
  // A 5 cm map pins a scan to a couple of centimetres (two-cell walls are flat across their
  // thickness); the adjustment with odometry and vision keeps that, far below the drift.
  EXPECT_GT(prior_error, 0.1);
  EXPECT_LT(fixed_mean, 0.03);
  EXPECT_LT(fixed_worst, 0.06);
  EXPECT_LT(fixed_mean, 0.25 * plain_mean);
  EXPECT_LT(fixed.reprojection_rms_px, 0.8 * plain.reprojection_rms_px);

  // Flags and the report tell the story.
  const auto db = landfall::db::Database::open(
    dir.file("fixed.landfall.db"), landfall::db::Database::Access::kReadOnly);
  for (const auto & k : db.keyframes()) {
    EXPECT_NE(k.flags & landfall::db::KeyframeRecord::kFlagScanMatched, 0U) << k.id;
  }
  const auto report = landfall::makeReport(db);
  EXPECT_NE(report.markdown.find("| Matched to the map with their scan | "), std::string::npos);
}
