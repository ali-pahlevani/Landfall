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

// The depot fixture (test/fixtures/depot_small, a cut of the Gazebo recording) through the whole
// offline pipeline: bag -> keyframes -> map database, against the recording's ground truth.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "landfall/build/map_builder.hpp"
#include "landfall/db/database.hpp"
#include "landfall/eval/trajectory.hpp"
#include "landfall/ros/bag_reader.hpp"
#include "landfall/ros/conversions.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"

namespace
{

namespace fs = std::filesystem;
using landfall::Pose2;
using landfall::Vec2;

const std::string kFixture = LANDFALL_FIXTURE_DIR;

class TempDir
{
public:
  TempDir()
  {
    std::string pattern = (fs::temp_directory_path() / "landfall_fixture_XXXXXX").string();
    if (mkdtemp(pattern.data()) == nullptr) {
      throw std::runtime_error("mkdtemp failed");
    }
    path_ = pattern;
  }
  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;
  ~TempDir() { fs::remove_all(path_); }
  std::string file(const std::string & name) const { return (path_ / name).string(); }

private:
  fs::path path_;
};

std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

landfall::eval::Trajectory groundTruth(const std::string & bag)
{
  landfall::eval::Trajectory truth;
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {"/ground_truth/pose"};
  reader.set_filter(filter);
  const rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    const rclcpp::SerializedMessage serialized(*message->serialized_data);
    geometry_msgs::msg::PoseStamped pose;
    serialization.deserialize_message(&serialized, &pose);
    truth.add(
      landfall::ros::toNanoseconds(pose.header.stamp),
      landfall::toPose2(landfall::ros::toPose3(pose.pose)));
  }
  return truth;
}

// Largest position (m) and yaw (deg) error against the ground truth, mapped into the map frame by
// the best-fitting SE(2) transform (the depot map's alignment to Gazebo's world is known only to
// about 2 cm) and by the nominal one (map = world + (8, 0)).
struct Errors
{
  std::size_t count = 0;
  double fitted_position = 0.0;
  double fitted_yaw_deg = 0.0;
  double nominal_position = 0.0;
};

Errors errors(
  const landfall::eval::Trajectory & truth, const std::vector<std::pair<int64_t, Pose2>> & poses)
{
  std::vector<Pose2> matched_truth;
  std::vector<Pose2> matched;
  for (const auto & [stamp, pose] : poses) {
    if (const auto t = truth.at(stamp, 250'000'000)) {
      matched_truth.push_back(*t);
      matched.push_back(pose);
    }
  }
  Errors e;
  e.count = matched.size();
  std::vector<double> position;
  std::vector<double> yaw;
  landfall::eval::poseErrors(
    matched_truth, matched, landfall::eval::alignSE2(matched_truth, matched), &position, &yaw);
  e.fitted_position = landfall::eval::errorStats(position).max;
  e.fitted_yaw_deg = landfall::eval::errorStats(yaw).max * 180.0 / M_PI;
  landfall::eval::poseErrors(matched_truth, matched, {8.0, 0.0, 0.0}, &position, &yaw);
  e.nominal_position = landfall::eval::errorStats(position).max;
  return e;
}

}  // namespace

TEST(FixtureBuild, DepotFixtureBuildsAnAccurateDeterministicMap)
{
  const auto config = landfall::parseBuildConfig(readFile(kFixture + "/build.yaml"));
  auto bag = landfall::ros::readBagKeyframes(kFixture + "/bag", config);
  EXPECT_EQ(bag.images, 90U);
  EXPECT_GE(bag.keyframes.size(), 30U);

  landfall::BuildInput input;
  input.map = landfall::loadMap(
    kFixture + "/depot.yaml", std::string(LANDFALL_ROS_DISTRO) == "humble"
                                ? landfall::MapServerSemantics::kHumble
                                : landfall::MapServerSemantics::kJazzy);
  input.sessions.push_back(bag.session);
  input.cameras.push_back(bag.camera);
  input.keyframes = bag.keyframes;

  TempDir dir;
  auto one_thread = config;
  one_thread.builder.threads = 1;
  auto four_threads = config;
  four_threads.builder.threads = 4;
  const auto started = std::chrono::steady_clock::now();
  const auto stats = landfall::buildMap(input, four_threads, dir.file("a.landfall.db"));
  const double seconds =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  landfall::buildMap(input, one_thread, dir.file("b.landfall.db"));
  RecordProperty("build_seconds_4_threads", std::to_string(seconds));
  RecordProperty("keyframes", std::to_string(stats.keyframes));
  RecordProperty("landmarks", std::to_string(stats.landmarks));
  RecordProperty("mean_track_length", std::to_string(stats.mean_track_length));
  RecordProperty("reprojection_rms_px", std::to_string(stats.reprojection_rms_px));
  RecordProperty("ba_max_shift_cm", std::to_string(100.0 * stats.ba_max_shift_m));
  RecordProperty("poses_scan_matched", std::to_string(stats.poses_scan_matched));
  RecordProperty("poses_scan_contradicted", std::to_string(stats.poses_scan_contradicted));
  RecordProperty("poses_odometry_links", std::to_string(stats.poses_odometry_links));
  RecordProperty("poses_mean_correction_cm", std::to_string(100.0 * stats.poses_mean_correction_m));

  // The fixture keeps a scan every 3 s and an image every 5 s: most keyframes have a scan within
  // bag.scan_max_time_offset. Every one matches the map, and none contradicts the recorded pose:
  // AMCL was good in this drive, and the build keeps its poses (V56).
  EXPECT_GE(bag.with_scan, bag.keyframes.size() / 2);
  EXPECT_EQ(stats.poses_scan_matched, static_cast<int>(bag.with_scan));
  EXPECT_EQ(stats.poses_scan_contradicted, 0);
  EXPECT_EQ(stats.poses_odometry_links, stats.keyframes - 1);
  EXPECT_LT(seconds, 60.0);
  EXPECT_GE(stats.landmarks, 600);
  EXPECT_GE(stats.mean_track_length, 3.0);
  EXPECT_LE(stats.reprojection_rms_px, 1.5);
  EXPECT_TRUE(stats.ba_usable);
  EXPECT_LE(stats.ba_max_shift_m, 0.03);
  EXPECT_EQ(stats.ba_keyframes_at_bound, 0);
  // A cut of the drive sees little of the depot, and the build says so; nothing else is wrong.
  RecordProperty("coverage_any_direction", std::to_string(stats.coverage_any_direction));
  RecordProperty("coverage_two_directions", std::to_string(stats.coverage_two_directions));
  EXPECT_LT(stats.coverage_any_direction, 0.8);
  std::string warnings;
  for (const std::string & w : stats.warnings) {
    warnings += w + "\n";
  }
  ASSERT_EQ(stats.warnings.size(), 1U) << warnings;
  EXPECT_NE(stats.warnings.front().find("below coverage.target"), std::string::npos);

  // Deterministic: the same content whatever the thread count.
  const auto a = landfall::db::Database::open(
    dir.file("a.landfall.db"), landfall::db::Database::Access::kReadOnly);
  const auto b = landfall::db::Database::open(
    dir.file("b.landfall.db"), landfall::db::Database::Access::kReadOnly);
  EXPECT_EQ(a.contentSha256(), b.contentSha256());
  EXPECT_EQ(a.meta("content_sha256").value_or(""), a.contentSha256());
  // The floor near the robot was hidden from the detector, and the database says so: queries
  // (test_fixture_query) hide it the same way.
  int masked = 0;
  for (const auto & backend : a.backends()) {
    masked += backend.role == "local" &&
                  backend.config.find("floor_mask_distance: 5\n") != std::string::npos
                ? 1
                : 0;
  }
  EXPECT_EQ(masked, 1);

  // Keyframe poses (recorded priors and after BA) against Gazebo's ground truth.
  const auto truth = groundTruth(kFixture + "/bag");
  std::vector<std::pair<int64_t, Pose2>> prior;
  std::vector<std::pair<int64_t, Pose2>> refined;
  for (const auto & k : a.keyframes()) {
    prior.emplace_back(k.stamp_ns, k.prior);
    refined.emplace_back(k.stamp_ns, k.refined);
  }
  const Errors before = errors(truth, prior);
  const Errors after = errors(truth, refined);
  RecordProperty("prior_max_error_cm", std::to_string(100.0 * before.fitted_position));
  RecordProperty("prior_max_yaw_error_deg", std::to_string(before.fitted_yaw_deg));
  RecordProperty("prior_nominal_max_error_cm", std::to_string(100.0 * before.nominal_position));
  RecordProperty("refined_max_error_cm", std::to_string(100.0 * after.fitted_position));
  RecordProperty("refined_max_yaw_error_deg", std::to_string(after.fitted_yaw_deg));
  RecordProperty("refined_nominal_max_error_cm", std::to_string(100.0 * after.nominal_position));
  EXPECT_GE(before.count, prior.size() - 2);
  EXPECT_LE(before.fitted_position, 0.05);
  EXPECT_LE(before.fitted_yaw_deg, 1.0);
  EXPECT_LE(before.nominal_position, 0.05);
  EXPECT_LE(after.fitted_position, 0.05);
  EXPECT_LE(after.fitted_yaw_deg, 1.0);
  // The trajectory as a whole stays where the recording put it, within the map frame's own
  // uncertainty: AMCL and the scan matching realize the map frame independently (both put it 2 to
  // 3 cm from the nominal transform here, which is known only to about 2 cm), so the build's mean
  // shift from the recorded poses is bounded instead of the nominal error.
  Vec2 shift = Vec2::Zero();
  for (std::size_t i = 0; i < prior.size(); ++i) {
    shift += Vec2(refined[i].second.x - prior[i].second.x, refined[i].second.y - prior[i].second.y);
  }
  shift /= static_cast<double>(prior.size());
  RecordProperty("mean_shift_cm", std::to_string(100.0 * shift.norm()));
  EXPECT_LE(shift.norm(), 0.02);
}
