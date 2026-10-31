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

  EXPECT_LT(seconds, 60.0);
  EXPECT_GE(stats.landmarks, 600);
  EXPECT_GE(stats.mean_track_length, 3.0);
  EXPECT_LE(stats.reprojection_rms_px, 1.5);
  EXPECT_TRUE(stats.ba_usable);
  EXPECT_LE(stats.ba_max_shift_m, 0.03);
  EXPECT_EQ(stats.ba_keyframes_at_bound, 0);
  EXPECT_TRUE(stats.warnings.empty()) << stats.warnings.front();

  // Deterministic: the same content whatever the thread count.
  const auto a = landfall::db::Database::open(
    dir.file("a.landfall.db"), landfall::db::Database::Access::kReadOnly);
  const auto b = landfall::db::Database::open(
    dir.file("b.landfall.db"), landfall::db::Database::Access::kReadOnly);
  EXPECT_EQ(a.contentSha256(), b.contentSha256());
  EXPECT_EQ(a.meta("content_sha256").value_or(""), a.contentSha256());

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
  RecordProperty("refined_max_error_cm", std::to_string(100.0 * after.fitted_position));
  RecordProperty("refined_max_yaw_error_deg", std::to_string(after.fitted_yaw_deg));
  EXPECT_GE(before.count, prior.size() - 2);
  EXPECT_LE(before.fitted_position, 0.05);
  EXPECT_LE(before.fitted_yaw_deg, 1.0);
  EXPECT_LE(before.nominal_position, 0.05);
  EXPECT_LE(after.fitted_position, 0.05);
  EXPECT_LE(after.fitted_yaw_deg, 1.0);
  EXPECT_LE(after.nominal_position, 0.05);
}
