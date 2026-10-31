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

// `evaluate_poses`: accuracy of recorded poses against a simulator's ground truth in a bag.
//
//   evaluate_poses --bag BAG [--gt-topic /ground_truth/pose] [--world-to-map X Y YAW_DEG]
//                  [--max-gap SECONDS] [--staging DIR]... [--db DB]... [--amcl-topic /amcl_pose]
//
// For each source (staging keyframes, database keyframes before and after BA, AMCL estimates) it
// prints position and yaw errors against the ground truth mapped into the map frame, once with the
// nominal world->map transform and once with the best-fitting one (the map's alignment to the
// simulator world is only known up to a few centimetres).

#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/db/database.hpp"
#include "landfall/eval/trajectory.hpp"
#include "landfall/record/keyframe_selector.hpp"
#include "landfall/record/staging.hpp"
#include "landfall/ros/conversions.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"

namespace
{

using landfall::Pose2;
using landfall::eval::ErrorStats;

struct Estimate
{
  int64_t stamp_ns = 0;
  Pose2 pose;
  double reported_std_xy = 0.0;  // AMCL only
  double reported_std_yaw = 0.0;
};

template <typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  static const rclcpp::Serialization<MessageT> serialization;
  const rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  MessageT message;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

double deg(double rad) { return rad * 180.0 / M_PI; }

void printStats(const char * what, const ErrorStats & s, double scale, const char * unit)
{
  std::printf(
    "    %-8s median %7.3f  p95 %7.3f  max %7.3f  rms %7.3f %s\n", what, s.median * scale,
    s.p95 * scale, s.max * scale, s.rms * scale, unit);
}

void evaluate(
  const std::string & name, const std::vector<Estimate> & estimates,
  const landfall::eval::Trajectory & truth, int64_t max_gap_ns, const Pose2 & nominal,
  bool with_reported)
{
  std::vector<Pose2> matched_truth;
  std::vector<Pose2> matched;
  std::vector<double> reported_xy;
  std::vector<double> reported_yaw;
  std::size_t skipped = 0;
  for (const auto & e : estimates) {
    const auto t = truth.at(e.stamp_ns, max_gap_ns);
    if (!t) {
      ++skipped;
      continue;
    }
    matched_truth.push_back(*t);
    matched.push_back(e.pose);
    reported_xy.push_back(e.reported_std_xy);
    reported_yaw.push_back(e.reported_std_yaw);
  }
  std::printf("%s: %zu poses (%zu without ground truth)\n", name.c_str(), matched.size(), skipped);
  if (matched.size() < 2) {
    return;
  }
  std::vector<double> position;
  std::vector<double> yaw;
  landfall::eval::poseErrors(matched_truth, matched, nominal, &position, &yaw);
  std::printf(
    "  nominal world->map (%.3f, %.3f, %.3f deg)\n", nominal.x, nominal.y, deg(nominal.yaw));
  printStats("position", landfall::eval::errorStats(position), 100.0, "cm");
  printStats("yaw", landfall::eval::errorStats(yaw), 180.0 / M_PI, "deg");
  const Pose2 fitted = landfall::eval::alignSE2(matched_truth, matched);
  std::vector<double> fitted_position;
  std::vector<double> fitted_yaw;
  landfall::eval::poseErrors(matched_truth, matched, fitted, &fitted_position, &fitted_yaw);
  std::printf("  fitted world->map  (%.3f, %.3f, %.3f deg)\n", fitted.x, fitted.y, deg(fitted.yaw));
  printStats("position", landfall::eval::errorStats(fitted_position), 100.0, "cm");
  printStats("yaw", landfall::eval::errorStats(fitted_yaw), 180.0 / M_PI, "deg");
  if (with_reported) {
    printStats("std_xy", landfall::eval::errorStats(reported_xy), 100.0, "cm (reported)");
    printStats("std_yaw", landfall::eval::errorStats(reported_yaw), 180.0 / M_PI, "deg (reported)");
    std::size_t beyond = 0;
    for (std::size_t i = 0; i < fitted_position.size(); ++i) {
      beyond += fitted_position[i] > 2.0 * reported_xy[i] ? 1 : 0;
    }
    std::printf(
      "    error beyond 2x the reported std in %zu of %zu estimates\n", beyond,
      fitted_position.size());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string bag;
  std::string gt_topic = "/ground_truth/pose";
  std::string amcl_topic;
  std::vector<std::string> staging;
  std::vector<std::string> databases;
  Pose2 nominal{8.0, 0.0, 0.0};
  // Longest ground-truth gap to interpolate across (fixtures thin it out to 10 Hz).
  int64_t max_gap_ns = 250'000'000;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      const auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::invalid_argument(a + " needs a value");
        }
        return argv[++i];
      };
      if (a == "--bag") {
        bag = next();
      } else if (a == "--gt-topic") {
        gt_topic = next();
      } else if (a == "--amcl-topic") {
        amcl_topic = next();
      } else if (a == "--staging") {
        staging.push_back(next());
      } else if (a == "--db") {
        databases.push_back(next());
      } else if (a == "--max-gap") {
        max_gap_ns = static_cast<int64_t>(std::stod(next()) * 1e9);
      } else if (a == "--world-to-map") {
        nominal.x = std::stod(next());
        nominal.y = std::stod(next());
        nominal.yaw = std::stod(next()) * M_PI / 180.0;
      } else {
        throw std::invalid_argument("unknown argument " + a);
      }
    }
    if (bag.empty()) {
      throw std::invalid_argument("--bag is required");
    }
  } catch (const std::exception & e) {
    std::cerr << "evaluate_poses: " << e.what()
              << "\nusage: evaluate_poses --bag BAG [--gt-topic T] [--world-to-map X Y YAW_DEG] "
                 "[--max-gap S] [--staging DIR]... [--db DB]... [--amcl-topic T]\n";
    return 2;
  }

  try {
    landfall::eval::Trajectory truth;
    std::vector<Estimate> amcl;
    rosbag2_cpp::Reader reader;
    reader.open(bag);
    rosbag2_storage::StorageFilter filter;
    filter.topics = {gt_topic};
    if (!amcl_topic.empty()) {
      filter.topics.push_back(amcl_topic);
    }
    reader.set_filter(filter);
    while (reader.has_next()) {
      const auto message = reader.read_next();
      if (message->topic_name == gt_topic) {
        const auto pose = deserialize<geometry_msgs::msg::PoseStamped>(*message);
        truth.add(
          landfall::ros::toNanoseconds(pose.header.stamp),
          landfall::toPose2(landfall::ros::toPose3(pose.pose)));
      } else {
        const auto pose = deserialize<geometry_msgs::msg::PoseWithCovarianceStamped>(*message);
        const landfall::Mat3 cov = landfall::ros::planarCovariance(pose.pose.covariance);
        amcl.push_back(
          {landfall::ros::toNanoseconds(pose.header.stamp),
           landfall::toPose2(landfall::ros::toPose3(pose.pose.pose)), landfall::worstStdXy(cov),
           std::sqrt(std::max(0.0, cov(2, 2)))});
      }
    }
    std::printf("ground truth: %zu poses on %s\n", truth.size(), gt_topic.c_str());
    for (const auto & dir : staging) {
      const auto folder = landfall::readStaging(dir);
      std::vector<Estimate> estimates;
      estimates.reserve(folder.keyframes.size());
      for (const auto & k : folder.keyframes) {
        estimates.push_back({k.stamp_ns, k.map_base});
      }
      evaluate("staging " + dir, estimates, truth, max_gap_ns, nominal, false);
    }
    for (const auto & path : databases) {
      const auto db = landfall::db::Database::open(path, landfall::db::Database::Access::kReadOnly);
      std::vector<Estimate> prior;
      std::vector<Estimate> refined;
      for (const auto & k : db.keyframes()) {
        prior.push_back({k.stamp_ns, k.prior});
        refined.push_back({k.stamp_ns, k.refined});
      }
      evaluate("database " + path + " (recorded priors)", prior, truth, max_gap_ns, nominal, false);
      evaluate("database " + path + " (after BA)", refined, truth, max_gap_ns, nominal, false);
    }
    if (!amcl_topic.empty()) {
      evaluate("AMCL " + amcl_topic, amcl, truth, max_gap_ns, nominal, true);
    }
  } catch (const std::exception & e) {
    std::cerr << "evaluate_poses: error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
