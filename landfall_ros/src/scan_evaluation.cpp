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

#include "landfall/ros/scan_evaluation.hpp"

#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "landfall/eval/trajectory.hpp"
#include "landfall/grid/likelihood_field.hpp"
#include "landfall/ros/bag_reader.hpp"
#include "landfall/ros/conversions.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "tf2/exceptions.hpp"

namespace landfall::ros
{

namespace
{

eval::Trajectory readTruth(const std::string & bag, const std::string & topic)
{
  eval::Trajectory truth;
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  rosbag2_storage::StorageFilter filter;
  filter.topics = {topic};
  reader.set_filter(filter);
  static const rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    const rclcpp::SerializedMessage serialized(*message->serialized_data);
    geometry_msgs::msg::PoseStamped pose;
    serialization.deserialize_message(&serialized, &pose);
    truth.add(toNanoseconds(pose.header.stamp), toPose2(toPose3(pose.pose)));
  }
  return truth;
}

// "hit ratio 0.53 < 0.6" -> "hit ratio": rejections grouped by their kind.
std::string reasonKind(const std::string & reason)
{
  std::string kind;
  for (const char c : reason) {
    if (std::isdigit(static_cast<unsigned char>(c)) != 0) {
      break;
    }
    kind += c;
  }
  while (!kind.empty() && kind.back() == ' ') {
    kind.pop_back();
  }
  return kind;
}

}  // namespace

ScanEvaluation evaluateScanCheck(
  const std::string & bag, const OccupancyGrid & grid, const ScanEvaluationOptions & options)
{
  const eval::Trajectory truth = readTruth(bag, options.gt_topic);
  if (truth.size() == 0) {
    throw std::runtime_error("bag " + bag + " has no ground truth on '" + options.gt_topic + "'");
  }
  const BagSideData side = readBagSideData(bag, "", "", "", "", options.scan_topic);
  if (side.scans.empty()) {
    throw std::runtime_error("bag " + bag + " has no LaserScan on '" + options.scan_topic + "'");
  }
  const auto field = std::make_shared<const LikelihoodField>(grid);
  const ScanVerifier verifier(field, options.params);
  std::mt19937_64 rng(options.seed);
  std::uniform_real_distribution<double> distance(options.wrong_min, options.wrong_max);
  std::uniform_real_distribution<double> direction(-M_PI, M_PI);
  std::uniform_real_distribution<double> turn(-options.wrong_yaw_deg, options.wrong_yaw_deg);
  const auto max_gap = static_cast<int64_t>(options.max_gt_gap * 1e9);

  ScanEvaluation e;
  const std::size_t every = std::max<std::size_t>(options.every, 1);
  for (std::size_t i = 0; i < side.scans.size(); i += every) {
    const auto & scan = side.scans[i];
    const int64_t stamp = toNanoseconds(scan.header.stamp);
    const auto world = truth.at(stamp, max_gap);
    PlanarScan planar;
    try {
      planar.T_base_laser = toPose3(side.buffer
                                      ->lookupTransform(
                                        options.base_frame, scan.header.frame_id,
                                        tf2::TimePoint(std::chrono::nanoseconds(stamp)))
                                      .transform);
    } catch (const tf2::TransformException &) {
      ++e.skipped;
      continue;
    }
    if (!world) {
      ++e.skipped;
      continue;
    }
    planar.ranges = scan.ranges;
    planar.angle_min = scan.angle_min;
    planar.angle_increment = scan.angle_increment;
    planar.range_min = scan.range_min;
    planar.range_max = scan.range_max;
    const ProjectedScan projected = verifier.project(planar, Vec3(0.0, 0.0, -1.0));
    const Pose2 pose = compose(options.world_to_map, *world);

    ++e.true_checks;
    const ScanCheckResult result = verifier.verify(projected, pose);
    if (result.passed) {
      ++e.true_accepted;
    } else {
      ++e.true_rejections[reasonKind(result.reason)];
    }
    const ScanScore at_truth = verifier.score(projected, pose, true);
    e.true_hit_ratios.push_back(at_truth.hit_ratio);
    e.true_violation_ratios.push_back(at_truth.violation_ratio);
    e.true_known_ratios.push_back(at_truth.known_ratio);
    e.true_valid_beams.push_back(at_truth.valid_beams);
    e.true_refinements.push_back(translationDistance(result.best_pose, pose));

    for (int w = 0; w < options.wrong_per_scan; ++w) {
      Pose2 wrong;
      double offset = 0.0;
      double yaw_offset = 0.0;
      bool found = false;
      for (int attempt = 0; attempt < 100 && !found; ++attempt) {
        offset = distance(rng);
        const double angle = direction(rng);
        yaw_offset = turn(rng);
        wrong = {
          pose.x + offset * std::cos(angle), pose.y + offset * std::sin(angle),
          normalizeAngle(pose.yaw + yaw_offset * M_PI / 180.0)};
        found = field->classify(wrong.x, wrong.y) == LikelihoodField::Cell::kFree;
      }
      if (!found) {
        continue;
      }
      ++e.wrong_checks;
      const ScanCheckResult wrong_result = verifier.verify(projected, wrong);
      if (wrong_result.passed) {
        ++e.wrong_accepted;
        if (e.wrong_examples.size() < 10) {
          e.wrong_examples.push_back(
            {offset, yaw_offset, wrong_result.score.hit_ratio, wrong_result.score.violation_ratio});
        }
      }
    }
  }
  return e;
}

void setScanCheckParameter(ScanCheckParams * p, const std::string & name, const std::string & value)
{
  std::size_t used = 0;
  double v = 0.0;
  try {
    v = std::stod(value, &used);
  } catch (const std::exception &) {
    used = 0;
  }
  if (used == 0 || used != value.size()) {
    throw std::invalid_argument(name + ": '" + value + "' is not a number");
  }
  if (name == "scan_check.max_range") {
    p->max_range = v;
  } else if (name == "scan_check.hit_tolerance") {
    p->hit_tolerance = v;
  } else if (name == "scan_check.min_hit_ratio") {
    p->min_hit_ratio = v;
  } else if (name == "scan_check.max_violation_ratio") {
    p->max_violation_ratio = v;
  } else if (name == "scan_check.min_known_ratio") {
    p->min_known_ratio = v;
  } else if (name == "scan_check.min_valid_beams") {
    p->min_valid_beams = static_cast<int>(v);
  } else if (name == "scan_check.violation_margin") {
    p->violation_margin = v;
  } else if (name == "scan_check.search_radius") {
    p->search_radius = v;
  } else if (name == "scan_check.search_angle_deg") {
    p->search_angle_deg = v;
  } else if (name == "scan_check.floor_clearance") {
    p->floor_clearance = v;
  } else {
    throw std::invalid_argument("unknown scan check parameter '" + name + "'");
  }
}

}  // namespace landfall::ros
