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

// `kidnap_runner`: the kidnap benchmark in a Gazebo simulation (plan §11 M7, docs/benchmark.md).
//
// It moves the robot to random places (Gazebo's set_pose through ros_gz_bridge), leaving the
// localization with the pose it had, and measures how the robot gets its pose back:
//  - method `landfall`: requests to the relocalizer, turning 45 degrees between attempts as the
//    example behavior tree does, then a turn on the spot for AMCL to converge on the pose;
//  - method `amcl`: AMCL's global localization (reinitialize_global_localization) and turns on
//    the spot, the baseline;
//  - method `watchdog`: the robot turns slowly where it was put, and the relocalizer's kidnap
//    watchdog has to notice and relocalize; then the runner counts the watchdog's needless
//    relocalizations while the localization is right.
// Every placement is compared with Gazebo's ground truth and written as one CSV row;
// `summarize_benchmark` turns the rows into the results tables.
//
//   ros2 run landfall_benchmark kidnap_runner --ros-args --params-file SCENARIO.yaml
//     -p method:=landfall -p label:="nominal (classical)" -p output:=results.csv

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "landfall/benchmark/metrics.hpp"
#include "landfall/benchmark/placements.hpp"
#include "landfall/db/database.hpp"
#include "landfall/grid/map_loader.hpp"
#include "landfall/ros/conversions.hpp"
#include "landfall_msgs/msg/relocalization_result.hpp"
#include "landfall_msgs/srv/relocalize.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/entity.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "std_srvs/srv/empty.hpp"

namespace
{

using landfall::Pose2;
using landfall::benchmark::ErrorLimits;
using landfall::benchmark::Outcome;
using landfall::benchmark::Placement;
using Clock = std::chrono::steady_clock;
using Result = landfall_msgs::msg::RelocalizationResult;
using Relocalize = landfall_msgs::srv::Relocalize;
using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;
using PoseCov = geometry_msgs::msg::PoseWithCovarianceStamped;

double deg(double rad) { return rad * 180.0 / M_PI; }
double rad(double degrees) { return degrees * M_PI / 180.0; }

double seconds(Clock::duration d) { return std::chrono::duration<double>(d).count(); }

void sleepFor(double s)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(std::max(0.0, s)));
}

std::string fixed(double value, int decimals)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

/// Largest standard deviation in x/y of a 6x6 pose covariance (worst direction).
double worstStdXy(const std::array<double, 36> & c)
{
  const double a = c[0];
  const double b = c[1];
  const double d = c[7];
  const double mean = 0.5 * (a + d);
  const double spread = std::sqrt(std::max(0.0, 0.25 * (a - d) * (a - d) + b * b));
  return std::sqrt(std::max(0.0, mean + spread));
}

struct Stamped
{
  Pose2 pose;
  Clock::time_point received;
  double std_xy = 0.0;   // AMCL only
  int64_t stamp_ns = 0;  // the message's (simulation time)
};

/// One CSV row.
struct Row
{
  std::string label;
  std::string method;
  int index = 0;
  Placement placement;
  std::optional<Pose2> prior;  // the localization's pose before the run (where it thinks it is)
  Outcome outcome = Outcome::kInvalid;
  int attempts = 0;
  std::string status;
  std::optional<Pose2> published;
  double published_error_m = NAN;
  double published_error_deg = NAN;
  // The published covariance: its x-y ellipse's long axis and yaw (standard deviations), and the
  // position error in its standard deviations (inside its 95 % ellipse up to 2.45).
  double published_std_m = NAN;
  double published_std_deg = NAN;
  double published_error_sigma = NAN;
  // How far the robot moved between the settled teleport and the published pose's stamp (turns
  // between attempts, or sliding that odometry doesn't see).
  double truth_drift_m = NAN;
  double truth_drift_deg = NAN;
  std::optional<Stamped> amcl;
  double amcl_error_m = NAN;
  double amcl_error_deg = NAN;
  double time_to_localize_s = NAN;
  double detection_s = NAN;
  int false_relocalizations = 0;
  double observed_s = 0.0;
  double elapsed_s = 0.0;
  std::string message;
};

std::string csvHeader()
{
  return "label,method,index,x,y,yaw_deg,keyframe_distance_m,heading_difference_deg,prior_x,"
         "prior_y,prior_yaw_deg,outcome,attempts,status,published,published_x,published_y,"
         "published_yaw_deg,published_error_m,published_error_deg,published_std_m,"
         "published_std_deg,published_error_sigma,truth_drift_m,truth_drift_deg,"
         "amcl_x,amcl_y,amcl_yaw_deg,amcl_std_xy,amcl_error_m,amcl_error_deg,time_to_localize_s,"
         "detection_s,false_relocalizations,observed_s,elapsed_s,message";
}

std::string number(double v, int decimals) { return std::isnan(v) ? "" : fixed(v, decimals); }

std::string csvLine(const Row & r)
{
  std::ostringstream o;
  const Placement & p = r.placement;
  // Free text can't hold the separator.
  auto field = [](std::string text) {
    std::replace(text.begin(), text.end(), ',', ';');
    std::replace(text.begin(), text.end(), '\n', ' ');
    return text;
  };
  o << field(r.label) << "," << r.method << "," << r.index << "," << fixed(p.pose.x, 3) << ","
    << fixed(p.pose.y, 3) << "," << fixed(deg(p.pose.yaw), 2) << ","
    << fixed(p.keyframe_distance, 3) << "," << fixed(deg(p.heading_difference), 2) << ",";
  if (r.prior) {
    o << fixed(r.prior->x, 3) << "," << fixed(r.prior->y, 3) << "," << fixed(deg(r.prior->yaw), 2)
      << ",";
  } else {
    o << ",,,";
  }
  o << landfall::benchmark::toString(r.outcome) << "," << r.attempts << "," << r.status << ","
    << (r.published ? 1 : 0) << ",";
  if (r.published) {
    o << fixed(r.published->x, 3) << "," << fixed(r.published->y, 3) << ","
      << fixed(deg(r.published->yaw), 2) << ",";
  } else {
    o << ",,,";
  }
  o << number(r.published_error_m, 4) << "," << number(r.published_error_deg, 3) << ","
    << number(r.published_std_m, 4) << "," << number(r.published_std_deg, 3) << ","
    << number(r.published_error_sigma, 2) << "," << number(r.truth_drift_m, 4) << ","
    << number(r.truth_drift_deg, 3) << ",";
  if (r.amcl) {
    o << fixed(r.amcl->pose.x, 3) << "," << fixed(r.amcl->pose.y, 3) << ","
      << fixed(deg(r.amcl->pose.yaw), 2) << "," << fixed(r.amcl->std_xy, 4) << ",";
  } else {
    o << ",,,,";
  }
  o << number(r.amcl_error_m, 4) << "," << number(r.amcl_error_deg, 3) << ","
    << number(r.time_to_localize_s, 3) << "," << number(r.detection_s, 3) << ","
    << r.false_relocalizations << "," << fixed(r.observed_s, 1) << "," << fixed(r.elapsed_s, 2)
    << "," << field(r.message);
  return o.str();
}

const char * statusName(uint8_t status)
{
  switch (status) {
    case Result::SUCCESS:
      return "SUCCESS";
    case Result::AMBIGUOUS:
      return "AMBIGUOUS";
    case Result::NO_MATCH:
      return "NO_MATCH";
    case Result::REJECTED_SANITY:
      return "REJECTED_SANITY";
    case Result::REJECTED_SCAN_CHECK:
      return "REJECTED_SCAN_CHECK";
    case Result::REJECTED_CONSISTENCY:
      return "REJECTED_CONSISTENCY";
    case Result::NOT_READY:
      return "NOT_READY";
    case Result::BUSY:
      return "BUSY";
    default:
      return "UNKNOWN";
  }
}

class KidnapRunner : public rclcpp::Node
{
public:
  KidnapRunner() : rclcpp::Node("kidnap_runner")
  {
    method_ = declare_parameter<std::string>("method", "landfall");
    if (method_ != "landfall" && method_ != "amcl" && method_ != "watchdog") {
      throw std::invalid_argument("method must be landfall, amcl or watchdog");
    }
    label_ = declare_parameter<std::string>("label", "");
    output_ = declare_parameter<std::string>("output", "kidnap.csv");
    count_ = static_cast<int>(declare_parameter<int64_t>("placements", 100));
    first_ = static_cast<int>(declare_parameter<int64_t>("first_placement", 0));
    seed_ = static_cast<uint64_t>(declare_parameter<int64_t>("seed", 1));
    map_yaml_ = declare_parameter<std::string>("sampling.map_yaml", "");
    sampling_database_ = declare_parameter<std::string>("sampling.database", "");
    sampling_.max_keyframe_distance =
      declare_parameter<double>("sampling.max_keyframe_distance", 1.0);
    sampling_.clearance = declare_parameter<double>("sampling.clearance", 0.4);
    sampling_.min_separation = declare_parameter<double>("sampling.min_separation", 2.0);
    const auto world_to_map =
      declare_parameter<std::vector<double>>("world_to_map", std::vector<double>{0.0, 0.0, 0.0});
    if (world_to_map.size() != 3) {
      throw std::invalid_argument("world_to_map is [x, y, yaw_deg]");
    }
    world_to_map_ = Pose2{world_to_map[0], world_to_map[1], rad(world_to_map[2])};
    entity_ = declare_parameter<std::string>("entity", "nav2_turtlebot4");
    spawn_z_ = declare_parameter<double>("spawn_z", 0.01);
    settle_s_ = declare_parameter<double>("settle_time", 1.5);
    // Longer than the relocalizer's startup attempts in the simulation (relocalizer_depot_sim.yaml:
    // 5 requests of up to 10 s, 5 s apart): no request of its own may still run at placement 0.
    startup_wait_s_ = declare_parameter<double>("startup_wait", 90.0);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.5);
    attempts_ = static_cast<int>(declare_parameter<int64_t>("landfall.attempts", 8));
    turn_deg_ = declare_parameter<double>("landfall.turn_deg", 45.0);
    num_frames_ = static_cast<int>(declare_parameter<int64_t>("landfall.num_frames", 0));
    request_timeout_ = declare_parameter<double>("landfall.request_timeout", 2.5);
    convergence_deg_ = declare_parameter<double>("convergence.turn_deg", 360.0);
    amcl_turn_deg_ = declare_parameter<double>("amcl.turn_deg", 720.0);
    amcl_max_std_xy_ = declare_parameter<double>("amcl.max_std_xy", 0.5);
    watchdog_wait_s_ = declare_parameter<double>("watchdog.wait", 90.0);
    watchdog_speed_ = declare_parameter<double>("watchdog.angular_speed", 0.1);
    watchdog_observe_s_ = declare_parameter<double>("watchdog.observe", 30.0);
    const auto home =
      declare_parameter<std::vector<double>>("watchdog.home", std::vector<double>{0.0, 0.0, 0.0});
    if (home.size() != 3) {
      throw std::invalid_argument("watchdog.home is [x, y, yaw_deg]");
    }
    home_ = Pose2{home[0], home[1], rad(home[2])};
    watchdog_reset_s_ = declare_parameter<double>("watchdog.reset_wait", 25.0);
    limits_.success_translation = declare_parameter<double>("limits.success_translation", 0.25);
    limits_.success_rotation_deg = declare_parameter<double>("limits.success_rotation_deg", 5.0);
    limits_.wrong_translation = declare_parameter<double>("limits.wrong_translation", 0.5);
    limits_.wrong_rotation_deg = declare_parameter<double>("limits.wrong_rotation_deg", 10.0);

    const std::string set_pose =
      declare_parameter<std::string>("set_pose_service", "/world/depot/set_pose");
    const std::string truth =
      declare_parameter<std::string>("ground_truth_topic", "/ground_truth/pose");
    const std::string amcl = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
    const std::string reinitialize =
      declare_parameter<std::string>("reinitialize_service", "/reinitialize_global_localization");
    const std::string relocalize =
      declare_parameter<std::string>("relocalize_service", "/landfall_relocalizer/relocalize");
    const std::string status =
      declare_parameter<std::string>("status_topic", "/landfall_relocalizer/status");
    const std::string cmd_vel = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const std::string initial_pose =
      declare_parameter<std::string>("initial_pose_topic", "/initialpose");

    set_pose_client_ = create_client<SetEntityPose>(set_pose);
    reinitialize_client_ = create_client<std_srvs::srv::Empty>(reinitialize);
    relocalize_client_ = create_client<Relocalize>(relocalize);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel, rclcpp::QoS(10));
    initial_pose_pub_ = create_publisher<PoseCov>(initial_pose, rclcpp::QoS(1).reliable());
    truth_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      truth, rclcpp::SensorDataQoS(), [this](const geometry_msgs::msg::PoseStamped & m) {
        const Pose2 pose =
          landfall::compose(world_to_map_, landfall::toPose2(landfall::ros::toPose3(m.pose)));
        const int64_t stamp = landfall::ros::toNanoseconds(m.header.stamp);
        const std::lock_guard<std::mutex> lock(mutex_);
        truth_ = Stamped{pose, Clock::now(), 0.0, stamp};
        // The last two minutes, to compare estimates with the truth at their own stamps.
        truth_history_.emplace_back(stamp, pose);
        while (truth_history_.front().first < stamp - 120'000'000'000) {
          truth_history_.pop_front();
        }
      });
    // AMCL publishes amcl_pose latched, on filter updates.
    amcl_sub_ = create_subscription<PoseCov>(
      amcl, rclcpp::QoS(1).reliable().transient_local(), [this](const PoseCov & m) {
        const std::lock_guard<std::mutex> lock(mutex_);
        amcl_ = Stamped{
          landfall::toPose2(landfall::ros::toPose3(m.pose.pose)), Clock::now(),
          worstStdXy(m.pose.covariance), landfall::ros::toNanoseconds(m.header.stamp)};
      });
    status_sub_ = create_subscription<Result>(
      status, rclcpp::QoS(10).reliable().transient_local(), [this](const Result & r) {
        const std::lock_guard<std::mutex> lock(mutex_);
        statuses_.emplace_back(Clock::now(), r);
      });
  }

  /// Runs every placement; returns the process exit code.
  int run(const std::atomic<bool> & stop)
  {
    const auto placements = samplePlacementsFromFiles();
    RCLCPP_INFO(
      get_logger(), "%s (%s): placements %d to %d of seed %lu", label_.c_str(), method_.c_str(),
      first_, count_ - 1, static_cast<unsigned long>(seed_));
    if (!waitForSystem(stop)) {
      return 1;
    }
    const bool fresh = !std::ifstream(output_).good();
    std::ofstream out(output_, std::ios::app);
    if (!out) {
      RCLCPP_ERROR(get_logger(), "cannot write %s", output_.c_str());
      return 1;
    }
    if (fresh) {
      out << csvHeader() << "\n";
    }
    for (int i = first_; i < count_ && !stop && rclcpp::ok(); ++i) {
      Row row;
      row.label = label_;
      row.method = method_;
      row.index = i;
      row.placement = placements[static_cast<std::size_t>(i)];
      const auto started = Clock::now();
      try {
        runPlacement(row, stop);
      } catch (const std::exception & e) {
        row.outcome = Outcome::kInvalid;
        row.message = e.what();
      }
      stopRobot();
      row.elapsed_s = seconds(Clock::now() - started);
      out << csvLine(row) << "\n";
      out.flush();
      RCLCPP_INFO(
        get_logger(), "placement %d (%.2f, %.2f, %.0f deg): %s after %d attempt(s) in %.1f s%s%s",
        i, row.placement.pose.x, row.placement.pose.y, deg(row.placement.pose.yaw),
        landfall::benchmark::toString(row.outcome), row.attempts, row.elapsed_s,
        row.message.empty() ? "" : ": ", row.message.c_str());
    }
    return 0;
  }

private:
  std::vector<Placement> samplePlacementsFromFiles() const
  {
    const auto map = landfall::loadMap(map_yaml_, landfall::MapServerSemantics::kJazzy);
    const auto db =
      landfall::db::Database::open(sampling_database_, landfall::db::Database::Access::kReadOnly);
    std::vector<Pose2> keyframes;
    for (const auto & k : db.keyframes()) {
      keyframes.push_back(k.refined);
    }
    return landfall::benchmark::samplePlacements(map.grid, keyframes, sampling_, seed_, count_);
  }

  /// Waits for the services, the ground truth and AMCL's first pose. That pose comes from the
  /// relocalizer's startup relocalization; where it finds none in `startup_wait` seconds (in the
  /// added_objects world, unmapped goods near the spawn pose make the LiDAR check refuse it), AMCL
  /// gets the true pose: the benchmark measures the placements, not the start.
  bool waitForSystem(const std::atomic<bool> & stop)
  {
    const auto started = Clock::now();
    const auto deadline = started + std::chrono::seconds(180);
    const auto fallback = started + std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double>(startup_wait_s_));
    bool gave_truth = false;
    auto ready = [&] {
      const std::lock_guard<std::mutex> lock(mutex_);
      return truth_.has_value() && amcl_.has_value();
    };
    while (!stop && rclcpp::ok() && Clock::now() < deadline) {
      const bool services = set_pose_client_->service_is_ready() &&
                            (method_ == "amcl" ? reinitialize_client_->service_is_ready()
                                               : relocalize_client_->service_is_ready());
      if (services && ready()) {
        return true;
      }
      if (services && !gave_truth && Clock::now() >= fallback && giveAmclTheTruth()) {
        RCLCPP_WARN(
          get_logger(),
          "AMCL has no pose %.0f s after the start (the startup relocalization found none): "
          "giving it the true pose",
          startup_wait_s_);
        gave_truth = true;
      }
      sleepFor(0.5);
    }
    RCLCPP_ERROR(
      get_logger(), "the simulation is not ready (set_pose, %s, ground truth, amcl_pose)",
      method_ == "amcl" ? "reinitialize_global_localization" : "relocalize");
    return false;
  }

  void runPlacement(Row & row, const std::atomic<bool> & stop)
  {
    if (method_ == "watchdog" && needs_reset_) {
      resetLocalization();
    }
    if (!teleport(row.placement.pose)) {
      row.outcome = Outcome::kInvalid;
      row.message = "teleport failed";
      return;
    }
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (amcl_) {
        row.prior = amcl_->pose;
      }
    }
    if (method_ == "landfall") {
      runLandfall(row, stop);
    } else if (method_ == "amcl") {
      runAmcl(row, stop);
    } else {
      runWatchdog(row, stop);
      // A kidnap the watchdog didn't undo leaves the localization wrong, and maybe a disagreeing
      // check counted: the next placement starts from a right localization again.
      needs_reset_ = row.outcome != Outcome::kSuccess;
    }
  }

  /// Back to the home pose, where vision localizes, with AMCL given the true pose; then enough
  /// time for the watchdog's checks to agree with it, which clears their count.
  void resetLocalization()
  {
    if (!teleport(home_)) {
      return;
    }
    if (!giveAmclTheTruth()) {
      return;
    }
    sleepFor(watchdog_reset_s_);
    needs_reset_ = false;
  }

  /// Publishes the ground truth as AMCL's initial pose (0.1 m, 3 degrees); false without one.
  bool giveAmclTheTruth()
  {
    const auto truth = latestTruth();
    if (!truth) {
      return false;
    }
    PoseCov pose;
    pose.header.frame_id = "map";
    pose.header.stamp = landfall::ros::toStamp(truth->stamp_ns);
    pose.pose.pose = landfall::ros::toPoseMsg(landfall::toPose3(truth->pose));
    pose.pose.covariance[0] = 0.01;
    pose.pose.covariance[7] = 0.01;
    pose.pose.covariance[35] = rad(3.0) * rad(3.0);
    initial_pose_pub_->publish(pose);
    return true;
  }

  void runLandfall(Row & row, const std::atomic<bool> & stop)
  {
    const auto started = Clock::now();
    for (int a = 1; a <= attempts_ && !stop; ++a) {
      row.attempts = a;
      const auto result = relocalize();
      if (!result) {
        row.status = "NO_ANSWER";
        row.message = "the relocalizer did not answer";
      } else {
        row.status = statusName(result->status);
        row.message = result->message;
        if (result->published) {
          row.time_to_localize_s = seconds(Clock::now() - started);
          recordPublished(row, *result);
          break;
        }
      }
      if (a < attempts_) {
        turn(rad(turn_deg_), angular_speed_);
      }
    }
    if (row.published) {
      converge(row, Clock::now());
    }
    classify(row);
  }

  void runAmcl(Row & row, const std::atomic<bool> & stop)
  {
    const auto started = Clock::now();
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = reinitialize_client_->async_send_request(request);
    if (future.future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      reinitialize_client_->remove_pending_request(future.request_id);
      row.outcome = Outcome::kInvalid;
      row.message = "reinitialize_global_localization did not answer";
      return;
    }
    row.attempts = 1;
    // Turn while AMCL updates; note when it first claims a pose.
    std::optional<double> claimed_at;
    const auto check = [&] {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (!claimed_at && amcl_ && amcl_->received > started && amcl_->std_xy <= amcl_max_std_xy_) {
        claimed_at = seconds(amcl_->received - started);
      }
    };
    turn(rad(amcl_turn_deg_), angular_speed_, check);
    if (stop) {
      return;
    }
    sleepFor(1.0);
    check();
    const auto amcl = latestAmclAfter(started);
    if (!amcl) {
      row.outcome = Outcome::kMiss;
      row.status = "NO_ESTIMATE";
      return;
    }
    row.amcl = amcl;
    measureAmcl(row);
    const bool claims = amcl->std_xy <= amcl_max_std_xy_;
    row.status = claims ? "CLAIMS_POSE" : "UNCERTAIN";
    if (claims && claimed_at) {
      row.time_to_localize_s = *claimed_at;
    }
    if (!claims) {
      row.outcome = Outcome::kMiss;
    } else if (isWrong(row.amcl_error_m, row.amcl_error_deg)) {
      row.outcome = Outcome::kWrong;
    } else if (isRight(row.amcl_error_m, row.amcl_error_deg)) {
      row.outcome = Outcome::kSuccess;
    } else {
      row.outcome = Outcome::kInaccurate;
    }
  }

  void runWatchdog(Row & row, const std::atomic<bool> & stop)
  {
    // The localization still has the old pose. The robot turns slowly where it is (views change,
    // as when it drives on); the watchdog has `watchdog.wait` seconds to relocalize it, counted
    // from the kidnap.
    const auto started = kidnapped_at_;
    const auto deadline = started + std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double>(watchdog_wait_s_));
    std::optional<Result> fired;
    while (!stop && rclcpp::ok() && Clock::now() < deadline && !fired) {
      publishTurn(watchdog_speed_);
      sleepFor(0.05);
      fired = watchdogResultAfter(started);
    }
    stopRobot();
    row.attempts = 1;
    if (!fired) {
      row.status = "NOT_DETECTED";
      classify(row);
      return;
    }
    row.detection_s = seconds(Clock::now() - started);  // from the kidnap
    row.status = statusName(fired->status);
    row.message = fired->message;
    recordPublished(row, *fired);
    const auto converged = Clock::now();
    converge(row, converged);
    classify(row);
    // Now the localization is right: any watchdog relocalization from here on is needless.
    const auto observing = Clock::now();
    const auto until = observing + std::chrono::duration_cast<Clock::duration>(
                                     std::chrono::duration<double>(watchdog_observe_s_));
    while (!stop && rclcpp::ok() && Clock::now() < until) {
      publishTurn(watchdog_speed_);
      sleepFor(0.05);
    }
    stopRobot();
    row.observed_s = seconds(Clock::now() - observing);
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & [received, r] : statuses_) {
      if (received > converged && r.trigger == "watchdog" && r.published) {
        ++row.false_relocalizations;
      }
    }
  }

  std::optional<Result> watchdogResultAfter(Clock::time_point t)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & [received, r] : statuses_) {
      if (received > t && r.trigger == "watchdog" && r.published) {
        return r;
      }
    }
    return std::nullopt;
  }

  /// The published pose against the ground truth at its stamp.
  void recordPublished(Row & row, const Result & result)
  {
    row.published = landfall::toPose2(landfall::ros::toPose3(result.pose.pose.pose));
    const landfall::Mat3 covariance = landfall::ros::planarCovariance(result.pose.pose.covariance);
    const Eigen::Matrix2d xy = covariance.topLeftCorner<2, 2>();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> axes(xy);
    const bool definite = axes.eigenvalues()(0) > 0.0;
    if (definite) {
      row.published_std_m = std::sqrt(axes.eigenvalues()(1));
    }
    row.published_std_deg = deg(std::sqrt(std::max(0.0, covariance(2, 2))));
    if (const auto truth = truthAt(landfall::ros::toNanoseconds(result.pose.header.stamp))) {
      row.published_error_m = landfall::translationDistance(*row.published, *truth);
      row.published_error_deg = deg(landfall::rotationDistance(*row.published, *truth));
      if (definite) {
        const Eigen::Vector2d e(row.published->x - truth->x, row.published->y - truth->y);
        row.published_error_sigma = std::sqrt(e.dot(xy.ldlt().solve(e)));
      }
      if (settled_) {
        row.truth_drift_m = landfall::translationDistance(*settled_, *truth);
        row.truth_drift_deg = deg(landfall::rotationDistance(*settled_, *truth));
      }
    } else {
      row.message += " (no ground truth at the published pose's stamp)";
    }
  }

  /// After a published pose: AMCL takes it, then the robot turns on the spot so that AMCL's
  /// filter converges; AMCL's estimate is then compared with the ground truth.
  void converge(Row & row, Clock::time_point published_at)
  {
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    while (Clock::now() < deadline && !latestAmclAfter(published_at)) {
      sleepFor(0.1);
    }
    const auto turning = Clock::now();
    turn(rad(convergence_deg_), angular_speed_);
    sleepFor(1.0);
    row.amcl = latestAmclAfter(turning);
    measureAmcl(row);
  }

  /// AMCL's estimate against the ground truth at its stamp: AMCL publishes only on filter updates,
  /// so its last estimate can be older than the robot's last motion.
  void measureAmcl(Row & row)
  {
    if (!row.amcl) {
      return;
    }
    if (const auto truth = truthAt(row.amcl->stamp_ns)) {
      row.amcl_error_m = landfall::translationDistance(row.amcl->pose, *truth);
      row.amcl_error_deg = deg(landfall::rotationDistance(row.amcl->pose, *truth));
    } else {
      row.message += " (no ground truth at AMCL's stamp)";
    }
  }

  void classify(Row & row) const
  {
    if (!row.published) {
      row.outcome = Outcome::kMiss;
    } else if (
      !std::isnan(row.published_error_m) &&
      isWrong(row.published_error_m, row.published_error_deg)) {
      row.outcome = Outcome::kWrong;
    } else if (!std::isnan(row.amcl_error_m) && isRight(row.amcl_error_m, row.amcl_error_deg)) {
      row.outcome = Outcome::kSuccess;
    } else {
      row.outcome = Outcome::kInaccurate;
    }
  }

  bool isWrong(double m, double degrees) const
  {
    return landfall::benchmark::isWrong(m, degrees, limits_);
  }
  bool isRight(double m, double degrees) const
  {
    return landfall::benchmark::isRight(m, degrees, limits_);
  }

  std::optional<Stamped> latestTruth()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return truth_;
  }

  /// The ground truth nearest to a stamp, if one is within 0.1 s of it.
  std::optional<Pose2> truthAt(int64_t stamp_ns)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto after = std::lower_bound(
      truth_history_.begin(), truth_history_.end(), stamp_ns,
      [](const std::pair<int64_t, Pose2> & a, int64_t t) { return a.first < t; });
    const std::pair<int64_t, Pose2> * best = nullptr;
    if (after != truth_history_.end()) {
      best = &*after;
    }
    if (after != truth_history_.begin()) {
      const auto & before = *std::prev(after);
      if (!best || stamp_ns - before.first < best->first - stamp_ns) {
        best = &before;
      }
    }
    if (!best || std::llabs(best->first - stamp_ns) > 100'000'000) {
      return std::nullopt;
    }
    return best->second;
  }

  std::optional<Stamped> latestAmclAfter(Clock::time_point t)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (amcl_ && amcl_->received > t) {
      return amcl_;
    }
    return std::nullopt;
  }

  bool teleport(const Pose2 & target)
  {
    settled_.reset();
    stopRobot();
    const Pose2 world = landfall::compose(landfall::inverse(world_to_map_), target);
    for (int attempt = 0; attempt < 3; ++attempt) {
      auto request = std::make_shared<SetEntityPose::Request>();
      request->entity.name = entity_;
      request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
      request->pose.position.x = world.x;
      request->pose.position.y = world.y;
      request->pose.position.z = spawn_z_;
      request->pose.orientation.z = std::sin(0.5 * world.yaw);
      request->pose.orientation.w = std::cos(0.5 * world.yaw);
      const auto sent = Clock::now();
      auto future = set_pose_client_->async_send_request(request);
      if (future.future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        set_pose_client_->remove_pending_request(future.request_id);
        continue;
      }
      if (!future.future.get()->success) {
        continue;
      }
      kidnapped_at_ = sent;
      sleepFor(settle_s_);
      const auto truth = latestTruth();
      if (
        truth && seconds(Clock::now() - truth->received) < 1.0 &&
        landfall::translationDistance(truth->pose, target) < 0.1 &&
        deg(landfall::rotationDistance(truth->pose, target)) < 5.0) {
        settled_ = truth->pose;
        return true;
      }
    }
    return false;
  }

  std::optional<Result> relocalize()
  {
    auto request = std::make_shared<Relocalize::Request>();
    request->num_frames = static_cast<uint8_t>(num_frames_);
    request->timeout = static_cast<float>(request_timeout_);
    request->dry_run = false;
    auto future = relocalize_client_->async_send_request(request);
    const auto wait = std::chrono::duration<double>(std::max(request_timeout_, 10.0) + 20.0);
    if (future.future.wait_for(wait) != std::future_status::ready) {
      relocalize_client_->remove_pending_request(future.request_id);
      return std::nullopt;
    }
    return future.future.get()->result;
  }

  void publishTurn(double speed)
  {
    geometry_msgs::msg::Twist twist;
    twist.angular.z = speed;
    cmd_pub_->publish(twist);
  }

  void stopRobot()
  {
    for (int i = 0; i < 3; ++i) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      sleepFor(0.02);
    }
  }

  /// Turns on the spot by `angle` (rad, counter-clockwise when positive), measured on the ground
  /// truth; `each` runs at every step.
  void turn(double angle, double speed, const std::function<void()> & each = {})
  {
    auto truth = latestTruth();
    if (!truth || angle == 0.0) {
      return;
    }
    double turned = 0.0;
    double last = truth->pose.yaw;
    const double direction = angle > 0.0 ? 1.0 : -1.0;
    const auto deadline =
      Clock::now() + std::chrono::duration_cast<Clock::duration>(
                       std::chrono::duration<double>(std::abs(angle) / speed * 2.0 + 5.0));
    while (std::abs(turned) < std::abs(angle) - rad(1.0) && Clock::now() < deadline &&
           rclcpp::ok()) {
      // Slow down for the last few degrees.
      const double left = std::abs(angle) - std::abs(turned);
      publishTurn(direction * std::clamp(2.0 * left, 0.1, speed));
      sleepFor(0.05);
      truth = latestTruth();
      if (truth) {
        turned += std::atan2(std::sin(truth->pose.yaw - last), std::cos(truth->pose.yaw - last));
        last = truth->pose.yaw;
      }
      if (each) {
        each();
      }
    }
    stopRobot();
    sleepFor(0.3);
  }

  std::string method_;
  std::string label_;
  std::string output_;
  int count_ = 100;
  int first_ = 0;
  uint64_t seed_ = 1;
  std::string map_yaml_;
  std::string sampling_database_;
  landfall::benchmark::PlacementParams sampling_;
  Pose2 world_to_map_;
  std::string entity_;
  double spawn_z_ = 0.01;
  double settle_s_ = 1.5;
  double startup_wait_s_ = 90.0;
  double angular_speed_ = 0.5;
  int attempts_ = 8;
  double turn_deg_ = 45.0;
  int num_frames_ = 0;
  double request_timeout_ = 2.5;
  double convergence_deg_ = 360.0;
  double amcl_turn_deg_ = 720.0;
  double amcl_max_std_xy_ = 0.5;
  double watchdog_wait_s_ = 90.0;
  double watchdog_speed_ = 0.1;
  double watchdog_observe_s_ = 30.0;
  Pose2 home_;
  double watchdog_reset_s_ = 25.0;
  ErrorLimits limits_;

  std::mutex mutex_;
  std::optional<Stamped> truth_;
  std::deque<std::pair<int64_t, Pose2>> truth_history_;  // stamp -> pose, the last two minutes
  std::optional<Stamped> amcl_;
  std::vector<std::pair<Clock::time_point, Result>> statuses_;
  std::optional<Pose2> settled_;    // the ground truth once the teleport settled
  Clock::time_point kidnapped_at_;  // when the last teleport was asked for
  bool needs_reset_ = false;        // watchdog: the localization may be wrong

  rclcpp::Client<SetEntityPose>::SharedPtr set_pose_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reinitialize_client_;
  rclcpp::Client<Relocalize>::SharedPtr relocalize_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<PoseCov>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr truth_sub_;
  rclcpp::Subscription<PoseCov>::SharedPtr amcl_sub_;
  rclcpp::Subscription<Result>::SharedPtr status_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 1;
  try {
    auto node = std::make_shared<KidnapRunner>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::atomic<bool> stop{false};
    std::thread spinner([&] {
      while (rclcpp::ok() && !stop) {
        executor.spin_some(std::chrono::milliseconds(20));
      }
    });
    code = node->run(stop);
    stop = true;
    spinner.join();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "kidnap_runner: %s\n", e.what());
  }
  rclcpp::shutdown();
  return code;
}
