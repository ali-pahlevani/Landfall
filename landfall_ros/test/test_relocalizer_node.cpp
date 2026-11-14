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

// The relocalizer node in-process, fed with the depot fixture: readiness (NOT_READY reasons), BUSY,
// publishing to AMCL's topic, dry runs, the scan check modes, odometry consistency, the startup and
// automatic triggers, the extrinsics monitor, the kidnap watchdog, lifecycle transitions and
// parameter validation.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "fixture_data.hpp"
#include "landfall/ros/relocalizer_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using landfall::ros::RelocalizerNode;
using lifecycle_msgs::msg::State;
using Result = landfall_msgs::msg::RelocalizationResult;
using Relocalize = landfall_msgs::srv::Relocalize;
using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
using Clock = std::chrono::steady_clock;

class RelocalizerNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    dir_ = new landfall::test::TempDir();
    db_path_ = new std::string(dir_->file("depot.landfall.db"));
    landfall::test::buildFixtureDatabase(*db_path_);
    data_ = new landfall::test::FixtureMessages(landfall::test::readFixtureMessages());
    map_ = new nav_msgs::msg::OccupancyGrid(landfall::test::fixtureMapMessage());
  }

  static void TearDownTestSuite()
  {
    delete map_;
    delete data_;
    delete db_path_;
    delete dir_;
    rclcpp::shutdown();
  }

  static rclcpp::NodeOptions options(const std::string & database)
  {
    rclcpp::NodeOptions o;
    o.arguments({"--ros-args", "-r", "map:=/reloc_test/map"});
    o.parameter_overrides({
      {"database_path", database},
      {"camera_info.topic", "/reloc_test/camera_info"},
      {"image.topic", "/reloc_test/image"},
      {"image.transport", "compressed"},
      {"imu.topic", "/reloc_test/imu"},
      {"output.topic", "/reloc_test/initialpose"},
      {"output.amcl_node", ""},
      {"bond.enabled", false},
      {"backend.num_threads", 2},
      {"consistency.num_frames", 2},
      {"query.timeout", 15.0},
      {"scan_check.topic", "/reloc_test/scan"},
      {"auto_trigger.pose_topic", "/reloc_test/amcl_pose"},
      {"extrinsics_monitor.enabled", false},
    });
    return o;
  }

  // The extrinsics monitor samples once a second while the localization is confident, here with
  // requests of one frame for more samples. The fixture robot drives at about 0.5 m/s, so the speed
  // gate is opened. The localization is the recording's own AMCL: the database was built from its
  // poses, so its map frame is the one vision localizes in.
  static rclcpp::NodeOptions monitorOptions()
  {
    rclcpp::NodeOptions o = options(*db_path_);
    o.append_parameter_override("consistency.num_frames", 1);
    o.append_parameter_override("extrinsics_monitor.enabled", true);
    o.append_parameter_override("extrinsics_monitor.period", 1.0);
    o.append_parameter_override("extrinsics_monitor.min_samples", 5);
    o.append_parameter_override("extrinsics_monitor.max_linear_speed", 5.0);
    o.append_parameter_override("extrinsics_monitor.max_angular_speed", 5.0);
    return o;
  }

  // The node under test, a driver node playing the fixture, and one executor for both.
  struct Harness
  {
    explicit Harness(const rclcpp::NodeOptions & node_options)
    {
      node = std::make_shared<RelocalizerNode>(node_options);
      driver = std::make_shared<rclcpp::Node>("reloc_test_driver");
      tf_pub = driver->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
      tf_static_pub = driver->create_publisher<tf2_msgs::msg::TFMessage>(
        "/tf_static", rclcpp::QoS(1).transient_local());
      info_pub =
        driver->create_publisher<sensor_msgs::msg::CameraInfo>("/reloc_test/camera_info", 10);
      image_pub = driver->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/reloc_test/image/compressed", 10);
      map_pub = driver->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/reloc_test/map", rclcpp::QoS(1).reliable().transient_local());
      scan_pub = driver->create_publisher<sensor_msgs::msg::LaserScan>("/reloc_test/scan", 10);
      amcl_pub =
        driver->create_publisher<PoseMsg>("/reloc_test/amcl_pose", rclcpp::QoS(1).reliable());
      diagnostics_sub = driver->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10),
        [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr & array) {
          for (const auto & status : array->status) {
            if (status.name.find("relocalizer") != std::string::npos) {
              diagnostics.push_back(status);
            }
          }
        });
      status_sub = driver->create_subscription<Result>(
        "/landfall_relocalizer/status", rclcpp::QoS(1).reliable().transient_local(),
        [this](const Result::ConstSharedPtr & r) { statuses.push_back(*r); });
      client = driver->create_client<Relocalize>("/landfall_relocalizer/relocalize");
      executor.add_node(node->get_node_base_interface());
      executor.add_node(driver);
    }

    ~Harness()
    {
      if (node->get_current_state().id() != State::PRIMARY_STATE_FINALIZED) {
        node->shutdown();
      }
      executor.remove_node(node->get_node_base_interface());
      executor.remove_node(driver);
    }

    void subscribeInitialPose()
    {
      pose_sub = driver->create_subscription<PoseMsg>(
        "/reloc_test/initialpose", rclcpp::QoS(10).reliable(),
        [this](const PoseMsg::ConstSharedPtr & p) { poses.push_back(*p); });
    }

    void spinFor(std::chrono::milliseconds duration)
    {
      const auto end = Clock::now() + duration;
      while (Clock::now() < end) {
        executor.spin_some(std::chrono::milliseconds(5));
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }

    void up(bool with_map = true, bool with_camera_info = true)
    {
      ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
      ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);
      tf_static_pub->publish(tf_static);
      if (with_map) {
        map_pub->publish(*map_);
      }
      if (with_camera_info) {
        info_pub->publish(data_->camera_info);
      }
      refreshScan();
      spinFor(std::chrono::milliseconds(300));
    }

    // Republishes the last scan played: the scan check needs a live LaserScan to be ready.
    void refreshScan()
    {
      if (send_scans) {
        scan_pub->publish(data_->scans[next_scan > 0 ? next_scan - 1 : 0]);
      }
    }

    // Publishes image i with its camera_info and the TF up to just after it.
    void playImage(std::size_t i, bool with_tf = true)
    {
      const auto & image = data_->images[i % data_->images.size()];
      const int64_t stamp = landfall::ros::toNanoseconds(image.header.stamp);
      if (with_tf) {
        while (next_tf < data_->tf.size()) {
          auto message = data_->tf[next_tf];
          if (
            !message.transforms.empty() &&
            landfall::ros::toNanoseconds(message.transforms.front().header.stamp) >
              stamp + 300'000'000) {
            break;
          }
          for (auto & t : message.transforms) {
            // A localization that is wrong by map_offset_x (a kidnapped robot's AMCL).
            if (t.header.frame_id == "map") {
              t.transform.translation.x += map_offset_x;
            }
            // Injected odometry error: past each step, the robot seems odom_step further ahead.
            if (t.header.frame_id == "odom") {
              const int64_t at = landfall::ros::toNanoseconds(t.header.stamp);
              t.transform.translation.x +=
                odom_step *
                static_cast<double>(std::count_if(
                  odom_steps.begin(), odom_steps.end(), [at](int64_t step) { return step <= at; }));
            }
          }
          tf_pub->publish(message);
          ++next_tf;
        }
      }
      while (send_scans && next_scan < data_->scans.size() &&
             landfall::ros::toNanoseconds(data_->scans[next_scan].header.stamp) <=
               stamp + 300'000'000) {
        auto scan = data_->scans[next_scan];
        scan.angle_min += static_cast<float>(scan_rotation);
        scan.angle_max += static_cast<float>(scan_rotation);
        scan_pub->publish(scan);
        ++next_scan;
      }
      auto info = data_->camera_info;
      info.header.stamp = image.header.stamp;
      info_pub->publish(info);
      image_pub->publish(image);
    }

    // Calls ~/relocalize and plays images (from `first_image`) until the answer arrives.
    Result call(
      int num_frames, bool dry_run, std::size_t first_image, bool play = true, bool with_tf = true,
      float timeout = 15.0F)
    {
      EXPECT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
      auto request = std::make_shared<Relocalize::Request>();
      request->num_frames = static_cast<uint8_t>(num_frames);
      request->dry_run = dry_run;
      request->timeout = timeout;
      refreshScan();
      auto future = client->async_send_request(request);
      // Let the request start, so that its first frame is first_image.
      spinFor(std::chrono::milliseconds(200));
      std::size_t i = first_image;
      const auto deadline = Clock::now() + std::chrono::seconds(40);
      while (future.future.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
             Clock::now() < deadline) {
        if (play) {
          playImage(i++, with_tf);
        }
        spinFor(std::chrono::milliseconds(150));
      }
      EXPECT_EQ(future.future.wait_for(std::chrono::seconds(0)), std::future_status::ready);
      return future.future.get()->result;
    }

    // A localization estimate (AMCL's amcl_pose) with this standard deviation in x and y.
    void publishLocalization(double std_xy, double std_yaw_deg = 1.0)
    {
      PoseMsg pose;
      pose.header.stamp = driver->now();
      pose.header.frame_id = "map";
      pose.pose.pose.orientation.w = 1.0;
      pose.pose.covariance[0] = std_xy * std_xy;
      pose.pose.covariance[7] = std_xy * std_xy;
      pose.pose.covariance[35] = std::pow(std_yaw_deg * M_PI / 180.0, 2);
      amcl_pub->publish(pose);
    }

    // Plays images from `first` for `duration`, with a localization estimate before each.
    std::size_t playFor(
      std::size_t first, std::chrono::milliseconds duration, std::optional<double> std_xy)
    {
      const auto end = Clock::now() + duration;
      std::size_t i = first;
      while (Clock::now() < end) {
        if (std_xy) {
          publishLocalization(*std_xy);
        }
        playImage(i++);
        spinFor(std::chrono::milliseconds(150));
      }
      return i;
    }

    std::size_t countTrigger(const std::string & trigger) const
    {
      std::size_t n = 0;
      for (const auto & r : statuses) {
        n += r.trigger == trigger ? 1U : 0U;
      }
      return n;
    }

    // The value of `key` in the latest relocalizer status on /diagnostics ("" if none).
    std::string diagnosticValue(const std::string & key) const
    {
      if (diagnostics.empty()) {
        return "";
      }
      for (const auto & kv : diagnostics.back().values) {
        if (kv.key == key) {
          return kv.value;
        }
      }
      return "";
    }

    std::shared_ptr<RelocalizerNode> node;
    rclcpp::Node::SharedPtr driver;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<PoseMsg>::SharedPtr amcl_pub;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub;
    rclcpp::Subscription<Result>::SharedPtr status_sub;
    rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub;
    rclcpp::Client<Relocalize>::SharedPtr client;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::vector<Result> statuses;
    std::vector<PoseMsg> poses;
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> diagnostics;
    tf2_msgs::msg::TFMessage tf_static = data_->tf_static;  ///< what up() publishes
    std::size_t next_tf = 0;
    std::size_t next_scan = 0;
    bool send_scans = true;
    double scan_rotation = 0.0;  ///< rad added to every scan's angles (a scan that doesn't fit)
    std::vector<int64_t> odom_steps;  ///< stamps where the odometry jumps ahead by odom_step
    double odom_step = 0.0;           ///< m
    double map_offset_x = 0.0;        ///< m added to the recorded map -> odom
  };

  // Error of a published map pose against the ground truth at its stamp.
  static void expectNearTruth(const PoseMsg & pose, double max_position, double max_yaw_deg)
  {
    const auto truth =
      data_->truth.at(landfall::ros::toNanoseconds(pose.header.stamp), 250'000'000);
    ASSERT_TRUE(truth.has_value());
    const landfall::Pose2 expected = landfall::compose(landfall::test::kWorldToMap, *truth);
    const landfall::Pose2 got = landfall::toPose2(landfall::ros::toPose3(pose.pose.pose));
    EXPECT_LT(landfall::translationDistance(got, expected), max_position);
    EXPECT_LT(landfall::rotationDistance(got, expected) * 180.0 / M_PI, max_yaw_deg);
  }

  static landfall::test::TempDir * dir_;
  static std::string * db_path_;
  static landfall::test::FixtureMessages * data_;
  static nav_msgs::msg::OccupancyGrid * map_;
};

landfall::test::TempDir * RelocalizerNodeTest::dir_ = nullptr;
std::string * RelocalizerNodeTest::db_path_ = nullptr;
landfall::test::FixtureMessages * RelocalizerNodeTest::data_ = nullptr;
nav_msgs::msg::OccupancyGrid * RelocalizerNodeTest::map_ = nullptr;

}  // namespace

TEST_F(RelocalizerNodeTest, LocalizesPublishesAndDryRuns)
{
  Harness h(options(*db_path_));
  h.subscribeInitialPose();
  h.up();
  const Result result = h.call(2, false, 4);
  ASSERT_EQ(result.status, Result::SUCCESS) << result.message;
  EXPECT_TRUE(result.published);
  EXPECT_EQ(result.trigger, "service");
  EXPECT_EQ(result.scores.frames_used, 2U);
  EXPECT_GE(result.scores.num_inliers, 40U);
  EXPECT_GE(result.scores.scan_hit_ratio, 0.85F);  // the LiDAR check ran (required by default)
  EXPECT_FALSE(result.hypotheses.empty());
  EXPECT_EQ(result.pose.header.frame_id, "map");
  h.spinFor(std::chrono::milliseconds(300));
  ASSERT_EQ(h.poses.size(), 1U);
  expectNearTruth(h.poses.front(), 0.25, 5.0);
  // The estimated covariance: x, y and yaw, symmetric, at least the map's error (2.5 cm, 0.2 deg),
  // and centimetres rather than the fixed 25 cm from landmarks a few metres away. No entry negative:
  // Nav2's AMCL from Lyrical on rejects the pose otherwise.
  const auto & c = h.poses.front().pose.covariance;
  EXPECT_EQ(c[1], c[6]);
  EXPECT_EQ(c[5], c[30]);
  EXPECT_EQ(c[11], c[31]);
  for (const double entry : c) {
    EXPECT_GE(entry, 0.0);
  }
  EXPECT_GE(c[0], 0.025 * 0.025);
  EXPECT_GE(c[7], 0.025 * 0.025);
  EXPECT_GE(c[35], std::pow(0.2 * M_PI / 180.0, 2));
  EXPECT_LT(c[0] + c[7], 2.0 * 0.1 * 0.1);
  EXPECT_LT(c[35], std::pow(2.0 * M_PI / 180.0, 2));
  EXPECT_GT(c[0] * c[7] - c[1] * c[1], 0.0);
  EXPECT_EQ(c[14] + c[21] + c[28], 0.0);  // z, roll and pitch aren't estimated
  ASSERT_FALSE(h.statuses.empty());
  EXPECT_EQ(h.statuses.back().status, Result::SUCCESS);

  // A dry run runs every check and publishes nothing; with output.covariance fixed its pose has
  // the configured circle.
  ASSERT_TRUE(h.node->set_parameter(rclcpp::Parameter("output.covariance", "fixed")).successful);
  const Result dry = h.call(2, true, 30);
  ASSERT_EQ(dry.status, Result::SUCCESS) << dry.message;
  EXPECT_FALSE(dry.published);
  expectNearTruth(dry.pose, 0.25, 5.0);
  EXPECT_EQ(dry.pose.pose.covariance[0], 0.25 * 0.25);
  EXPECT_EQ(dry.pose.pose.covariance[1], 0.0);
  EXPECT_DOUBLE_EQ(dry.pose.pose.covariance[35], std::pow(5.0 * M_PI / 180.0, 2));
  h.spinFor(std::chrono::milliseconds(300));
  EXPECT_EQ(h.poses.size(), 1U);
}

TEST_F(RelocalizerNodeTest, NotReadyWithoutDatabase)
{
  Harness h(options(dir_->file("missing.landfall.db")));
  h.subscribeInitialPose();
  h.up();  // configure succeeds: a broken database must not block AMCL's lifecycle manager
  const Result r = h.call(1, false, 0, false);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("map database not loaded"), std::string::npos) << r.message;
}

TEST_F(RelocalizerNodeTest, NotReadyWithoutOrWithAnotherMap)
{
  Harness h(options(*db_path_));
  h.subscribeInitialPose();
  h.up(false);
  Result r = h.call(1, true, 0, false);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("no occupancy grid"), std::string::npos) << r.message;

  auto other = *map_;
  other.data[other.data.size() / 2] = other.data[other.data.size() / 2] == 100 ? 0 : 100;
  h.map_pub->publish(other);
  h.spinFor(std::chrono::milliseconds(300));
  r = h.call(1, true, 0, false);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("1 cells differ"), std::string::npos) << r.message;

  other = *map_;
  other.info.origin.position.x += 0.05;
  h.map_pub->publish(other);
  h.spinFor(std::chrono::milliseconds(300));
  r = h.call(1, true, 0, false);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("origin"), std::string::npos) << r.message;
}

TEST_F(RelocalizerNodeTest, NotReadyWithoutCameraInfoOrWithAnotherCamera)
{
  Harness h(options(*db_path_));
  h.subscribeInitialPose();
  h.up(true, false);
  Result r = h.call(1, true, 0, false);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("no camera_info"), std::string::npos) << r.message;

  auto info = data_->camera_info;
  info.width /= 2;
  info.height /= 2;
  h.info_pub->publish(info);
  h.spinFor(std::chrono::milliseconds(300));
  r = h.call(1, true, 0, false);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("the database was built for"), std::string::npos) << r.message;
}

TEST_F(RelocalizerNodeTest, NotReadyWithoutTfOrImages)
{
  rclcpp::NodeOptions o = options(*db_path_);
  o.append_parameter_override("tf_timeout", 0.05);
  Harness h(o);
  h.subscribeInitialPose();
  h.up();
  // Images keep coming, but without odometry.
  Result r = h.call(1, true, 0, true, false, 3.0F);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("no transform"), std::string::npos) << r.message;

  // No images at all.
  r = h.call(1, true, 0, false, true, 2.0F);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("no image"), std::string::npos) << r.message;
}

TEST_F(RelocalizerNodeTest, NotReadyWhenNothingCanReceiveThePose)
{
  rclcpp::NodeOptions o = options(*db_path_);
  o.append_parameter_override("output.amcl_node", "/reloc_test_amcl");
  Harness h(o);
  h.up();
  // Nobody subscribes to the output topic.
  Result r = h.call(1, false, 0, false);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("nothing subscribes"), std::string::npos) << r.message;

  // A subscriber, but no AMCL.
  h.subscribeInitialPose();
  h.spinFor(std::chrono::milliseconds(300));
  r = h.call(1, false, 0, false);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("not running"), std::string::npos) << r.message;

  // AMCL exists but is inactive.
  uint8_t amcl_state = State::PRIMARY_STATE_INACTIVE;
  auto amcl = h.driver->create_service<lifecycle_msgs::srv::GetState>(
    "/reloc_test_amcl/get_state",
    [&amcl_state](
      const std::shared_ptr<lifecycle_msgs::srv::GetState::Request>,        // NOLINT
      std::shared_ptr<lifecycle_msgs::srv::GetState::Response> response) {  // NOLINT
      response->current_state.id = amcl_state;
      response->current_state.label =
        amcl_state == State::PRIMARY_STATE_ACTIVE ? "active" : "inactive";
    });
  h.spinFor(std::chrono::milliseconds(300));
  r = h.call(1, false, 0, false);
  EXPECT_EQ(r.status, Result::NOT_READY);
  EXPECT_NE(r.message.find("inactive, not active"), std::string::npos) << r.message;

  // Active: the request runs and the pose goes out.
  amcl_state = State::PRIMARY_STATE_ACTIVE;
  r = h.call(2, false, 4);
  EXPECT_EQ(r.status, Result::SUCCESS) << r.message;
  EXPECT_TRUE(r.published);
}

TEST_F(RelocalizerNodeTest, SecondRequestIsBusyAndDeactivationStopsTheFirst)
{
  Harness h(options(*db_path_));
  h.subscribeInitialPose();
  h.up();
  ASSERT_TRUE(h.client->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<Relocalize::Request>();
  request->num_frames = 2;
  request->timeout = 30.0F;
  auto first = h.client->async_send_request(request);  // no images: it waits
  h.spinFor(std::chrono::milliseconds(300));
  auto second = h.client->async_send_request(request);
  h.spinFor(std::chrono::milliseconds(300));
  ASSERT_EQ(second.future.wait_for(std::chrono::seconds(0)), std::future_status::ready);
  EXPECT_EQ(second.future.get()->result.status, Result::BUSY);
  EXPECT_EQ(first.future.wait_for(std::chrono::seconds(0)), std::future_status::timeout);

  // Deactivating stops the running request; it still gets an answer.
  ASSERT_EQ(h.node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  h.spinFor(std::chrono::milliseconds(300));
  ASSERT_EQ(first.future.wait_for(std::chrono::seconds(0)), std::future_status::ready);
  const Result stopped = first.future.get()->result;
  EXPECT_EQ(stopped.status, Result::NOT_READY);
  EXPECT_NE(stopped.message.find("stopped"), std::string::npos) << stopped.message;

  // The usual transitions after that.
  EXPECT_EQ(h.node->activate().id(), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(h.node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(h.node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(h.node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(h.node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(RelocalizerNodeTest, StartupRelocalizationPublishesOnce)
{
  rclcpp::NodeOptions o = options(*db_path_);
  o.append_parameter_override("startup.enabled", true);
  o.append_parameter_override("startup.delay", 0.5);
  o.append_parameter_override("startup.retry_interval", 1.0);
  Harness h(o);
  h.subscribeInitialPose();
  h.up();
  const auto deadline = Clock::now() + std::chrono::seconds(30);
  std::size_t i = 4;
  while (h.poses.empty() && Clock::now() < deadline) {
    h.playImage(i++);
    h.spinFor(std::chrono::milliseconds(150));
  }
  ASSERT_EQ(h.poses.size(), 1U);
  expectNearTruth(h.poses.front(), 0.25, 5.0);
  h.spinFor(std::chrono::milliseconds(300));
  ASSERT_FALSE(h.statuses.empty());
  EXPECT_EQ(h.statuses.back().trigger, "startup");
  EXPECT_EQ(h.statuses.back().status, Result::SUCCESS);
  // Done after one success: more images publish nothing more.
  for (int k = 0; k < 15; ++k) {
    h.playImage(i++);
    h.spinFor(std::chrono::milliseconds(150));
  }
  EXPECT_EQ(h.poses.size(), 1U);
}

TEST_F(RelocalizerNodeTest, ScanCheckModes)
{
  {
    // Required (the default): no request without a LaserScan.
    Harness h(options(*db_path_));
    h.send_scans = false;
    h.subscribeInitialPose();
    h.up();
    const Result r = h.call(1, true, 4);
    EXPECT_EQ(r.status, Result::NOT_READY);
    EXPECT_NE(r.message.find("no LaserScan on '/reloc_test/scan'"), std::string::npos) << r.message;
  }
  {
    // A scan that doesn't fit the map where vision puts the robot.
    Harness h(options(*db_path_));
    h.scan_rotation = M_PI / 2.0;
    h.subscribeInitialPose();
    h.up();
    const Result r = h.call(2, false, 4);
    EXPECT_EQ(r.status, Result::REJECTED_SCAN_CHECK) << r.message;
    EXPECT_FALSE(r.published);
    h.spinFor(std::chrono::milliseconds(300));
    EXPECT_TRUE(h.poses.empty());
  }
  for (const char * mode : {"optional", "disabled"}) {
    // Vision checks only.
    rclcpp::NodeOptions o = options(*db_path_);
    o.append_parameter_override("scan_check.mode", std::string(mode));
    Harness h(o);
    h.send_scans = false;
    h.up();
    const Result r = h.call(2, true, 4);
    EXPECT_EQ(r.status, Result::SUCCESS) << mode << ": " << r.message;
    EXPECT_EQ(r.scores.scan_hit_ratio, 0.0F) << mode;
  }
}

TEST_F(RelocalizerNodeTest, OdometryThatDisagreesIsRejected)
{
  Harness h(options(*db_path_));
  h.subscribeInitialPose();
  h.up();
  // From image 11 on (the first that localizes), odometry jumps 0.5 m ahead halfway between every
  // two images: whichever two frames the request takes, they disagree. Each image's scan and
  // odometry samples (within 0.2 s; the images are 0.8 s apart) stay on the image's step.
  for (std::size_t k = 11; k < 40; ++k) {
    h.odom_steps.push_back(
      (landfall::ros::toNanoseconds(data_->images[k].header.stamp) +
       landfall::ros::toNanoseconds(data_->images[k + 1].header.stamp)) /
      2);
  }
  h.odom_step = 0.5;
  const Result r = h.call(2, false, 11);
  EXPECT_EQ(r.status, Result::REJECTED_CONSISTENCY) << r.message;
  EXPECT_FALSE(r.published);
  EXPECT_GT(r.scores.consistency_translation_error_m, 0.3F);
  h.spinFor(std::chrono::milliseconds(300));
  EXPECT_TRUE(h.poses.empty());
}

TEST_F(RelocalizerNodeTest, AutoTriggerFiresOnceAndHonorsTheCooldown)
{
  rclcpp::NodeOptions o = options(*db_path_);
  o.append_parameter_override("auto_trigger.enabled", true);
  o.append_parameter_override("auto_trigger.duration", 1.0);
  o.append_parameter_override("auto_trigger.cooldown", 600.0);
  Harness h(o);
  h.subscribeInitialPose();
  h.up();
  // A confident localization: nothing happens.
  std::size_t i = h.playFor(4, std::chrono::milliseconds(2500), 0.02);
  EXPECT_EQ(h.countTrigger("auto"), 0U);
  // Uncertain from now on: one request after a second, then the cooldown holds.
  i = h.playFor(i, std::chrono::milliseconds(9000), 1.0);
  ASSERT_EQ(h.countTrigger("auto"), 1U);
  EXPECT_EQ(h.statuses.back().status, Result::SUCCESS) << h.statuses.back().message;
  EXPECT_EQ(h.poses.size(), 1U);
  // A shorter cooldown applies at once: it fires again. The localization stays uncertain, so it
  // keeps firing once a second; a slow machine can see more than one request before it looks.
  ASSERT_TRUE(h.node->set_parameter(rclcpp::Parameter("auto_trigger.cooldown", 1.0)).successful);
  const auto deadline = Clock::now() + std::chrono::seconds(20);
  while (h.countTrigger("auto") < 2 && Clock::now() < deadline) {
    i = h.playFor(i, std::chrono::milliseconds(150), 1.0);
  }
  EXPECT_GE(h.countTrigger("auto"), 2U);
}

TEST_F(RelocalizerNodeTest, ExtrinsicsMonitorIsSilentOnTheRecording)
{
  Harness h(monitorOptions());
  h.subscribeInitialPose();
  h.up();
  h.playFor(4, std::chrono::milliseconds(11000), 0.02);
  h.spinFor(std::chrono::milliseconds(1200));  // one more diagnostics period
  const std::string monitor = h.diagnosticValue("extrinsics_monitor");
  std::cout << "extrinsics monitor on the recording: " << monitor << "\n";
  ASSERT_EQ(monitor.rfind("camera ", 0), 0U) << monitor;  // enough samples for an estimate
  EXPECT_EQ(h.diagnostics.back().level, diagnostic_msgs::msg::DiagnosticStatus::OK)
    << h.diagnostics.back().message;
  // Dry runs: no pose, and the samples aren't relocalizations.
  EXPECT_TRUE(h.poses.empty());
  EXPECT_EQ(h.countTrigger("monitor"), 0U);
}

TEST_F(RelocalizerNodeTest, ExtrinsicsMonitorFlagsACameraThatMoved)
{
  Harness h(monitorOptions());
  // The camera turned 2 degrees and slid 3 cm sideways on its mount; TF still says otherwise.
  bool found = false;
  for (auto & t : h.tf_static.transforms) {
    if (t.child_frame_id == "oakd_rgb_camera_frame") {
      t.transform.translation.y += 0.03;
      const double half = 0.5 * 2.0 * M_PI / 180.0;
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = std::sin(half);
      t.transform.rotation.w = std::cos(half);
      found = true;
    }
  }
  ASSERT_TRUE(found);
  h.subscribeInitialPose();
  h.up();
  h.playFor(4, std::chrono::milliseconds(11000), 0.02);
  h.spinFor(std::chrono::milliseconds(1200));
  const std::string monitor = h.diagnosticValue("extrinsics_monitor");
  std::cout << "extrinsics monitor with the moved camera: " << monitor << "\n";
  ASSERT_EQ(monitor.rfind("camera ", 0), 0U) << monitor;
  EXPECT_GE(h.diagnostics.back().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_NE(h.diagnostics.back().message.find("extrinsics monitor: camera"), std::string::npos)
    << h.diagnostics.back().message;
}

TEST_F(RelocalizerNodeTest, ServiceRequestsPreemptTheMonitor)
{
  Harness h(monitorOptions());
  h.subscribeInitialPose();
  h.up();
  // After a second of confident localization a monitor sample starts; without images it waits
  // (up to 5 s). /diagnostics shows it once a second.
  const auto end = Clock::now() + std::chrono::seconds(5);
  while (Clock::now() < end &&
         (h.diagnostics.empty() || h.diagnostics.back().message != "relocalizing")) {
    h.publishLocalization(0.02);
    h.refreshScan();
    h.spinFor(std::chrono::milliseconds(200));
  }
  ASSERT_FALSE(h.diagnostics.empty());
  ASSERT_EQ(h.diagnostics.back().message, "relocalizing");
  // A service request doesn't get BUSY: the sample stops and the request runs.
  const Result r = h.call(2, true, 11);
  EXPECT_EQ(r.status, Result::SUCCESS) << r.message;
  EXPECT_EQ(r.trigger, "service");
}

namespace
{

// The watchdog checks once a second, with requests of one frame. The localization is the
// recording's own AMCL (its map -> odom), confident throughout.
rclcpp::NodeOptions watchdogOptions(rclcpp::NodeOptions o)
{
  o.append_parameter_override("consistency.num_frames", 1);
  o.append_parameter_override("kidnap_watchdog.enabled", true);
  o.append_parameter_override("kidnap_watchdog.period", 1.0);
  o.append_parameter_override("kidnap_watchdog.confirmations", 2);
  o.append_parameter_override("kidnap_watchdog.cooldown", 600.0);
  return o;
}

}  // namespace

TEST_F(RelocalizerNodeTest, KidnapWatchdogRelocalizesAWrongLocalization)
{
  Harness h(watchdogOptions(options(*db_path_)));
  // The localization is confident and 3 m off, as after a kidnap: two checks in a row disagree,
  // and the watchdog's relocalization publishes vision's pose. The recorded localization doesn't
  // take it, so the checks keep disagreeing, but the cooldown holds.
  h.map_offset_x = 3.0;
  h.subscribeInitialPose();
  h.up();
  h.playFor(4, std::chrono::milliseconds(9000), 0.02);
  h.spinFor(std::chrono::milliseconds(1200));  // one more diagnostics period
  ASSERT_EQ(h.countTrigger("watchdog"), 1U);
  const auto it = std::find_if(
    h.statuses.begin(), h.statuses.end(), [](const Result & r) { return r.trigger == "watchdog"; });
  ASSERT_NE(it, h.statuses.end());
  EXPECT_EQ(it->status, Result::SUCCESS) << it->message;
  EXPECT_TRUE(it->published);
  ASSERT_EQ(h.poses.size(), 1U);
  expectNearTruth(h.poses.front(), 0.25, 5.0);
  // The checks themselves are no relocalizations.
  EXPECT_EQ(h.statuses.size(), 1U);
  const std::string watchdog = h.diagnosticValue("kidnap_watchdog");
  EXPECT_NE(watchdog.find("relocalizes at 2"), std::string::npos) << watchdog;
}

TEST_F(RelocalizerNodeTest, KidnapWatchdogIsSilentWhileTheLocalizationIsRight)
{
  Harness h(watchdogOptions(options(*db_path_)));
  h.subscribeInitialPose();
  h.up();
  h.playFor(4, std::chrono::milliseconds(7000), 0.02);
  h.spinFor(std::chrono::milliseconds(1200));
  EXPECT_EQ(h.countTrigger("watchdog"), 0U);
  EXPECT_TRUE(h.poses.empty());
  EXPECT_TRUE(h.statuses.empty());
  const std::string watchdog = h.diagnosticValue("kidnap_watchdog");
  EXPECT_EQ(watchdog.rfind("0 disagreeing", 0), 0U) << watchdog;
}

TEST_F(RelocalizerNodeTest, ValidatesParameters)
{
  auto node = std::make_shared<RelocalizerNode>(options(*db_path_));
  // Thresholds change at runtime, within their ranges and enums.
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("sanity.min_inliers", 60)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("sanity.min_inliers", 2)).successful);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("pnp.solver", "p3p")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("pnp.solver", "epnp")).successful);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("pnp.refinement", "free")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("pnp.refinement", "planar")).successful);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("output.covariance", "fixed")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("output.covariance", "large")).successful);
  // Structural parameters are read-only.
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("database_path", "/tmp/x.db")).successful);
  EXPECT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);

  rclcpp::NodeOptions bad = options(*db_path_);
  bad.append_parameter_override("output.mode", "carrier_pigeon");
  auto invalid = std::make_shared<RelocalizerNode>(bad);
  EXPECT_EQ(invalid->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(invalid->shutdown().id(), State::PRIMARY_STATE_FINALIZED);

  rclcpp::NodeOptions out_of_range = options(*db_path_);
  out_of_range.append_parameter_override("query.top_k", 0);
  EXPECT_ANY_THROW(std::make_shared<RelocalizerNode>(out_of_range));
}
