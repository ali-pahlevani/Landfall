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
// publishing to AMCL's topic, dry runs, the startup trigger, lifecycle transitions and parameter
// validation.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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
    });
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
      tf_static_pub->publish(data_->tf_static);
      if (with_map) {
        map_pub->publish(*map_);
      }
      if (with_camera_info) {
        info_pub->publish(data_->camera_info);
      }
      spinFor(std::chrono::milliseconds(300));
    }

    // Publishes image i with its camera_info and the TF up to just after it.
    void playImage(std::size_t i, bool with_tf = true)
    {
      const auto & image = data_->images[i % data_->images.size()];
      const int64_t stamp = landfall::ros::toNanoseconds(image.header.stamp);
      if (with_tf) {
        while (next_tf < data_->tf.size()) {
          const auto & message = data_->tf[next_tf];
          if (
            !message.transforms.empty() &&
            landfall::ros::toNanoseconds(message.transforms.front().header.stamp) >
              stamp + 300'000'000) {
            break;
          }
          tf_pub->publish(message);
          ++next_tf;
        }
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
      auto future = client->async_send_request(request);
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

    std::shared_ptr<RelocalizerNode> node;
    rclcpp::Node::SharedPtr driver;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Subscription<Result>::SharedPtr status_sub;
    rclcpp::Subscription<PoseMsg>::SharedPtr pose_sub;
    rclcpp::Client<Relocalize>::SharedPtr client;
    rclcpp::executors::SingleThreadedExecutor executor;
    std::vector<Result> statuses;
    std::vector<PoseMsg> poses;
    std::size_t next_tf = 0;
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
  EXPECT_FALSE(result.hypotheses.empty());
  EXPECT_EQ(result.pose.header.frame_id, "map");
  h.spinFor(std::chrono::milliseconds(300));
  ASSERT_EQ(h.poses.size(), 1U);
  EXPECT_EQ(h.poses.front().pose.covariance[0], 0.25 * 0.25);
  expectNearTruth(h.poses.front(), 0.25, 5.0);
  ASSERT_FALSE(h.statuses.empty());
  EXPECT_EQ(h.statuses.back().status, Result::SUCCESS);

  // A dry run runs every check and publishes nothing.
  const Result dry = h.call(2, true, 30);
  ASSERT_EQ(dry.status, Result::SUCCESS) << dry.message;
  EXPECT_FALSE(dry.published);
  expectNearTruth(dry.pose, 0.25, 5.0);
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

TEST_F(RelocalizerNodeTest, ValidatesParameters)
{
  auto node = std::make_shared<RelocalizerNode>(options(*db_path_));
  // Thresholds change at runtime, within their ranges and enums.
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("sanity.min_inliers", 60)).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("sanity.min_inliers", 2)).successful);
  EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("pnp.solver", "p3p")).successful);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("pnp.solver", "epnp")).successful);
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
