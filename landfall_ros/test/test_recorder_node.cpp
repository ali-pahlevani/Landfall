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

// The recorder node in-process: lifecycle, live TF, a session written to disk, status, pause.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/record/staging.hpp"
#include "landfall/ros/recorder_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{

namespace fs = std::filesystem;
using lifecycle_msgs::msg::State;

class TempDir
{
public:
  TempDir()
  {
    std::string pattern = (fs::temp_directory_path() / "landfall_rec_XXXXXX").string();
    if (mkdtemp(pattern.data()) == nullptr) {
      throw std::runtime_error("mkdtemp failed");
    }
    path = pattern;
  }
  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;
  ~TempDir() { fs::remove_all(path); }
  std::string path;
};

geometry_msgs::msg::TransformStamped transform(
  const rclcpp::Time & stamp, const std::string & parent, const std::string & child, double x)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.rotation.w = 1.0;
  return t;
}

void spinFor(rclcpp::Executor & executor, std::chrono::milliseconds duration)
{
  const auto end = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < end) {
    executor.spin_some(std::chrono::milliseconds(5));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

rclcpp::NodeOptions recorderOptions(const std::string & output_dir)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"output_dir", output_dir},
    {"session_name", "test_session"},
    {"camera_info.topic", "/rec_test/camera_info"},
    {"image.topic", "/rec_test/image"},
    {"pose_source.covariance_topic", "/rec_test/amcl_pose"},
    {"imu.topic", "/rec_test/imu"},
    {"gating.min_translation", 0.2},
    {"bond.enabled", false},
  });
  return options;
}

class RecorderNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

}  // namespace

TEST_F(RecorderNodeTest, RecordsAndClosesASession)
{
  TempDir dir;
  auto recorder = std::make_shared<landfall::ros::RecorderNode>(recorderOptions(dir.path));
  auto driver = std::make_shared<rclcpp::Node>("rec_test_driver");
  auto tf_pub = driver->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100));
  auto tf_static_pub = driver->create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static", rclcpp::QoS(1).transient_local());
  auto info_pub =
    driver->create_publisher<sensor_msgs::msg::CameraInfo>("/rec_test/camera_info", 10);
  auto image_pub = driver->create_publisher<sensor_msgs::msg::Image>("/rec_test/image", 10);
  auto pose_pub = driver->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/rec_test/amcl_pose", 10);
  landfall_msgs::msg::RecorderStatus last_status;
  int statuses = 0;
  auto status_sub = driver->create_subscription<landfall_msgs::msg::RecorderStatus>(
    "/landfall_recorder/status", rclcpp::QoS(1).reliable().transient_local(),
    [&](const landfall_msgs::msg::RecorderStatus::ConstSharedPtr & status) {
      last_status = *status;
      ++statuses;
    });
  auto pause = driver->create_client<std_srvs::srv::SetBool>("/landfall_recorder/set_paused");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder->get_node_base_interface());
  executor.add_node(driver);
  ASSERT_EQ(recorder->configure().id(), State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(recorder->activate().id(), State::PRIMARY_STATE_ACTIVE);

  tf2_msgs::msg::TFMessage statics;
  geometry_msgs::msg::TransformStamped camera =
    transform(driver->now(), "base_footprint", "cam", 0.1);
  camera.transform.translation.z = 0.5;
  camera.transform.rotation.x = -0.5;  // optical axes: z forward, x right, y down
  camera.transform.rotation.y = 0.5;
  camera.transform.rotation.z = -0.5;
  camera.transform.rotation.w = 0.5;
  statics.transforms.push_back(camera);
  tf_static_pub->publish(statics);

  cv::RNG rng(3);
  for (int k = 0; k < 40; ++k) {  // 4 s at 10 Hz, 0.4 m/s
    const rclcpp::Time stamp = driver->now();
    tf2_msgs::msg::TFMessage tf;
    tf.transforms.push_back(transform(stamp, "map", "odom", 0.0));
    tf.transforms.push_back(transform(stamp, "odom", "base_footprint", 0.04 * k));
    tf_pub->publish(tf);
    if (k % 3 == 0) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = "map";
      pose.pose.covariance[0] = pose.pose.covariance[7] = 0.01 * 0.01;
      pose.pose.covariance[35] = 0.0001;
      pose_pub->publish(pose);
    }
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = "cam";
    info.width = 160;
    info.height = 120;
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.k = {120.0, 0.0, 79.5, 0.0, 120.0, 59.5, 0.0, 0.0, 1.0};
    info_pub->publish(info);
    sensor_msgs::msg::Image image;
    image.header = info.header;
    image.width = 160;
    image.height = 120;
    image.encoding = "mono8";
    image.step = 160;
    cv::Mat pixels(120, 160, CV_8UC1);
    rng.fill(pixels, cv::RNG::UNIFORM, 30, 220);
    image.data.assign(pixels.data, pixels.data + pixels.total());
    image_pub->publish(image);
    spinFor(executor, std::chrono::milliseconds(100));
  }
  spinFor(executor, std::chrono::milliseconds(500));
  EXPECT_GT(statuses, 0);
  EXPECT_TRUE(last_status.recording);
  EXPECT_EQ(last_status.session, "test_session");
  EXPECT_GE(last_status.keyframes, 4U);
  EXPECT_EQ(last_status.last_keyframe_pose.header.frame_id, "map");

  // Pause: the service answers and the status says so.
  ASSERT_TRUE(pause->wait_for_service(std::chrono::seconds(2)));
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto response = pause->async_send_request(request);
  executor.spin_until_future_complete(response, std::chrono::seconds(2));
  ASSERT_TRUE(response.get()->success);
  spinFor(executor, std::chrono::milliseconds(200));
  EXPECT_TRUE(last_status.paused);
  EXPECT_FALSE(last_status.recording);

  ASSERT_EQ(recorder->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  const auto folder = landfall::readStaging(dir.path + "/test_session");
  EXPECT_TRUE(folder.session.complete);
  EXPECT_EQ(folder.session.frames.camera, "cam");
  EXPECT_EQ(folder.session.camera.width(), 160);
  EXPECT_EQ(folder.keyframes.size(), last_status.keyframes);
  EXPECT_GE(folder.keyframes.size(), 4U);
  for (std::size_t i = 1; i < folder.keyframes.size(); ++i) {
    EXPECT_GE(folder.keyframes[i].map_base.x - folder.keyframes[i - 1].map_base.x, 0.2 - 1e-9);
  }
  EXPECT_NEAR(folder.keyframes.front().T_base_cam.translation().z(), 0.5, 1e-12);
  EXPECT_TRUE(fs::exists(dir.path + "/test_session/" + folder.keyframes.front().image));

  // A second activation is a new session folder.
  ASSERT_EQ(recorder->activate().id(), State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(recorder->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(recorder->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(recorder->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(RecorderNodeTest, RejectsInvalidParameters)
{
  TempDir dir;
  rclcpp::NodeOptions bad_enum = recorderOptions(dir.path);
  bad_enum.append_parameter_override("image.transport", "jpeg");
  auto node = std::make_shared<landfall::ros::RecorderNode>(bad_enum);
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);

  rclcpp::NodeOptions out_of_range = recorderOptions(dir.path);
  out_of_range.append_parameter_override("image.jpeg_quality", 10);
  EXPECT_ANY_THROW(std::make_shared<landfall::ros::RecorderNode>(out_of_range));
}
