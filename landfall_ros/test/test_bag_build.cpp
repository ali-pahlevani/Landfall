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

// Keyframe selection from a bag written in the test, and the `landfall` CLI on it.

#include <gtest/gtest.h>
#include <sys/wait.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/ros/bag_reader.hpp"
#include "landfall/ros/conversions.hpp"
#include "rclcpp/time.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace
{

namespace fs = std::filesystem;

constexpr int64_t kStart = 1'000'000'000'000;  // 1000 s
constexpr double kMapOdomYaw = 0.1;

class TempDir
{
public:
  TempDir()
  {
    std::string pattern = (fs::temp_directory_path() / "landfall_bag_XXXXXX").string();
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

// Robot x along odom: 0.25 m/s, except 1.2 m/s between 14 s and 16 s.
double odomX(double t)
{
  if (t < 14.0) {
    return 0.25 * t;
  }
  if (t < 16.0) {
    return 3.5 + 1.2 * (t - 14.0);
  }
  return 5.9 + 0.25 * (t - 16.0);
}

geometry_msgs::msg::TransformStamped transform(
  int64_t stamp, const std::string & parent, const std::string & child, double x, double y,
  double z, double yaw, double pitch = 0.0)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = landfall::ros::toStamp(stamp);
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  const Eigen::Quaterniond q(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  return t;
}

// 20 s drive; images and camera_info at 5 Hz; amcl_pose at 2 Hz with a 20 cm std between 8 and
// 10 s; map->odom at 10 Hz, odom->base at 50 Hz, base->camera static.
void writeBag(const std::string & uri)
{
  rosbag2_storage::StorageOptions options;
  options.uri = uri;
  options.storage_id = "sqlite3";
  rosbag2_cpp::Writer writer;
  writer.open(options);
  tf2_msgs::msg::TFMessage statics;
  // Camera 0.2 m ahead, 0.3 m up, optical axes (z forward, x right, y down).
  geometry_msgs::msg::TransformStamped base_cam =
    transform(kStart, "base_footprint", "camera_optical", 0.2, 0.0, 0.3, 0.0);
  const Eigen::Quaterniond q_optical((Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished());
  base_cam.transform.rotation.x = q_optical.x();
  base_cam.transform.rotation.y = q_optical.y();
  base_cam.transform.rotation.z = q_optical.z();
  base_cam.transform.rotation.w = q_optical.w();
  statics.transforms.push_back(base_cam);
  writer.write(statics, "/tf_static", rclcpp::Time(kStart));

  cv::RNG rng(7);
  for (int step = 0; step <= 1000; ++step) {  // 20 ms ticks
    const double t = step * 0.02;
    const int64_t stamp = kStart + static_cast<int64_t>(step) * 20'000'000;
    tf2_msgs::msg::TFMessage tf;
    tf.transforms.push_back(transform(stamp, "odom", "base_footprint", odomX(t), 0.0, 0.0, 0.0));
    if (step % 5 == 0) {
      tf.transforms.push_back(transform(stamp, "map", "odom", 1.0, 2.0, 0.0, kMapOdomYaw));
    }
    writer.write(tf, "/tf", rclcpp::Time(stamp));
    if (step % 25 == 0) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header.stamp = landfall::ros::toStamp(stamp);
      pose.header.frame_id = "map";
      const double sigma = (t >= 8.0 && t < 10.0) ? 0.2 : 0.02;
      pose.pose.covariance[0] = pose.pose.covariance[7] = sigma * sigma;
      pose.pose.covariance[35] = std::pow(0.5 * M_PI / 180.0, 2);
      writer.write(pose, "/amcl_pose", rclcpp::Time(stamp));
    }
    if (step % 10 == 5) {  // images at 5 Hz, offset from the TF ticks
      sensor_msgs::msg::CameraInfo info;
      info.header.stamp = landfall::ros::toStamp(stamp);
      info.header.frame_id = "camera_optical";
      info.width = 320;
      info.height = 240;
      info.distortion_model = "plumb_bob";
      info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
      info.k = {250.0, 0.0, 159.5, 0.0, 250.0, 119.5, 0.0, 0.0, 1.0};
      writer.write(info, "/camera/camera_info", rclcpp::Time(stamp));
      cv::Mat image(240, 320, CV_8UC3);
      rng.fill(image, cv::RNG::UNIFORM, 30, 220);
      sensor_msgs::msg::CompressedImage compressed;
      compressed.header = info.header;
      compressed.format = "bgr8; jpeg compressed bgr8";
      std::vector<uint8_t> jpeg;  // the field is a rosidl::Buffer on Lyrical
      cv::imencode(".jpg", image, jpeg, {cv::IMWRITE_JPEG_QUALITY, 90});
      compressed.data = jpeg;
      writer.write(compressed, "/camera/image/compressed", rclcpp::Time(stamp));
    }
  }
}

void writeMap(const std::string & yaml)
{
  const std::string pgm = fs::path(yaml).replace_extension(".pgm").string();
  cv::Mat image(200, 200, CV_8UC1, cv::Scalar(254));
  image.row(0).setTo(0);
  cv::imwrite(pgm, image);
  std::ofstream(yaml) << "image: " << fs::path(pgm).filename().string()
                      << "\nmode: trinary\nresolution: 0.05\norigin: [-2.0, -2.0, 0.0]\n"
                         "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
}

const char * kConfig = R"(bag:
  image_topic: /camera/image/compressed
  camera_info_topic: /camera/camera_info
  pose_topic: /amcl_pose
)";

int run(const std::string & command)
{
  const int status = std::system((command + " > /dev/null 2>&1").c_str());
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

}  // namespace

TEST(BagBuild, SelectsKeyframesWithTheRecorderGates)
{
  TempDir dir;
  const std::string bag = dir.file("drive");
  writeBag(bag);
  const auto config = landfall::parseBuildConfig(kConfig);
  const auto result = landfall::ros::readBagKeyframes(bag, config);

  EXPECT_EQ(result.images, 100U);
  EXPECT_EQ(result.counts.frames_seen, 100U);
  EXPECT_EQ(result.camera.frame_id, "camera_optical");
  EXPECT_EQ(result.camera.model.width(), 320);
  EXPECT_EQ(result.session.name, "drive");
  // Without odometry 0.2 s before the first images, they have no pose (speed needs the window).
  EXPECT_GE(result.counts.no_pose, 1U);
  EXPECT_GE(result.counts.pose_uncertain, 1U);  // 8 .. 10 s
  EXPECT_GE(result.counts.speed, 1U);           // 14 .. 16 s
  ASSERT_GE(result.keyframes.size(), 8U);
  ASSERT_EQ(result.keyframes.size(), result.counts.keyframes);

  const double c = std::cos(kMapOdomYaw);
  const double s = std::sin(kMapOdomYaw);
  double last_x = -1.0;
  for (const auto & k : result.keyframes) {
    const double t = static_cast<double>(k.stamp_ns - kStart) * 1e-9;
    const double x = odomX(t);
    EXPECT_NEAR(k.prior.x, 1.0 + c * x, 1e-6);
    EXPECT_NEAR(k.prior.y, 2.0 + s * x, 1e-6);
    EXPECT_NEAR(k.prior.yaw, kMapOdomYaw, 1e-9);
    EXPECT_TRUE(t < 8.0 || t >= 10.0) << "keyframe while localization was uncertain: " << t;
    // The speed is averaged over the 0.2 s before the image.
    EXPECT_TRUE(t < 14.0 || t >= 16.2) << "keyframe while driving fast: " << t;
    EXPECT_GE(x - last_x, 0.5 - 1e-9);
    last_x = x;
    ASSERT_TRUE(k.covariance.has_value());
    EXPECT_NEAR(std::sqrt(k.covariance.value()(0, 0)), 0.02, 1e-9);
    EXPECT_NEAR(k.T_base_cam.translation().x(), 0.2, 1e-12);
    EXPECT_EQ(k.load_image().size(), cv::Size(320, 240));
  }

  auto missing = config;
  missing.bag.image_topic = "/nope";
  EXPECT_THROW(landfall::ros::readBagKeyframes(bag, missing), std::runtime_error);
  EXPECT_THROW(landfall::ros::readBagKeyframes(dir.file("absent"), config), std::runtime_error);
}

TEST(BagBuild, CommandLineBuildVerifyInspect)
{
  TempDir dir;
  const std::string bag = dir.file("drive");
  writeBag(bag);
  const std::string map = dir.file("map.yaml");
  writeMap(map);
  const std::string config = dir.file("build.yaml");
  std::ofstream(config) << kConfig;
  const std::string db = dir.file("out.landfall.db");
  const std::string cli = LANDFALL_CLI;

  EXPECT_EQ(
    run(
      cli + " build --map " + map + " --bag " + bag + " --config " + config + " --output " + db +
      " --threads 2"),
    0);
  ASSERT_TRUE(fs::exists(db));
  EXPECT_TRUE(fs::exists(db + ".report/report.md"));
  EXPECT_TRUE(fs::exists(db + ".report/coverage.png"));
  EXPECT_EQ(run(cli + " verify " + db + " --map " + map), 0);
  EXPECT_EQ(run(cli + " inspect " + db), 0);
  EXPECT_EQ(run(cli + " report " + db + " --output " + dir.file("again")), 0);
  EXPECT_TRUE(fs::exists(dir.file("again/report.json")));
  EXPECT_EQ(run(cli + " migrate " + db), 0);
  // Never overwrites; usage errors exit with 2.
  EXPECT_EQ(
    run(cli + " build --map " + map + " --bag " + bag + " --config " + config + " --output " + db),
    1);
  EXPECT_FALSE(fs::exists(db + ".partial"));
  EXPECT_EQ(run(cli + " build --map " + map + " --output " + dir.file("x.db")), 2);
  EXPECT_EQ(run(cli + " frobnicate"), 2);
  EXPECT_EQ(run(cli + " --version"), 0);

  // A different map fails verification.
  cv::Mat other(200, 200, CV_8UC1, cv::Scalar(254));
  cv::imwrite(dir.file("map.pgm"), other);
  EXPECT_EQ(run(cli + " verify " + db + " --map " + map), 1);
}
