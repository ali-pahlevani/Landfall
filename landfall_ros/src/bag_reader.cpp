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

#include "landfall/ros/bag_reader.hpp"

#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/ros/conversions.hpp"
#include "landfall/ros/frame_gate.hpp"
#include "landfall/ros/imu_gravity_source.hpp"
#include "landfall/ros/tf_pose_source.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace landfall::ros
{

namespace
{

constexpr const char * kTfTopic = "/tf";
constexpr const char * kTfStaticTopic = "/tf_static";
constexpr const char * kImageType = "sensor_msgs/msg/Image";
constexpr const char * kCompressedType = "sensor_msgs/msg/CompressedImage";

template <typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  static const rclcpp::Serialization<MessageT> serialization;
  const rclcpp::SerializedMessage serialized(*bag_message.serialized_data);
  MessageT message;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

rosbag2_storage::StorageFilter topicsFilter(std::vector<std::string> topics)
{
  rosbag2_storage::StorageFilter filter;
  filter.topics = std::move(topics);
  return filter;
}

}  // namespace

BagKeyframes readBagKeyframes(
  const std::string & uri, const BuildConfigFile & config, const BagProgress & progress)
{
  const BagConfig & bag = config.bag;
  if (!std::filesystem::exists(uri)) {
    throw std::runtime_error("bag not found: " + uri);
  }
  const auto metadata = rosbag2_storage::MetadataIo().read_metadata(uri);
  std::string image_type;
  std::size_t image_count = 0;
  std::string available;
  for (const auto & info : metadata.topics_with_message_count) {
    const auto & topic = info.topic_metadata;
    if (topic.type == kImageType || topic.type == kCompressedType) {
      available += " " + topic.name;
    }
    if (topic.name == bag.image_topic) {
      image_type = topic.type;
      image_count = info.message_count;
    }
  }
  if (image_type.empty()) {
    throw std::runtime_error(
      "bag " + uri + " has no topic '" + bag.image_topic +
      "'; image topics:" + (available.empty() ? " none" : available));
  }
  if (image_type != kImageType && image_type != kCompressedType) {
    throw std::runtime_error(
      "'" + bag.image_topic + "' is a " + image_type + ", not an Image or CompressedImage");
  }

  // Pass 1: everything except images.
  const auto duration = std::chrono::duration_cast<tf2::Duration>(metadata.duration);
  auto buffer = std::make_shared<tf2::BufferCore>(duration + std::chrono::seconds(60));
  TfPoseSource poses(buffer, bag.frames, bag.speed_window, std::numeric_limits<std::size_t>::max());
  ImuGravitySource imu(std::numeric_limits<std::size_t>::max());
  sensor_msgs::msg::CameraInfo info;  // the first one
  bool have_camera_info = false;
  std::size_t invalid_transforms = 0;
  std::string imu_frame;
  std::vector<std::string> topics = {kTfTopic, kTfStaticTopic, bag.camera_info_topic};
  if (!bag.pose_topic.empty() && bag.pose_source == "tf_with_covariance") {
    topics.push_back(bag.pose_topic);
  }
  if (!bag.imu_topic.empty()) {
    topics.push_back(bag.imu_topic);
  }
  {
    rosbag2_cpp::Reader reader;
    reader.open(uri);
    reader.set_filter(topicsFilter(topics));
    while (reader.has_next()) {
      const auto message = reader.read_next();
      const std::string & topic = message->topic_name;
      if (topic == kTfTopic || topic == kTfStaticTopic) {
        const auto tf = deserialize<tf2_msgs::msg::TFMessage>(*message);
        for (const auto & transform : tf.transforms) {
          // Invalid transforms (e.g. empty frame ids) are skipped, as tf2_ros does.
          try {
            buffer->setTransform(transform, "bag", topic == kTfStaticTopic);
          } catch (const tf2::TransformException &) {
            ++invalid_transforms;
          }
        }
      } else if (topic == bag.camera_info_topic) {
        if (!have_camera_info) {
          info = deserialize<sensor_msgs::msg::CameraInfo>(*message);
          have_camera_info = true;
        }
      } else if (topic == bag.pose_topic) {
        const auto pose = deserialize<geometry_msgs::msg::PoseWithCovarianceStamped>(*message);
        poses.addCovariance(
          toNanoseconds(pose.header.stamp), planarCovariance(pose.pose.covariance));
      } else if (topic == bag.imu_topic) {
        const auto sample = deserialize<sensor_msgs::msg::Imu>(*message);
        if (imu.add(sample) && imu_frame.empty()) {
          imu_frame = bag.imu_frame.empty() ? sample.header.frame_id : bag.imu_frame;
        }
      }
    }
  }
  if (!have_camera_info) {
    throw std::runtime_error(
      "bag " + uri + " has no camera_info on '" + bag.camera_info_topic + "'");
  }
  BagKeyframes result;
  result.invalid_transforms = invalid_transforms;
  const std::string camera_frame =
    bag.frames.camera.empty() ? info.header.frame_id : bag.frames.camera;
  try {
    result.camera.model = toCameraModel(info);
  } catch (const std::invalid_argument & e) {
    throw std::runtime_error("bag " + uri + ": " + e.what());
  }
  result.camera.frame_id = camera_frame;

  TiltInput tilt;
  tilt.source = parseGravitySource(bag.gravity_source).value_or(GravitySource::kAuto);
  tilt.max_age = bag.imu_max_age;
  if (!bag.imu_topic.empty() && tilt.source != GravitySource::kFlatFloor) {
    if (imu.empty()) {
      if (tilt.source == GravitySource::kImu) {
        throw std::runtime_error(
          "gravity source imu, but '" + bag.imu_topic + "' has no orientation samples");
      }
    } else {
      try {
        tilt.q_base_imu = Eigen::Quaterniond(
          toPose3(buffer->lookupTransform(bag.frames.base, imu_frame, tf2::TimePointZero).transform)
            .linear());
        tilt.imu = &imu;
      } catch (const tf2::TransformException & e) {
        throw std::runtime_error(
          "no transform " + bag.frames.base + " -> " + imu_frame + " in the bag: " + e.what());
      }
    }
  }

  // Pass 2: the images, through the recorder's gates.
  KeyframeSelector selector(config.gating);
  const auto offset_ns = static_cast<int64_t>(bag.image_time_offset * 1e9);
  rosbag2_cpp::Reader reader;
  reader.open(uri);
  reader.set_filter(topicsFilter({bag.image_topic}));
  const bool compressed = image_type == kCompressedType;
  while (reader.has_next()) {
    const auto message = reader.read_next();
    std::function<cv::Mat()> decode;
    int64_t stamp_ns = 0;
    if (compressed) {
      auto image = std::make_shared<sensor_msgs::msg::CompressedImage>(
        deserialize<sensor_msgs::msg::CompressedImage>(*message));
      stamp_ns = toNanoseconds(image->header.stamp) + offset_ns;
      decode = [image] { return decodeImage(*image); };
    } else {
      auto image =
        std::make_shared<sensor_msgs::msg::Image>(deserialize<sensor_msgs::msg::Image>(*message));
      stamp_ns = toNanoseconds(image->header.stamp) + offset_ns;
      decode = [image] { return decodeImage(*image); };
    }
    const FrameDecision decision = gateFrame(selector, poses, tilt, stamp_ns, camera_frame, decode);
    ++result.images;
    if (progress && (result.images % 50 == 0 || result.images == image_count)) {
      progress(result.images, image_count);
    }
    if (decision.result == GateResult::kLimitReached) {
      break;
    }
    if (decision.result != GateResult::kAccepted) {
      continue;
    }
    BuildKeyframe keyframe;
    keyframe.stamp_ns = stamp_ns;
    keyframe.prior = decision.pose.map_base;
    keyframe.covariance = decision.pose.covariance;
    keyframe.T_base_cam = decision.pose.T_base_cam;
    keyframe.quality_json = toJson(decision.quality);
    keyframe.load_image = [image = decision.image] { return image; };
    result.keyframes.push_back(std::move(keyframe));
  }
  result.counts = selector.counts();
  result.session.name = std::filesystem::path(uri).filename().string();
  result.session.source = uri;
  result.session.pose_source = bag.pose_source;
  result.session.started_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(metadata.starting_time.time_since_epoch())
      .count();
  if (!result.keyframes.empty()) {
    result.camera.T_base_cam = result.keyframes.front().T_base_cam;
  }
  return result;
}

}  // namespace landfall::ros
