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

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <utility>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "landfall/ros/conversions.hpp"
#include "landfall/ros/frame_gate.hpp"
#include "landfall/ros/imu_gravity_source.hpp"
#include "landfall/ros/scan_input.hpp"
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

BagSideData readBagSideData(
  const std::string & uri, const std::string & camera_info_topic, const std::string & pose_topic,
  const std::string & imu_topic, const std::string & imu_frame, const std::string & scan_topic)
{
  if (!std::filesystem::exists(uri)) {
    throw std::runtime_error("bag not found: " + uri);
  }
  const auto metadata = rosbag2_storage::MetadataIo().read_metadata(uri);
  const auto duration = std::chrono::duration_cast<tf2::Duration>(metadata.duration);
  BagSideData data;
  data.buffer = std::make_shared<tf2::BufferCore>(duration + std::chrono::seconds(60));
  std::vector<std::string> topics = {kTfTopic, kTfStaticTopic};
  for (const auto & topic : {camera_info_topic, pose_topic, imu_topic, scan_topic}) {
    if (!topic.empty()) {
      topics.push_back(topic);
    }
  }
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
          data.buffer->setTransform(transform, "bag", topic == kTfStaticTopic);
        } catch (const tf2::TransformException &) {
          ++data.invalid_transforms;
        }
      }
    } else if (topic == camera_info_topic) {
      if (!data.camera_info) {
        data.camera_info = deserialize<sensor_msgs::msg::CameraInfo>(*message);
      }
    } else if (topic == pose_topic) {
      const auto pose = deserialize<geometry_msgs::msg::PoseWithCovarianceStamped>(*message);
      data.covariances.emplace_back(
        toNanoseconds(pose.header.stamp), planarCovariance(pose.pose.covariance));
    } else if (topic == imu_topic) {
      const auto sample = deserialize<sensor_msgs::msg::Imu>(*message);
      if (data.imu.add(sample) && data.imu_frame.empty()) {
        data.imu_frame = imu_frame.empty() ? sample.header.frame_id : imu_frame;
      }
    } else if (topic == scan_topic) {
      data.scans.push_back(deserialize<sensor_msgs::msg::LaserScan>(*message));
    }
  }
  std::stable_sort(data.scans.begin(), data.scans.end(), [](const auto & a, const auto & b) {
    return toNanoseconds(a.header.stamp) < toNanoseconds(b.header.stamp);
  });
  return data;
}

std::size_t countBagImages(const std::string & uri, const std::string & image_topic)
{
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
    if (topic.name == image_topic) {
      image_type = topic.type;
      image_count = info.message_count;
    }
  }
  if (image_type.empty()) {
    throw std::runtime_error(
      "bag " + uri + " has no topic '" + image_topic +
      "'; image topics:" + (available.empty() ? " none" : available));
  }
  if (image_type != kImageType && image_type != kCompressedType) {
    throw std::runtime_error(
      "'" + image_topic + "' is a " + image_type + ", not an Image or CompressedImage");
  }
  return image_count;
}

void forEachBagImage(
  const std::string & uri, const std::string & image_topic, double time_offset,
  const std::function<bool(int64_t, std::function<cv::Mat()>)> & visit)
{
  const auto offset_ns = static_cast<int64_t>(time_offset * 1e9);
  rosbag2_cpp::Reader reader;
  reader.open(uri);
  std::string type;
  for (const auto & topic : reader.get_all_topics_and_types()) {
    if (topic.name == image_topic) {
      type = topic.type;
    }
  }
  reader.set_filter(topicsFilter({image_topic}));
  while (reader.has_next()) {
    const auto message = reader.read_next();
    std::function<cv::Mat()> decode;
    int64_t stamp_ns = 0;
    if (type == kCompressedType) {
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
    if (!visit(stamp_ns, std::move(decode))) {
      return;
    }
  }
}

BagKeyframes readBagKeyframes(
  const std::string & uri, const BuildConfigFile & config, const BagProgress & progress)
{
  const BagConfig & bag = config.bag;
  const std::size_t image_count = countBagImages(uri, bag.image_topic);

  // Pass 1: everything except images.
  // Scans only when the build matches keyframes to the map with them.
  BagSideData side = readBagSideData(
    uri, bag.camera_info_topic, bag.pose_source == "tf_with_covariance" ? bag.pose_topic : "",
    bag.imu_topic, bag.imu_frame,
    config.builder.poses.scan_matching ? bag.scan_topic : std::string());
  TfPoseSource poses(
    side.buffer, bag.frames, bag.speed_window, std::numeric_limits<std::size_t>::max());
  for (const auto & [stamp, covariance] : side.covariances) {
    poses.addCovariance(stamp, covariance);
  }
  if (!side.camera_info) {
    throw std::runtime_error(
      "bag " + uri + " has no camera_info on '" + bag.camera_info_topic + "'");
  }
  const sensor_msgs::msg::CameraInfo & info = *side.camera_info;
  BagKeyframes result;
  result.invalid_transforms = side.invalid_transforms;
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
    if (side.imu.empty()) {
      if (tilt.source == GravitySource::kImu) {
        throw std::runtime_error(
          "gravity source imu, but '" + bag.imu_topic + "' has no orientation samples");
      }
    } else {
      try {
        tilt.q_base_imu = Eigen::Quaterniond(
          toPose3(side.buffer->lookupTransform(bag.frames.base, side.imu_frame, tf2::TimePointZero)
                    .transform)
            .linear());
        tilt.imu = &side.imu;
      } catch (const tf2::TransformException & e) {
        throw std::runtime_error(
          "no transform " + bag.frames.base + " -> " + side.imu_frame + " in the bag: " + e.what());
      }
    }
  }

  // Pass 2: the images, through the recorder's gates.
  KeyframeSelector selector(config.gating);
  forEachBagImage(
    uri, bag.image_topic, bag.image_time_offset,
    [&](int64_t stamp_ns, const std::function<cv::Mat()> & decode) {
      const FrameDecision decision =
        gateFrame(selector, poses, tilt, stamp_ns, camera_frame, decode);
      ++result.images;
      if (progress && (result.images % 50 == 0 || result.images == image_count)) {
        progress(result.images, image_count);
      }
      if (decision.result == GateResult::kLimitReached) {
        return false;
      }
      if (decision.result != GateResult::kAccepted) {
        return true;
      }
      BuildKeyframe keyframe;
      keyframe.stamp_ns = stamp_ns;
      keyframe.prior = decision.pose.map_base;
      keyframe.covariance = decision.pose.covariance;
      keyframe.T_base_cam = decision.pose.T_base_cam;
      keyframe.quality_json = toJson(decision.quality);
      keyframe.odom = decision.pose.odom_base;
      if (
        const auto * nearest =
          nearestScan(side.scans, stamp_ns, static_cast<int64_t>(bag.scan_max_time_offset * 1e9))) {
        std::string error;
        if (
          auto scan = scanAtImage(
            *nearest, *side.buffer, bag.frames.odom, bag.frames.base, stamp_ns, &error)) {
          keyframe.scan = std::move(*scan);
          ++result.with_scan;
        }
      }
      keyframe.load_image = [image = decision.image] { return image; };
      result.keyframes.push_back(std::move(keyframe));
      return true;
    });
  result.counts = selector.counts();
  const auto metadata = rosbag2_storage::MetadataIo().read_metadata(uri);
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
