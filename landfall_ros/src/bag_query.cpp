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

#include "landfall/ros/bag_query.hpp"

#include <chrono>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include "landfall/ros/bag_reader.hpp"
#include "landfall/ros/conversions.hpp"
#include "landfall/ros/scan_input.hpp"
#include "tf2/exceptions.hpp"

namespace landfall::ros
{

BagQueryCounts queryBagImages(
  const std::string & uri, Localizer & localizer, const BagQueryOptions & options,
  const std::function<void(int64_t, const Pose2 &, FrameResult &&)> & visit)
{
  const BagConfig & bag = options.bag;
  BagQueryCounts counts;
  counts.images = countBagImages(uri, bag.image_topic);
  const bool use_scans = options.scan_mode != ScanCheckMode::kDisabled;
  const BagSideData side = readBagSideData(
    uri, bag.camera_info_topic, "", bag.imu_topic, bag.imu_frame, use_scans ? bag.scan_topic : "");
  if (options.scan_mode == ScanCheckMode::kRequired && side.scans.empty()) {
    throw std::runtime_error(
      "bag " + uri + " has no LaserScan on '" + bag.scan_topic +
      "' and the scan check is required");
  }
  if (!side.camera_info) {
    throw std::runtime_error(
      "bag " + uri + " has no camera_info on '" + bag.camera_info_topic + "'");
  }
  const CameraModel camera = toCameraModel(*side.camera_info);
  const CameraModel & built = localizer.map().camera();
  if (camera.width() != built.width() || camera.height() != built.height()) {
    throw std::runtime_error("the bag's camera resolution differs from the database's");
  }
  const std::string camera_frame =
    bag.frames.camera.empty() ? side.camera_info->header.frame_id : bag.frames.camera;
  const GravitySource gravity_source =
    parseGravitySource(bag.gravity_source).value_or(GravitySource::kAuto);
  std::optional<Eigen::Quaterniond> q_base_imu;
  if (!side.imu.empty() && gravity_source != GravitySource::kFlatFloor) {
    try {
      q_base_imu = Eigen::Quaterniond(
        toPose3(side.buffer->lookupTransform(bag.frames.base, side.imu_frame, tf2::TimePointZero)
                  .transform)
          .linear());
    } catch (const tf2::TransformException & e) {
      throw std::runtime_error(
        "no transform " + bag.frames.base + " -> " + side.imu_frame + " in the bag: " + e.what());
    }
  }

  const auto interval_ns = static_cast<int64_t>(options.min_frame_interval * 1e9);
  const std::size_t every = std::max<std::size_t>(options.every, 1);
  std::size_t index = 0;
  int64_t last_used = std::numeric_limits<int64_t>::min();
  forEachBagImage(
    uri, bag.image_topic, bag.image_time_offset,
    [&](int64_t stamp, const std::function<cv::Mat()> & decode) {
      if (index++ % every != 0) {
        return true;
      }
      if (options.skip_stamps.count(stamp) > 0) {
        ++counts.skipped_stamps;
        return true;
      }
      if (last_used != std::numeric_limits<int64_t>::min() && stamp < last_used + interval_ns) {
        return true;
      }
      QueryFrame frame;
      frame.stamp_ns = stamp;
      frame.camera = camera;
      Pose2 odom_T_base;
      const tf2::TimePoint at{std::chrono::nanoseconds(stamp)};
      try {
        frame.T_base_cam =
          toPose3(side.buffer->lookupTransform(bag.frames.base, camera_frame, at).transform);
        odom_T_base = toPose2(
          toPose3(side.buffer->lookupTransform(bag.frames.odom, bag.frames.base, at).transform));
      } catch (const tf2::TransformException &) {
        ++counts.skipped_tf;
        return true;
      }
      if (q_base_imu) {
        const auto gravity = side.imu.at(stamp, *q_base_imu, gravity_source, options.gravity);
        if (!gravity) {
          ++counts.skipped_gravity;
          return true;
        }
        frame.gravity = *gravity;
      } else if (gravity_source == GravitySource::kImu) {
        ++counts.skipped_gravity;
        return true;
      }
      if (use_scans) {
        const auto max_offset = static_cast<int64_t>(options.scan_max_time_offset * 1e9);
        if (const auto * scan = nearestScan(side.scans, stamp, max_offset)) {
          frame.scan =
            scanAtImage(*scan, *side.buffer, bag.frames.odom, bag.frames.base, stamp, nullptr);
        }
        if (!frame.scan && options.scan_mode == ScanCheckMode::kRequired) {
          ++counts.skipped_scan;
          return true;
        }
      }
      frame.image = decode();
      FrameResult result = localizer.localize(frame);
      last_used = stamp;
      ++counts.localized;
      visit(stamp, odom_T_base, std::move(result));
      return true;
    });
  return counts;
}

}  // namespace landfall::ros
