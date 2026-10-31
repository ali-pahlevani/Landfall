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

#include "landfall/ros/tf_pose_source.hpp"

#include <chrono>
#include <cmath>
#include <utility>

#include "landfall/ros/conversions.hpp"
#include "tf2/exceptions.hpp"

namespace landfall::ros
{

namespace
{

tf2::TimePoint timePoint(int64_t stamp_ns)
{
  return tf2::TimePoint(std::chrono::nanoseconds(stamp_ns));
}

}  // namespace

TfPoseSource::TfPoseSource(
  std::shared_ptr<const tf2::BufferCore> buffer, StagingFrames frames, double speed_window,
  std::size_t history)
: buffer_(std::move(buffer)),
  frames_(std::move(frames)),
  speed_window_ns_(static_cast<int64_t>(speed_window * 1e9)),
  history_(history)
{
}

void TfPoseSource::addCovariance(int64_t stamp_ns, const Mat3 & covariance)
{
  covariances_[stamp_ns] = covariance;
  while (covariances_.size() > history_) {
    covariances_.erase(covariances_.begin());
  }
}

bool TfPoseSource::canTransform(int64_t stamp_ns, const std::string & camera_frame) const
{
  const auto t = timePoint(stamp_ns);
  const auto before = timePoint(stamp_ns - speed_window_ns_);
  return buffer_->canTransform(frames_.global, frames_.base, t, nullptr) &&
         buffer_->canTransform(frames_.base, camera_frame, t, nullptr) &&
         buffer_->canTransform(frames_.odom, frames_.base, t, nullptr) &&
         buffer_->canTransform(frames_.odom, frames_.base, before, nullptr);
}

std::optional<PoseSample> TfPoseSource::poseAt(
  int64_t stamp_ns, const std::string & camera_frame, std::string * error)
{
  PoseSample sample;
  try {
    const auto t = timePoint(stamp_ns);
    sample.map_base =
      toPose2(toPose3(buffer_->lookupTransform(frames_.global, frames_.base, t).transform));
    sample.T_base_cam = toPose3(buffer_->lookupTransform(frames_.base, camera_frame, t).transform);
    const Pose2 now =
      toPose2(toPose3(buffer_->lookupTransform(frames_.odom, frames_.base, t).transform));
    const Pose2 earlier = toPose2(toPose3(
      buffer_->lookupTransform(frames_.odom, frames_.base, timePoint(stamp_ns - speed_window_ns_))
        .transform));
    const double window = static_cast<double>(speed_window_ns_) * 1e-9;
    sample.linear_speed = translationDistance(earlier, now) / window;
    sample.angular_speed = rotationDistance(earlier, now) / window;
  } catch (const tf2::TransformException & e) {
    if (error != nullptr) {
      *error = e.what();
    }
    return std::nullopt;
  }
  if (!covariances_.empty()) {
    // The latest estimate at or before the image; the first one after it otherwise.
    auto it = covariances_.upper_bound(stamp_ns);
    if (it != covariances_.begin()) {
      --it;
    }
    sample.covariance = it->second;
    sample.covariance_age = static_cast<double>(stamp_ns - it->first) * 1e-9;
  }
  return sample;
}

}  // namespace landfall::ros
