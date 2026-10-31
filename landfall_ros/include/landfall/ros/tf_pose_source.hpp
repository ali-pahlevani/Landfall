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

#ifndef LANDFALL_ROS_TF_POSE_SOURCE_HPP_
#define LANDFALL_ROS_TF_POSE_SOURCE_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "landfall/interfaces/pose_source.hpp"
#include "landfall/record/staging.hpp"
#include "tf2/buffer_core.hpp"

namespace landfall::ros
{

/// Keyframe poses from TF: global->base and base->camera at the image stamp, speed from the
/// odom->base transform over a short window before it, and the covariance of the latest
/// localization estimate (e.g. amcl_pose). Works on a live tf2_ros::Buffer and on a BufferCore
/// filled from a bag, so the recorder and `landfall build --bag` compute identical poses.
class TfPoseSource : public IPoseSource
{
public:
  /// `speed_window` in seconds; `history` bounds the stored covariance estimates.
  TfPoseSource(
    std::shared_ptr<const tf2::BufferCore> buffer, StagingFrames frames, double speed_window,
    std::size_t history = 10000);

  /// Adds the covariance (x, y, yaw) of a localization estimate stamped `stamp_ns`.
  void addCovariance(int64_t stamp_ns, const Mat3 & covariance);
  /// Removes the stored estimates.
  void clearCovariances() { covariances_.clear(); }

  std::optional<PoseSample> poseAt(
    int64_t stamp_ns, const std::string & camera_frame, std::string * error) override;

  /// True when every transform poseAt() needs is available at the stamp (live recording waits
  /// for late TF before deciding).
  bool canTransform(int64_t stamp_ns, const std::string & camera_frame) const;

  const StagingFrames & frames() const { return frames_; }

private:
  std::shared_ptr<const tf2::BufferCore> buffer_;
  StagingFrames frames_;
  int64_t speed_window_ns_;
  std::size_t history_;
  std::map<int64_t, Mat3> covariances_;
};

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_TF_POSE_SOURCE_HPP_
