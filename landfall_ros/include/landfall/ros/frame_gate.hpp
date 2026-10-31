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

#ifndef LANDFALL_ROS_FRAME_GATE_HPP_
#define LANDFALL_ROS_FRAME_GATE_HPP_

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "landfall/record/keyframe_selector.hpp"
#include "landfall/ros/imu_gravity_source.hpp"
#include "landfall/ros/tf_pose_source.hpp"

namespace landfall::ros
{

/// IMU input of the ramp gate.
struct TiltInput
{
  const ImuGravitySource * imu = nullptr;  ///< nullptr: no IMU configured
  Eigen::Quaterniond q_base_imu = Eigen::Quaterniond::Identity();
  GravitySource source = GravitySource::kAuto;
  double max_age = 0.1;  ///< s
};

/// Outcome of the keyframe gates for one image.
struct FrameDecision
{
  GateResult result = GateResult::kNoPose;
  PoseSample pose;
  std::optional<double> tilt_deg;
  ImageQuality quality;
  cv::Mat image;      ///< decoded image, when the image gates ran
  std::string error;  ///< why there is no pose
};

/// Runs the keyframe gates on one image: the pose at its stamp, the IMU tilt (`auto`: from a fresh
/// sample, no tilt gate without one; `imu`: no fresh sample means no keyframe), then the image
/// quality on the image `decode` returns (only called when the pose gates pass). Shared by the
/// recorder and `landfall build --bag`, so both select the same keyframes from the same data.
FrameDecision gateFrame(
  KeyframeSelector & selector, TfPoseSource & poses, const TiltInput & tilt, int64_t stamp_ns,
  const std::string & camera_frame, const std::function<cv::Mat()> & decode);

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_FRAME_GATE_HPP_
