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

#include "landfall/ros/frame_gate.hpp"

#include <limits>

namespace landfall::ros
{

FrameDecision gateFrame(
  KeyframeSelector & selector, TfPoseSource & poses, const TiltInput & tilt, int64_t stamp_ns,
  const std::string & camera_frame, const std::function<cv::Mat()> & decode)
{
  FrameDecision decision;
  const auto pose = poses.poseAt(stamp_ns, camera_frame, &decision.error);
  if (!pose) {
    selector.countNoPose();
    decision.result = GateResult::kNoPose;
    return decision;
  }
  decision.pose = *pose;
  FrameCandidate candidate;
  candidate.map_base = pose->map_base;
  candidate.covariance = pose->covariance;
  candidate.covariance_age = pose->covariance_age;
  candidate.linear_speed = pose->linear_speed;
  candidate.angular_speed = pose->angular_speed;
  if (tilt.imu != nullptr && tilt.source != GravitySource::kFlatFloor) {
    // The ramp gate only needs the floor's slope; turning doesn't change it, so the stillness
    // requirement of the query-time gravity is not applied here.
    GravityParams params;
    params.max_age = tilt.max_age;
    params.max_angular_rate = std::numeric_limits<double>::infinity();
    const auto gravity = tilt.imu->at(stamp_ns, tilt.q_base_imu, tilt.source, params);
    if (!gravity) {
      candidate.tilt_deg = std::numeric_limits<double>::infinity();  // IMU required, none fresh
    } else if (gravity->from_imu) {
      candidate.tilt_deg = gravity->tilt_deg;
    }
  }
  decision.tilt_deg = candidate.tilt_deg;
  decision.result = selector.evaluate(candidate, [&] {
    decision.image = decode();
    decision.quality = measureImageQuality(toGray(decision.image));
    return decision.quality;
  });
  return decision;
}

}  // namespace landfall::ros
