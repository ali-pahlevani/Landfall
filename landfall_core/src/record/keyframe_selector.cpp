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

#include "landfall/record/keyframe_selector.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

namespace landfall
{

namespace
{
double toRad(double degrees) { return degrees * M_PI / 180.0; }
}  // namespace

const char * toString(GateResult result)
{
  switch (result) {
    case GateResult::kAccepted:
      return "accepted";
    case GateResult::kNoPose:
      return "no_pose";
    case GateResult::kNoMotion:
      return "no_motion";
    case GateResult::kPoseUncertain:
      return "pose_uncertain";
    case GateResult::kSpeed:
      return "speed";
    case GateResult::kTilt:
      return "tilt";
    case GateResult::kBlur:
      return "blur";
    case GateResult::kExposure:
      return "exposure";
    case GateResult::kLimitReached:
      return "limit_reached";
  }
  return "unknown";
}

double worstStdXy(const Mat3 & covariance)
{
  const Eigen::Matrix2d xy = covariance.topLeftCorner<2, 2>();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(xy, Eigen::EigenvaluesOnly);
  return std::sqrt(std::max(0.0, solver.eigenvalues().maxCoeff()));
}

KeyframeSelector::KeyframeSelector(const KeyframeGating & gating) : gating_(gating) {}

GateResult KeyframeSelector::checkPose(const FrameCandidate & frame) const
{
  if (gating_.max_keyframes > 0 && static_cast<int>(counts_.keyframes) >= gating_.max_keyframes) {
    return GateResult::kLimitReached;
  }
  if (
    last_keyframe_ &&
    translationDistance(*last_keyframe_, frame.map_base) < gating_.min_translation &&
    rotationDistance(*last_keyframe_, frame.map_base) < toRad(gating_.min_rotation_deg)) {
    return GateResult::kNoMotion;
  }
  if (gating_.require_covariance) {
    if (!frame.covariance || !frame.covariance->allFinite()) {
      return GateResult::kPoseUncertain;
    }
    const Mat3 & cov = *frame.covariance;
    if (
      std::abs(frame.covariance_age) > gating_.max_covariance_age ||
      worstStdXy(cov) > gating_.max_std_xy ||
      std::sqrt(std::max(0.0, cov(2, 2))) > toRad(gating_.max_std_yaw_deg)) {
      return GateResult::kPoseUncertain;
    }
  }
  if (
    std::abs(frame.linear_speed) > gating_.max_linear_speed ||
    std::abs(frame.angular_speed) > gating_.max_angular_speed) {
    return GateResult::kSpeed;
  }
  if (frame.tilt_deg && *frame.tilt_deg > gating_.max_tilt_deg) {
    return GateResult::kTilt;
  }
  return GateResult::kAccepted;
}

GateResult KeyframeSelector::checkImage(const ImageQuality & quality) const
{
  if (quality.sharpness < gating_.min_sharpness) {
    return GateResult::kBlur;
  }
  if (
    quality.mean_intensity < gating_.min_mean_intensity ||
    quality.mean_intensity > gating_.max_mean_intensity ||
    quality.saturated_fraction > gating_.max_saturated_fraction) {
    return GateResult::kExposure;
  }
  return GateResult::kAccepted;
}

GateResult KeyframeSelector::evaluate(
  const FrameCandidate & frame, const std::function<ImageQuality()> & measure)
{
  GateResult result = checkPose(frame);
  if (result == GateResult::kLimitReached) {
    return result;
  }
  if (result == GateResult::kAccepted) {
    result = checkImage(measure());
  }
  count(result);
  if (result == GateResult::kAccepted) {
    last_keyframe_ = frame.map_base;
  }
  return result;
}

void KeyframeSelector::countNoPose() { count(GateResult::kNoPose); }

void KeyframeSelector::count(GateResult result)
{
  ++counts_.frames_seen;
  switch (result) {
    case GateResult::kAccepted:
      ++counts_.keyframes;
      break;
    case GateResult::kNoPose:
      ++counts_.no_pose;
      break;
    case GateResult::kNoMotion:
      ++counts_.no_motion;
      break;
    case GateResult::kPoseUncertain:
      ++counts_.pose_uncertain;
      break;
    case GateResult::kSpeed:
      ++counts_.speed;
      break;
    case GateResult::kTilt:
      ++counts_.tilt;
      break;
    case GateResult::kBlur:
      ++counts_.blur;
      break;
    case GateResult::kExposure:
      ++counts_.exposure;
      break;
    case GateResult::kLimitReached:
      break;
  }
}

}  // namespace landfall
