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

#include "landfall/query/decision.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace landfall
{

Pose2 propagate(const Pose2 & map_T_base_then, const Pose2 & odom_then, const Pose2 & odom_now)
{
  return compose(map_T_base_then, between(odom_then, odom_now));
}

Mat3 propagateCovariance(
  const Mat3 & covariance, const Pose2 & map_T_base_then, const Pose2 & odom_then,
  const Pose2 & odom_now)
{
  // d(then ⊕ motion) / d(then): the motion, turned into the map frame, swings with the heading.
  const Pose2 motion = between(odom_then, odom_now);
  const double c = std::cos(map_T_base_then.yaw);
  const double s = std::sin(map_T_base_then.yaw);
  Mat3 J = Mat3::Identity();
  J(0, 2) = -s * motion.x - c * motion.y;
  J(1, 2) = c * motion.x - s * motion.y;
  // The product's two halves can round apart in the last bit (they did with GCC 13).
  const Mat3 moved = J * covariance * J.transpose();
  return 0.5 * (moved + moved.transpose());
}

Mat3 coverWithoutNegativeEntries(const Mat3 & covariance)
{
  const Mat3 symmetric = 0.5 * (covariance + covariance.transpose());
  Mat3 cover = symmetric;
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      const double correlation = symmetric(i, j);
      if (correlation >= 0.0) {
        continue;
      }
      // Adding [r v_i, |c|; |c|, r v_j] with r = |c| / sqrt(v_i v_j) = |rho| (determinant 0, so
      // positive semidefinite) cancels the correlation c at the least area; the sum over the pairs
      // stays positive semidefinite. Without variances to scale by, |c| on both does it too.
      const double scale = std::sqrt(symmetric(i, i) * symmetric(j, j));
      if (scale > 0.0) {
        const double ratio = -correlation / scale;
        cover(i, i) += ratio * symmetric(i, i);
        cover(j, j) += ratio * symmetric(j, j);
      } else {
        cover(i, i) -= correlation;
        cover(j, j) -= correlation;
      }
      cover(i, j) = 0.0;
      cover(j, i) = 0.0;
    }
  }
  return cover;
}

ConsistencyResult checkConsistency(
  const std::vector<FrameEstimate> & frames, const ConsistencyParams & params)
{
  ConsistencyResult result;
  if (frames.empty()) {
    return result;
  }
  for (std::size_t i = 0; i < frames.size(); ++i) {
    for (std::size_t j = i + 1; j < frames.size(); ++j) {
      const Pose2 predicted =
        propagate(frames[i].map_T_base, frames[i].odom_T_base, frames[j].odom_T_base);
      result.max_translation_error = std::max(
        result.max_translation_error, translationDistance(predicted, frames[j].map_T_base));
      result.max_rotation_error_deg = std::max(
        result.max_rotation_error_deg,
        rotationDistance(predicted, frames[j].map_T_base) * 180.0 / M_PI);
    }
  }
  // Every estimate moved to the last frame.
  const Pose2 & odom_last = frames.back().odom_T_base;
  std::vector<Pose2> poses;
  std::vector<Mat3> covariances;
  for (const auto & frame : frames) {
    poses.push_back(propagate(frame.map_T_base, frame.odom_T_base, odom_last));
    if (frame.covariance) {
      covariances.push_back(
        propagateCovariance(*frame.covariance, frame.map_T_base, frame.odom_T_base, odom_last));
    }
  }
  const auto n = static_cast<double>(frames.size());
  if (covariances.size() == frames.size()) {
    // Covariance intersection with equal weights: the frames share their landmarks and so their
    // errors, by an unknown amount; this fusion stays consistent whatever it is (frames of one
    // view, alike, give one frame's covariance, not a third of it), and weighs each frame by what
    // it fixes, e.g. along a corridor.
    Mat3 information = Mat3::Zero();
    Vec3 weighted = Vec3::Zero();
    const Pose2 & reference = poses.front();
    for (std::size_t i = 0; i < poses.size(); ++i) {
      const Mat3 frame_information = covariances[i].inverse() / n;
      const Vec3 offset(
        poses[i].x - reference.x, poses[i].y - reference.y,
        normalizeAngle(poses[i].yaw - reference.yaw));
      information += frame_information;
      weighted += frame_information * offset;
    }
    const Mat3 covariance = information.inverse();
    const Vec3 offset = covariance * weighted;
    result.fused = {
      reference.x + offset.x(), reference.y + offset.y(),
      normalizeAngle(reference.yaw + offset.z())};
    result.covariance = 0.5 * (covariance + covariance.transpose());
  } else {
    // The mean (circular for the heading).
    double x = 0.0;
    double y = 0.0;
    double s = 0.0;
    double c = 0.0;
    for (const Pose2 & p : poses) {
      x += p.x;
      y += p.y;
      s += std::sin(p.yaw);
      c += std::cos(p.yaw);
    }
    result.fused = {x / n, y / n, std::atan2(s, c)};
  }
  result.consistent = result.max_translation_error <= params.max_translation_error &&
                      result.max_rotation_error_deg <= params.max_rotation_error_deg;
  return result;
}

const char * toString(Status status)
{
  switch (status) {
    case Status::kSuccess:
      return "SUCCESS";
    case Status::kAmbiguous:
      return "AMBIGUOUS";
    case Status::kNoMatch:
      return "NO_MATCH";
    case Status::kRejectedSanity:
      return "REJECTED_SANITY";
    case Status::kRejectedScanCheck:
      return "REJECTED_SCAN_CHECK";
    case Status::kRejectedConsistency:
      return "REJECTED_CONSISTENCY";
    case Status::kNotReady:
      return "NOT_READY";
    case Status::kBusy:
      return "BUSY";
  }
  return "UNKNOWN";
}

Decision decide(const DecisionInput & input, const ConsistencyParams & consistency)
{
  Decision decision;
  if (!input.ready) {
    decision.status = Status::kNotReady;
    decision.message = input.not_ready_reason;
    return decision;
  }
  if (input.busy) {
    decision.status = Status::kBusy;
    decision.message = "a query is already running";
    return decision;
  }
  std::vector<FrameEstimate> valid;
  bool any_failed = false;
  Stage deepest_failure = Stage::kRetrieved;
  for (const auto & frame : input.frames) {
    if (frame.ambiguous) {
      decision.status = Status::kAmbiguous;
      decision.message =
        frame.scan_elsewhere ? "the LiDAR scan fits another place better than the one vision found"
        : frame.scan_twin
          ? "the landmarks come from keyframes whose LiDAR scans fitted two places within 1 m (a "
            "wall the map has twice?): the pose may be that far off"
          : "visually similar places pass every check";
      return decision;
    }
    if (frame.estimate) {
      valid.push_back(*frame.estimate);
    } else {
      any_failed = true;
      deepest_failure = std::max(deepest_failure, frame.deepest_stage);
    }
  }
  decision.frames_used = static_cast<int>(valid.size());
  const int required = std::max(input.required_frames, 1);
  if (decision.frames_used < required) {
    std::ostringstream message;
    message << decision.frames_used << " of " << required << " frames produced a valid pose";
    decision.message = message.str();
    if (!any_failed || deepest_failure == Stage::kRetrieved) {
      decision.status = Status::kNoMatch;
    } else if (deepest_failure == Stage::kSolved) {
      decision.status = Status::kRejectedSanity;
    } else {
      decision.status = Status::kRejectedScanCheck;
    }
    return decision;
  }
  decision.consistency = checkConsistency(valid, consistency);
  if (!decision.consistency.consistent) {
    std::ostringstream message;
    message << "frames disagree with odometry by " << decision.consistency.max_translation_error
            << " m / " << decision.consistency.max_rotation_error_deg << " deg";
    decision.status = Status::kRejectedConsistency;
    decision.message = message.str();
    return decision;
  }
  decision.status = Status::kSuccess;
  decision.pose = decision.consistency.fused;
  decision.covariance = decision.consistency.covariance;
  return decision;
}

}  // namespace landfall
