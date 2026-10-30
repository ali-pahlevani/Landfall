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

namespace landfall
{

Pose2 propagate(const Pose2 & map_T_base_then, const Pose2 & odom_then, const Pose2 & odom_now)
{
  return compose(map_T_base_then, between(odom_then, odom_now));
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
  // Mean of every estimate moved to the last frame (circular mean for the heading).
  const Pose2 & odom_last = frames.back().odom_T_base;
  double x = 0.0;
  double y = 0.0;
  double s = 0.0;
  double c = 0.0;
  for (const auto & frame : frames) {
    const Pose2 p = propagate(frame.map_T_base, frame.odom_T_base, odom_last);
    x += p.x;
    y += p.y;
    s += std::sin(p.yaw);
    c += std::cos(p.yaw);
  }
  const auto n = static_cast<double>(frames.size());
  result.fused = {x / n, y / n, std::atan2(s, c)};
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
      decision.message = "visually similar places pass every check";
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
  return decision;
}

}  // namespace landfall
