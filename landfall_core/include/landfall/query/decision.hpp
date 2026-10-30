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

#ifndef LANDFALL_QUERY_DECISION_HPP_
#define LANDFALL_QUERY_DECISION_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "landfall/query/consistency.hpp"

namespace landfall
{

/// Query outcome. Values equal landfall_msgs/RelocalizationResult's constants.
enum class Status : uint8_t {
  kSuccess = 0,
  kAmbiguous = 1,
  kNoMatch = 2,
  kRejectedSanity = 3,
  kRejectedScanCheck = 4,
  kRejectedConsistency = 5,
  kNotReady = 6,
  kBusy = 7,
};

/// How far a hypothesis got. Values equal landfall_msgs/Hypothesis's STAGE_* constants.
enum class Stage : uint8_t {
  kRetrieved = 0,
  kSolved = 1,
  kSanityPassed = 2,
  kScanPassed = 3,
};

const char * toString(Status status);

/// What one query frame produced.
struct FrameOutcome
{
  Stage deepest_stage = Stage::kRetrieved;  ///< deepest stage any of its hypotheses reached
  bool ambiguous = false;
  std::optional<FrameEstimate> estimate;  ///< set when the frame has an accepted hypothesis
};

struct DecisionInput
{
  bool ready = true;
  std::string not_ready_reason;
  bool busy = false;
  std::vector<FrameOutcome> frames;
  int required_frames = 3;
};

struct Decision
{
  Status status = Status::kNoMatch;
  std::string message;
  std::optional<Pose2> pose;  ///< map_T_base at the last valid frame; SUCCESS only
  ConsistencyResult consistency;
  int frames_used = 0;
};

/// The policy of plan §7.13. Every branch errs toward a miss:
/// NOT_READY / BUSY first; any ambiguous frame is AMBIGUOUS; too few valid frames report the
/// deepest stage the failing frames reached (NO_MATCH < REJECTED_SANITY < REJECTED_SCAN_CHECK);
/// valid frames that disagree with odometry are REJECTED_CONSISTENCY; otherwise SUCCESS.
Decision decide(const DecisionInput & input, const ConsistencyParams & consistency);

}  // namespace landfall

#endif  // LANDFALL_QUERY_DECISION_HPP_
