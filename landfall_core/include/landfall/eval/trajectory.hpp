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

#ifndef LANDFALL_EVAL_TRAJECTORY_HPP_
#define LANDFALL_EVAL_TRAJECTORY_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "landfall/types.hpp"

namespace landfall::eval
{

/// A time-stamped planar trajectory (e.g. a simulator's ground truth), interpolated linearly
/// in position and along the shorter arc in yaw.
class Trajectory
{
public:
  void add(int64_t stamp_ns, const Pose2 & pose) { poses_[stamp_ns] = pose; }
  std::size_t size() const { return poses_.size(); }
  /// Pose at the stamp; nullopt outside the trajectory or across a gap longer than `max_gap_ns`.
  std::optional<Pose2> at(int64_t stamp_ns, int64_t max_gap_ns) const;

private:
  std::map<int64_t, Pose2> poses_;
};

/// SE(2) transform A minimizing sum |A (+) truth_i - estimate_i|^2 over positions (2D Umeyama
/// without scale); the heading comes from the same rotation. Needs at least two poses.
Pose2 alignSE2(const std::vector<Pose2> & truth, const std::vector<Pose2> & estimates);

struct ErrorStats
{
  std::size_t count = 0;
  double median = 0.0;
  double p95 = 0.0;
  double max = 0.0;
  double rms = 0.0;
};

ErrorStats errorStats(std::vector<double> values);

/// Position (m) and absolute yaw (rad) errors of estimates against `transform (+) truth`.
void poseErrors(
  const std::vector<Pose2> & truth, const std::vector<Pose2> & estimates, const Pose2 & transform,
  std::vector<double> * position, std::vector<double> * yaw);

}  // namespace landfall::eval

#endif  // LANDFALL_EVAL_TRAJECTORY_HPP_
