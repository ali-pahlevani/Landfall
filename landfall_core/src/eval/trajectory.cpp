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

#include "landfall/eval/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace landfall::eval
{

std::optional<Pose2> Trajectory::at(int64_t stamp_ns, int64_t max_gap_ns) const
{
  const auto after = poses_.lower_bound(stamp_ns);
  if (after == poses_.end()) {
    return std::nullopt;
  }
  if (after->first == stamp_ns) {
    return after->second;
  }
  if (after == poses_.begin()) {
    return std::nullopt;
  }
  const auto before = std::prev(after);
  const int64_t gap = after->first - before->first;
  if (gap > max_gap_ns) {
    return std::nullopt;
  }
  const double s = static_cast<double>(stamp_ns - before->first) / static_cast<double>(gap);
  const Pose2 & a = before->second;
  const Pose2 & b = after->second;
  return Pose2{
    a.x + s * (b.x - a.x), a.y + s * (b.y - a.y),
    normalizeAngle(a.yaw + s * normalizeAngle(b.yaw - a.yaw))};
}

Pose2 alignSE2(const std::vector<Pose2> & truth, const std::vector<Pose2> & estimates)
{
  if (truth.size() != estimates.size() || truth.size() < 2) {
    throw std::invalid_argument("alignSE2: needs two or more matching poses");
  }
  Vec2 mean_truth = Vec2::Zero();
  Vec2 mean_estimate = Vec2::Zero();
  for (std::size_t i = 0; i < truth.size(); ++i) {
    mean_truth += Vec2(truth[i].x, truth[i].y);
    mean_estimate += Vec2(estimates[i].x, estimates[i].y);
  }
  mean_truth /= static_cast<double>(truth.size());
  mean_estimate /= static_cast<double>(truth.size());
  double sxx = 0.0;
  double sxy = 0.0;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    const Vec2 g = Vec2(truth[i].x, truth[i].y) - mean_truth;
    const Vec2 e = Vec2(estimates[i].x, estimates[i].y) - mean_estimate;
    sxx += g.dot(e);
    sxy += g.x() * e.y() - g.y() * e.x();
  }
  const double theta = std::atan2(sxy, sxx);
  const Eigen::Rotation2Dd rotation(theta);
  const Vec2 t = mean_estimate - rotation * mean_truth;
  return {t.x(), t.y(), theta};
}

ErrorStats errorStats(std::vector<double> values)
{
  ErrorStats stats;
  stats.count = values.size();
  if (values.empty()) {
    return stats;
  }
  std::sort(values.begin(), values.end());
  const auto last = static_cast<double>(values.size() - 1);
  const auto quantile = [&](double q) {
    return values[static_cast<std::size_t>(std::lround(q * last))];
  };
  stats.median = quantile(0.5);
  stats.p95 = quantile(0.95);
  stats.max = values.back();
  double squares = 0.0;
  for (double v : values) {
    squares += v * v;
  }
  stats.rms = std::sqrt(squares / static_cast<double>(values.size()));
  return stats;
}

void poseErrors(
  const std::vector<Pose2> & truth, const std::vector<Pose2> & estimates, const Pose2 & transform,
  std::vector<double> * position, std::vector<double> * yaw)
{
  position->clear();
  yaw->clear();
  for (std::size_t i = 0; i < truth.size() && i < estimates.size(); ++i) {
    const Pose2 expected = compose(transform, truth[i]);
    position->push_back(translationDistance(expected, estimates[i]));
    yaw->push_back(rotationDistance(expected, estimates[i]));
  }
}

}  // namespace landfall::eval
