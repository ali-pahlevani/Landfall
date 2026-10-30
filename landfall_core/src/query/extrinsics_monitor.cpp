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

#include "landfall/query/extrinsics_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace landfall
{
namespace
{

double median(std::vector<double> values)
{
  const std::size_t mid = values.size() / 2;
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(mid);
  std::nth_element(values.begin(), middle, values.end());
  const double upper = values[mid];
  if (values.size() % 2 == 1) {
    return upper;
  }
  return 0.5 * (upper + *std::max_element(values.begin(), middle));
}

}  // namespace

ExtrinsicsMonitor::ExtrinsicsMonitor(const ExtrinsicsMonitorParams & params) : params_(params) {}

void ExtrinsicsMonitor::addSample(const Pose3 & T_map_cam_visual, const Pose3 & T_map_cam_predicted)
{
  // Where the camera really is, relative to where TF says it is, in the camera frame.
  const Pose3 offset = T_map_cam_predicted.inverse() * T_map_cam_visual;
  const Eigen::AngleAxisd rotation(offset.linear());
  Eigen::Matrix<double, 6, 1> sample;
  sample.head<3>() = offset.translation();
  sample.tail<3>() = rotation.angle() * rotation.axis();
  samples_.push_back(sample);
  while (samples_.size() > std::max<std::size_t>(params_.window, 1)) {
    samples_.pop_front();
  }
}

ExtrinsicsEstimate ExtrinsicsMonitor::estimate() const
{
  ExtrinsicsEstimate estimate;
  estimate.samples = samples_.size();
  if (samples_.empty()) {
    return estimate;
  }
  Eigen::Matrix<double, 6, 1> m;
  for (int k = 0; k < 6; ++k) {
    std::vector<double> values;
    values.reserve(samples_.size());
    for (const auto & s : samples_) {
      values.push_back(s(k));
    }
    m(k) = median(std::move(values));
  }
  estimate.translation = m.head<3>();
  estimate.rotation = m.tail<3>();
  estimate.translation_norm = estimate.translation.norm();
  estimate.rotation_deg = estimate.rotation.norm() * 180.0 / M_PI;
  estimate.level = samples_.size() < params_.min_samples
                     ? HealthLevel::kInsufficientData
                     : gradeOffset(
                         estimate.translation_norm, estimate.rotation_deg, params_.warn_translation,
                         params_.warn_rotation_deg, params_.error_factor);
  return estimate;
}

HealthLevel gradeOffset(
  double translation, double rotation_deg, double warn_translation, double warn_rotation_deg,
  double error_factor)
{
  if (
    translation > error_factor * warn_translation ||
    rotation_deg > error_factor * warn_rotation_deg) {
    return HealthLevel::kError;
  }
  if (translation > warn_translation || rotation_deg > warn_rotation_deg) {
    return HealthLevel::kWarn;
  }
  return HealthLevel::kOk;
}

ExtrinsicsDifference compareExtrinsics(const Pose3 & stored, const Pose3 & current)
{
  const Pose3 difference = stored.inverse() * current;
  return {difference.translation().norm(), rotationAngle(difference.linear()) * 180.0 / M_PI};
}

}  // namespace landfall
