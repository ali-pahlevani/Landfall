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

#ifndef LANDFALL_QUERY_EXTRINSICS_MONITOR_HPP_
#define LANDFALL_QUERY_EXTRINSICS_MONITOR_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>

#include "landfall/types.hpp"

namespace landfall
{

enum class HealthLevel : uint8_t {
  kInsufficientData,
  kOk,
  kWarn,
  kError,
};

struct ExtrinsicsMonitorParams
{
  std::size_t window = 50;  ///< most recent samples kept
  std::size_t min_samples = 10;
  double warn_translation = 0.02;  ///< m
  double warn_rotation_deg = 1.0;
  double error_factor = 2.5;  ///< ERROR at this multiple of the warning thresholds
};

struct ExtrinsicsEstimate
{
  std::size_t samples = 0;
  Vec3 translation = Vec3::Zero();  ///< median camera offset, camera frame, m
  Vec3 rotation = Vec3::Zero();     ///< median rotation vector, camera frame, rad
  double translation_norm = 0.0;
  double rotation_deg = 0.0;
  HealthLevel level = HealthLevel::kInsufficientData;
};

/// Detects a camera that moved on its mount (plan §7.19); never corrects anything. Each sample
/// compares the camera pose from vision with the one localization + TF predict; the per-axis
/// median over a sliding window rejects the samples where localization itself was off.
class ExtrinsicsMonitor
{
public:
  explicit ExtrinsicsMonitor(const ExtrinsicsMonitorParams & params = ExtrinsicsMonitorParams());

  void addSample(const Pose3 & T_map_cam_visual, const Pose3 & T_map_cam_predicted);
  ExtrinsicsEstimate estimate() const;
  void reset() { samples_.clear(); }

private:
  ExtrinsicsMonitorParams params_;
  std::deque<Eigen::Matrix<double, 6, 1>> samples_;
};

/// Grades a translation/rotation offset against warning thresholds and the error factor.
HealthLevel gradeOffset(
  double translation, double rotation_deg, double warn_translation, double warn_rotation_deg,
  double error_factor);

/// Startup check: extrinsics stored in the database vs the current TF.
struct ExtrinsicsDifference
{
  double translation = 0.0;
  double rotation_deg = 0.0;
};
ExtrinsicsDifference compareExtrinsics(const Pose3 & stored, const Pose3 & current);

}  // namespace landfall

#endif  // LANDFALL_QUERY_EXTRINSICS_MONITOR_HPP_
