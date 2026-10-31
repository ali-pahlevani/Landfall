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

#ifndef LANDFALL_BUILD_REPORT_HPP_
#define LANDFALL_BUILD_REPORT_HPP_

#include <opencv2/core.hpp>
#include <string>

#include "landfall/db/database.hpp"

namespace landfall
{

/// Numbers a report derives from a database's content.
struct MapSummary
{
  int keyframes = 0;
  int sessions = 0;
  int landmarks = 0;
  int observations = 0;
  double mean_track_length = 0.0;
  double mean_landmarks_per_keyframe = 0.0;
  double keyframes_with_100_landmarks = 0.0;  ///< fraction
  double mean_landmark_reprojection_px = 0.0;
  double ba_mean_shift_m = 0.0;  ///< refined vs prior base positions
  double ba_max_shift_m = 0.0;
  double ba_max_shift_yaw_deg = 0.0;
  int flagged_keyframes = 0;
  /// Mean fraction of the 8 heading sectors seen per 1 m cell visited by keyframes.
  double heading_sector_fraction = 0.0;
  /// Fraction of visited 1 m cells seen in two opposite directions ("drive both ways").
  double both_directions_fraction = 0.0;
};

struct MapReport
{
  MapSummary summary;
  std::string markdown;  ///< report.md
  std::string json;      ///< report.json: {"summary": ..., "build": <meta build_report>}
  cv::Mat coverage;      ///< coverage.png (BGR)
};

/// Builds the report of a map database: summary numbers, the builder's own statistics (meta
/// `build_report`) and a coverage image (keyframes with headings over the occupancy grid, an
/// 8-sector heading rose per 1 m cell showing directions never driven, landmark density).
MapReport makeReport(const db::Database & database);

/// Writes report.md, report.json and coverage.png into `directory` (created if needed).
void writeReport(const MapReport & report, const std::string & directory);

}  // namespace landfall

#endif  // LANDFALL_BUILD_REPORT_HPP_
