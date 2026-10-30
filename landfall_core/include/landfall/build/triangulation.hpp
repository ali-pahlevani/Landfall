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

#ifndef LANDFALL_BUILD_TRIANGULATION_HPP_
#define LANDFALL_BUILD_TRIANGULATION_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

/// One observation of a track: the view it was seen in and its undistorted normalized
/// coordinates.
struct TrackObservation
{
  int view = 0;
  Vec2 normalized = Vec2::Zero();
};

struct TriangulationParams
{
  double max_reprojection_px = 3.84;  ///< caller resolves "auto" (0.6 % of the image width)
  double focal_px = 500.0;
  double min_angle_deg = 2.0;  ///< largest angle between two rays must reach this
  int min_track_length = 3;
  double min_depth = 0.2;
  double max_depth = 30.0;
  double min_height = -0.05;  ///< map z; below-floor points are reflections on shiny floors
  bool refine = true;         ///< Ceres point refinement with a Huber loss
  double huber_px = 1.0;
};

enum class TriangulationStatus : uint8_t {
  kOk,
  kTooFewObservations,  ///< fewer than min_track_length consistent observations
  kSmallAngle,
  kDepthOutOfRange,
  kBelowFloor,
};

struct TriangulationResult
{
  TriangulationStatus status = TriangulationStatus::kTooFewObservations;
  Vec3 position = Vec3::Zero();
  std::vector<int> inliers;  ///< indices into the observations, in input order
  double mean_reprojection_px = 0.0;
  double max_angle_deg = 0.0;
};

/// Triangulates a track with known camera poses, as COLMAP's point triangulator does for
/// hloc: RANSAC over observation pairs picks the consistent set, multi-view DLT on it, Ceres
/// refinement, and repeated removal of the worst observation while any exceeds the threshold.
/// `T_cam_map[view]` maps map points into each view's camera frame.
TriangulationResult triangulateTrack(
  const std::vector<Pose3> & T_cam_map, const std::vector<TrackObservation> & observations,
  const TriangulationParams & params);

}  // namespace landfall

#endif  // LANDFALL_BUILD_TRIANGULATION_HPP_
