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

#ifndef LANDFALL_GRID_SCAN_MATCHER_HPP_
#define LANDFALL_GRID_SCAN_MATCHER_HPP_

#include <optional>
#include <string>
#include <vector>

#include "landfall/grid/likelihood_field.hpp"
#include "landfall/types.hpp"

namespace landfall
{

/// Builder keys `poses.*` of the scan matching (plan §7.6).
struct ScanMatchParams
{
  double search_xy = 1.0;       ///< m: half-width of the search window around the guess
  double search_yaw_deg = 6.0;  ///< half-width of the heading window
  /// m: spread of a beam endpoint around the nearest obstacle (the likelihood field's sigma); also
  /// the noise of one beam in the covariance.
  double beam_sigma = 0.05;
  double hit_tolerance = 0.10;  ///< m: an endpoint this close to an obstacle is a hit
  int min_beams = 60;           ///< fewer valid beams: no match
  double min_hit_ratio = 0.6;   ///< hits / beams at the best pose; below: no match
  /// Beams count as this many independent measurements in the covariance: neighbouring beams hit
  /// the same surface and the map's own errors, so their errors are far from independent.
  int effective_beams = 40;
  /// m: a separate fit of the window at least this far from the best is another place the scan
  /// could be at (ScanMatch::alternative).
  double alternative_distance = 0.15;
};

/// Where a scan fits the map best near a guess, and how sure that is per direction.
struct ScanMatch
{
  bool valid = false;
  std::string reason;  ///< why not, when invalid
  Pose2 pose;          ///< best fit (base pose)
  /// Covariance of x, y and yaw at the best fit: the geometry (each hit beam constrains the pose
  /// along its surface's normal: small across a corridor, large along it) plus the fit's spread
  /// over the search window (thick walls flatten the minimum; repeated structure, e.g. shelf legs
  /// along an aisle, gives near-equal minima a few tens of centimetres apart).
  Mat3 covariance = Mat3::Identity();
  double hit_ratio = 0.0;        ///< hits / beams at the best fit
  double guess_hit_ratio = 0.0;  ///< hits / beams at the guess
  int beams = 0;                 ///< valid beams used
  /// The best other place in the window: a separate maximum of the fit at least
  /// alternative_distance from the best (a wall mapped twice, repeating structure), refined.
  std::optional<Pose2> alternative;
  /// How much worse the alternative fits, in nats with the beams counted as effective_beams; near
  /// 0 the scan can't tell the two apart (it may even be negative: the best is the best mean fit,
  /// with a pull to the guess on ties).
  double alternative_nats = 0.0;
  /// How much worse the guess itself fits than the best, in the same nats (set with the best).
  double guess_nats = 0.0;
};

/// Matches scan endpoints (base frame, e.g. ProjectedScan::endpoints) to the map around `guess`:
/// a correlative search over the window (every 5 cm and 0.5 deg), refined to 2.5 mm and 0.025 deg
/// on the interpolated distance field. Near-ties go to the pose closest to the guess. Invalid with
/// too few beams, a poor fit, or the best fit on the edge of the window (the scan may fit better
/// outside it). Deterministic and thread-safe.
ScanMatch matchScan(
  const LikelihoodField & field, const std::vector<Vec2> & endpoints, const Pose2 & guess,
  const ScanMatchParams & params);

}  // namespace landfall

#endif  // LANDFALL_GRID_SCAN_MATCHER_HPP_
