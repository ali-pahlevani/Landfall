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

#ifndef LANDFALL_BUILD_PAIR_SELECTION_HPP_
#define LANDFALL_BUILD_PAIR_SELECTION_HPP_

#include <utility>
#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

struct PairSelectionParams
{
  double max_distance = 6.0;    ///< m between camera centers
  double max_angle_deg = 60.0;  ///< between optical axes
  int retrieval_k = 10;         ///< most similar keyframes by global descriptor
  int max_per_keyframe = 25;    ///< candidates each keyframe proposes
};

/// Camera center and optical axis (unit z axis of the optical frame) in the map.
struct KeyframeView
{
  Vec3 center = Vec3::Zero();
  Vec3 axis = Vec3::UnitX();
};

KeyframeView keyframeView(const Pose3 & T_map_cam);

/// Image pairs to match during the build. Each keyframe proposes its retrieval neighbours
/// (`retrieved[i]`, best first; they catch cross-session revisits and long sight lines) and then
/// nearby keyframes looking the same way, closest first, up to max_per_keyframe. The result is
/// the union of all proposals as (i, j) with i < j, sorted.
std::vector<std::pair<int, int>> selectPairs(
  const std::vector<KeyframeView> & views, const std::vector<std::vector<int>> & retrieved,
  const PairSelectionParams & params);

}  // namespace landfall

#endif  // LANDFALL_BUILD_PAIR_SELECTION_HPP_
