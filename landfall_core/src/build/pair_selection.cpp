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

#include "landfall/build/pair_selection.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <tuple>

namespace landfall
{

KeyframeView keyframeView(const Pose3 & T_map_cam)
{
  return {T_map_cam.translation(), T_map_cam.linear().col(2).normalized()};
}

std::vector<std::pair<int, int>> selectPairs(
  const std::vector<KeyframeView> & views, const std::vector<std::vector<int>> & retrieved,
  const PairSelectionParams & params)
{
  const int n = static_cast<int>(views.size());
  if (!retrieved.empty() && static_cast<int>(retrieved.size()) != n) {
    throw std::invalid_argument("selectPairs: one retrieval list per keyframe expected");
  }
  const double min_cos = std::cos(params.max_angle_deg * M_PI / 180.0);
  std::set<std::pair<int, int>> pairs;
  for (int i = 0; i < n; ++i) {
    std::set<int> chosen;
    const auto propose = [&](int j) {
      if (j == i || j < 0 || j >= n || static_cast<int>(chosen.size()) >= params.max_per_keyframe) {
        return;
      }
      if (chosen.insert(j).second) {
        pairs.emplace(std::min(i, j), std::max(i, j));
      }
    };
    if (!retrieved.empty()) {
      const int k = std::min(params.retrieval_k, static_cast<int>(retrieved[i].size()));
      for (int r = 0; r < k; ++r) {
        propose(retrieved[i][r]);
      }
    }
    // Nearby keyframes looking the same way, ranked by normalized distance + angle.
    std::vector<std::tuple<double, int>> nearby;
    for (int j = 0; j < n; ++j) {
      if (j == i) {
        continue;
      }
      const double distance = (views[i].center - views[j].center).norm();
      const double cosine = std::clamp(views[i].axis.dot(views[j].axis), -1.0, 1.0);
      if (distance > params.max_distance || cosine < min_cos) {
        continue;
      }
      const double angle = std::acos(cosine) * 180.0 / M_PI;
      nearby.emplace_back(distance / params.max_distance + angle / params.max_angle_deg, j);
    }
    std::sort(nearby.begin(), nearby.end());
    for (const auto & [score, j] : nearby) {
      propose(j);
    }
  }
  return {pairs.begin(), pairs.end()};
}

}  // namespace landfall
