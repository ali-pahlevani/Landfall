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

#include "landfall/build/tracks.hpp"

#include <algorithm>
#include <map>
#include <numeric>
#include <stdexcept>

namespace landfall
{

namespace
{

class UnionFind
{
public:
  explicit UnionFind(std::size_t n) : parent_(n) { std::iota(parent_.begin(), parent_.end(), 0); }

  std::size_t find(std::size_t x)
  {
    while (parent_[x] != x) {
      parent_[x] = parent_[parent_[x]];
      x = parent_[x];
    }
    return x;
  }

  // The smaller root survives, which keeps the result independent of the merge order.
  void unite(std::size_t a, std::size_t b)
  {
    a = find(a);
    b = find(b);
    if (a != b) {
      parent_[std::max(a, b)] = std::min(a, b);
    }
  }

private:
  std::vector<std::size_t> parent_;
};

}  // namespace

std::vector<std::vector<TrackNode>> buildTracks(
  const std::vector<int> & keypoint_counts, const std::vector<PairMatches> & pairs, int min_length,
  TrackStats * stats)
{
  const int keyframes = static_cast<int>(keypoint_counts.size());
  std::vector<std::size_t> offset(keypoint_counts.size() + 1, 0);
  for (int k = 0; k < keyframes; ++k) {
    offset[k + 1] = offset[k] + static_cast<std::size_t>(keypoint_counts[k]);
  }
  std::vector<int> keyframe_of(offset.back());
  for (int k = 0; k < keyframes; ++k) {
    std::fill(
      keyframe_of.begin() + static_cast<std::ptrdiff_t>(offset[k]),
      keyframe_of.begin() + static_cast<std::ptrdiff_t>(offset[k + 1]), k);
  }
  const auto node = [&](int keyframe, int keypoint) {
    if (
      keyframe < 0 || keyframe >= keyframes || keypoint < 0 ||
      keypoint >= keypoint_counts[keyframe]) {
      throw std::out_of_range("buildTracks: match refers to a missing keypoint");
    }
    return offset[keyframe] + static_cast<std::size_t>(keypoint);
  };

  // Pass 1: components of the full match graph; find keyframes seen twice in a component.
  UnionFind first(offset.back());
  for (const auto & pair : pairs) {
    for (const auto & [ka, kb] : pair.matches) {
      first.unite(node(pair.a, ka), node(pair.b, kb));
    }
  }
  std::vector<char> matched(offset.back(), 0);
  for (const auto & pair : pairs) {
    for (const auto & [ka, kb] : pair.matches) {
      matched[node(pair.a, ka)] = 1;
      matched[node(pair.b, kb)] = 1;
    }
  }
  // (root, keyframe) -> number of keypoints; nodes of keyframes counted twice are removed.
  std::map<std::pair<std::size_t, int>, int> per_keyframe;
  std::map<std::size_t, int> component_size;
  for (std::size_t n = 0; n < matched.size(); ++n) {
    if (matched[n] != 0) {
      const std::size_t root = first.find(n);
      ++per_keyframe[{root, keyframe_of[n]}];
      ++component_size[root];
    }
  }
  TrackStats local;
  std::map<std::size_t, char> conflicted;
  std::vector<char> removed(offset.back(), 0);
  for (std::size_t n = 0; n < matched.size(); ++n) {
    if (matched[n] != 0) {
      const std::size_t root = first.find(n);
      if (per_keyframe[{root, keyframe_of[n]}] > 1) {
        removed[n] = 1;
        conflicted[root] = 1;
        ++local.dropped_nodes;
      }
    }
  }
  local.components = static_cast<int>(component_size.size());
  local.conflicts = static_cast<int>(conflicted.size());

  // Pass 2: components without the removed nodes.
  UnionFind second(offset.back());
  for (const auto & pair : pairs) {
    for (const auto & [ka, kb] : pair.matches) {
      const std::size_t na = node(pair.a, ka);
      const std::size_t nb = node(pair.b, kb);
      if (removed[na] == 0 && removed[nb] == 0) {
        second.unite(na, nb);
      }
    }
  }
  std::map<std::size_t, std::vector<TrackNode>> components;
  for (std::size_t n = 0; n < matched.size(); ++n) {
    if (matched[n] != 0 && removed[n] == 0) {
      components[second.find(n)].push_back(
        {keyframe_of[n], static_cast<int>(n - offset[keyframe_of[n]])});
    }
  }
  std::vector<std::vector<TrackNode>> tracks;
  for (auto & [root, nodes] : components) {
    if (static_cast<int>(nodes.size()) >= std::max(2, min_length)) {
      tracks.push_back(std::move(nodes));
    }
  }
  if (stats != nullptr) {
    *stats = local;
  }
  return tracks;
}

}  // namespace landfall
