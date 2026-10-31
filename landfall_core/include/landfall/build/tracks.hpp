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

#ifndef LANDFALL_BUILD_TRACKS_HPP_
#define LANDFALL_BUILD_TRACKS_HPP_

#include <utility>
#include <vector>

namespace landfall
{

/// Verified matches of one image pair: keypoint index in keyframe a, keypoint index in b.
struct PairMatches
{
  int a = 0;
  int b = 0;
  std::vector<std::pair<int, int>> matches;
};

/// One keypoint of one keyframe.
struct TrackNode
{
  int keyframe = 0;
  int keypoint = 0;
};

struct TrackStats
{
  int components = 0;  ///< connected components with at least two keypoints
  int conflicts = 0;   ///< components that held two keypoints of one keyframe
  int dropped_nodes = 0;
};

/// Feature tracks: connected components of the match graph (union-find). A component holding two
/// or more keypoints of the same keyframe is ambiguous there, so all of that keyframe's keypoints
/// leave it and the rest is split again by the remaining matches. Tracks shorter than `min_length`
/// are dropped. Deterministic: tracks are ordered by their first node, nodes by keyframe.
std::vector<std::vector<TrackNode>> buildTracks(
  const std::vector<int> & keypoint_counts, const std::vector<PairMatches> & pairs, int min_length,
  TrackStats * stats = nullptr);

}  // namespace landfall

#endif  // LANDFALL_BUILD_TRACKS_HPP_
