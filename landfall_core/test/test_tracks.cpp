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

#include <gtest/gtest.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "landfall/build/pair_selection.hpp"
#include "landfall/build/tracks.hpp"
#include "synthetic_scene.hpp"

using landfall::KeyframeView;
using landfall::PairMatches;
using landfall::TrackNode;
using landfall::Vec3;

namespace
{

std::vector<std::pair<int, int>> nodes(const std::vector<TrackNode> & track)
{
  std::vector<std::pair<int, int>> out;
  out.reserve(track.size());
  for (const auto & n : track) {
    out.emplace_back(n.keyframe, n.keypoint);
  }
  return out;
}

}  // namespace

TEST(Tracks, ChainsMergeAndConflictsSplit)
{
  // Keyframes 0..3 with 10 keypoints each.
  const std::vector<int> counts(4, 10);
  std::vector<PairMatches> pairs = {
    {0, 1, {{0, 0}, {1, 1}, {2, 5}}},
    {1, 2, {{0, 3}, {1, 1}, {5, 6}}},
    {2, 3, {{3, 7}, {6, 6}}},
    // Conflict: keypoint 9 of keyframe 0 links to keypoints 8 and 9 of keyframe 3 (via 1 and 2).
    {0, 2, {{9, 8}}},
    {2, 3, {{8, 8}}},
    {0, 3, {{9, 9}}},
  };
  landfall::TrackStats stats;
  const auto tracks = landfall::buildTracks(counts, pairs, 2, &stats);
  // {0:0,1:0,2:3,3:7}, {0:1,1:1,2:1}, {0:2,1:5,2:6,3:6}; the conflicted component
  // {0:9,2:8,3:8,3:9} loses keyframe 3 and keeps {0:9,2:8}.
  ASSERT_EQ(tracks.size(), 4U);
  EXPECT_EQ(nodes(tracks[0]), (std::vector<std::pair<int, int>>{{0, 0}, {1, 0}, {2, 3}, {3, 7}}));
  EXPECT_EQ(nodes(tracks[1]), (std::vector<std::pair<int, int>>{{0, 1}, {1, 1}, {2, 1}}));
  EXPECT_EQ(nodes(tracks[2]), (std::vector<std::pair<int, int>>{{0, 2}, {1, 5}, {2, 6}, {3, 6}}));
  EXPECT_EQ(nodes(tracks[3]), (std::vector<std::pair<int, int>>{{0, 9}, {2, 8}}));
  EXPECT_EQ(stats.components, 4);
  EXPECT_EQ(stats.conflicts, 1);
  EXPECT_EQ(stats.dropped_nodes, 2);

  // The order of the pairs doesn't matter; min_length drops short tracks.
  std::reverse(pairs.begin(), pairs.end());
  const auto again = landfall::buildTracks(counts, pairs, 3);
  ASSERT_EQ(again.size(), 3U);
  EXPECT_EQ(nodes(again[0]), nodes(tracks[0]));
  EXPECT_THROW(landfall::buildTracks(counts, {{0, 1, {{0, 10}}}}, 2), std::out_of_range);
}

TEST(PairSelection, ProximityRetrievalAndCap)
{
  // Keyframes along x looking forward, and one at the far end looking back.
  std::vector<KeyframeView> views;
  views.reserve(11);
  for (int i = 0; i < 10; ++i) {
    views.push_back({Vec3(i * 1.0, 0, 1), Vec3::UnitX()});
  }
  views.push_back({Vec3(9.0, 0, 1), -Vec3::UnitX()});
  landfall::PairSelectionParams params;
  params.max_distance = 2.5;
  params.max_per_keyframe = 25;
  auto pairs = landfall::selectPairs(views, {}, params);
  // Neighbours within 2.5 m that look the same way: (i, i+1), (i, i+2).
  EXPECT_EQ(pairs.size(), 9U + 8U);
  EXPECT_TRUE(std::is_sorted(pairs.begin(), pairs.end()));
  EXPECT_EQ(std::count(pairs.begin(), pairs.end(), std::make_pair(9, 10)), 0);

  // Retrieval adds the far revisit; the cap keeps each keyframe's closest proposals.
  std::vector<std::vector<int>> retrieved(views.size());
  retrieved[0] = {10};
  pairs = landfall::selectPairs(views, retrieved, params);
  EXPECT_EQ(std::count(pairs.begin(), pairs.end(), std::make_pair(0, 10)), 1);
  params.max_per_keyframe = 1;
  pairs = landfall::selectPairs(views, {}, params);
  EXPECT_EQ(std::count(pairs.begin(), pairs.end(), std::make_pair(0, 2)), 0);
  EXPECT_EQ(std::count(pairs.begin(), pairs.end(), std::make_pair(0, 1)), 1);

  const auto view =
    landfall::keyframeView(landfall::test::basePose(1, 2, 0) * landfall::test::baseToCamera(0.0));
  EXPECT_TRUE(view.axis.isApprox(Vec3::UnitX(), 1e-12));
  EXPECT_TRUE(view.center.isApprox(Vec3(1.2, 2.0, 1.0), 1e-12));
}
