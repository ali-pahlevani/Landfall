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

#ifndef LANDFALL_QUERY_CLUSTERING_HPP_
#define LANDFALL_QUERY_CLUSTERING_HPP_

#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

struct ClusterParams
{
  double radius = 0.5;  ///< m
  double angle_deg = 15.0;
};

struct PoseCluster
{
  std::vector<int> members;  ///< indices into the input, best score first
  Pose2 pose;                ///< the best member's pose
  double score = 0.0;        ///< the best member's score (inlier count)
};

/// Greedy SE(2) clustering: the best-scoring unassigned pose opens a cluster and absorbs every
/// unassigned pose within radius and angle of it. Clusters come out best first.
std::vector<PoseCluster> clusterPoses(
  const std::vector<Pose2> & poses, const std::vector<double> & scores,
  const ClusterParams & params);

struct AmbiguityParams
{
  double max_score_ratio = 0.5;  ///< S2 / S1 at or above this is ambiguous
  int min_inliers = 40;          ///< a runner-up below this can't make a result ambiguous
};

struct AmbiguityResult
{
  bool ambiguous = false;
  double score_ratio = 0.0;  ///< S2 / S1; 0 with a single cluster
};

/// Decides on the valid clusters only (after sanity and scan checks), best first.
AmbiguityResult checkAmbiguity(
  const std::vector<PoseCluster> & valid_clusters, const AmbiguityParams & params);

}  // namespace landfall

#endif  // LANDFALL_QUERY_CLUSTERING_HPP_
