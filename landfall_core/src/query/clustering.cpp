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

#include "landfall/query/clustering.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace landfall
{

std::vector<PoseCluster> clusterPoses(
  const std::vector<Pose2> & poses, const std::vector<double> & scores,
  const ClusterParams & params)
{
  if (poses.size() != scores.size()) {
    throw std::invalid_argument("clusterPoses: one score per pose");
  }
  std::vector<int> order(poses.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return scores[a] > scores[b]; });
  const double max_angle = params.angle_deg * M_PI / 180.0;
  std::vector<char> assigned(poses.size(), 0);
  std::vector<PoseCluster> clusters;
  for (int seed : order) {
    if (assigned[seed] != 0) {
      continue;
    }
    PoseCluster cluster;
    cluster.pose = poses[seed];
    cluster.score = scores[seed];
    for (int candidate : order) {
      if (
        assigned[candidate] == 0 &&
        translationDistance(poses[candidate], cluster.pose) <= params.radius &&
        rotationDistance(poses[candidate], cluster.pose) <= max_angle) {
        assigned[candidate] = 1;
        cluster.members.push_back(candidate);
      }
    }
    clusters.push_back(std::move(cluster));
  }
  return clusters;
}

AmbiguityResult checkAmbiguity(
  const std::vector<PoseCluster> & valid_clusters, const AmbiguityParams & params)
{
  AmbiguityResult result;
  if (valid_clusters.size() < 2 || valid_clusters[0].score <= 0.0) {
    return result;
  }
  const double runner_up = valid_clusters[1].score;
  result.score_ratio = runner_up / valid_clusters[0].score;
  result.ambiguous =
    runner_up >= params.min_inliers && result.score_ratio >= params.max_score_ratio;
  return result;
}

}  // namespace landfall
