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

#ifndef LANDFALL_QUERY_LOCALIZER_HPP_
#define LANDFALL_QUERY_LOCALIZER_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "landfall/camera_model.hpp"
#include "landfall/features/matchers.hpp"
#include "landfall/geometry/gravity.hpp"
#include "landfall/geometry/pnp.hpp"
#include "landfall/interfaces/retrieval_index.hpp"
#include "landfall/query/clustering.hpp"
#include "landfall/query/decision.hpp"
#include "landfall/query/localization_map.hpp"
#include "landfall/query/sanity.hpp"
#include "landfall/util/thread_pool.hpp"

namespace landfall
{

struct LocalizerParams
{
  int top_k = 10;  ///< keyframes retrieved per frame
  /// Retrieved keyframes form one place when they share this many landmarks ...
  int place_min_shared = 15;
  /// ... or their cameras are this close (plan §7.7).
  double place_max_distance = 2.0;
  double place_max_angle_deg = 45.0;
  RatioMatcherParams ratio;  ///< binary descriptors
  MutualNearestParams mnn;   ///< float descriptors
  PnpStrategy pnp = PnpStrategy::kUprightThenP3P;
  double pnp_threshold_px = 0.0;  ///< 0: 1 % of the image width
  int pnp_max_iterations = 10000;
  double pnp_success_prob = 0.9999;
  uint64_t seed = 42;
  /// `max_mean_reprojection_px` 0 means half the PnP threshold.
  SanityParams sanity;
  ClusterParams clustering;
  double max_score_ratio = 0.5;  ///< ambiguity: S2 / S1 at or above this (S2 >= min_inliers)
  int threads = 0;               ///< places solved in parallel; 0: hardware concurrency
};

/// One query image with what the robot knew at its stamp.
struct QueryFrame
{
  int64_t stamp_ns = 0;
  cv::Mat image;                     ///< 8-bit gray or BGR; unused when `features` is set
  std::optional<Features> features;  ///< precomputed features (tests, replays)
  CameraModel camera;
  Pose3 T_base_cam = Pose3::Identity();  ///< extrinsics at the stamp (TF)
  GravityEstimate gravity;
};

/// A pose candidate from one place (group of retrieved keyframes).
struct Hypothesis
{
  std::vector<int64_t> keyframe_ids;  ///< the place, best retrieval rank first
  int correspondences = 0;            ///< 2D-3D matches after deduplication
  Stage stage = Stage::kRetrieved;
  std::string rejection;  ///< why it stopped; empty for a valid hypothesis
  bool solved = false;
  PnpMethod method = PnpMethod::kP3P;
  Pose3 T_map_cam = Pose3::Identity();
  SanityResult sanity;  ///< metrics; valid when solved
  std::vector<Vec2> inlier_pixels;
  std::vector<Vec2> outlier_pixels;
  int cluster = -1;  ///< index into FrameResult::clusters for valid hypotheses
};

struct FrameTimings
{
  double features_s = 0.0;
  double retrieval_s = 0.0;
  double solve_s = 0.0;  ///< matching, PnP and checks of every place
  double total_s = 0.0;
};

struct FrameResult
{
  int num_features = 0;
  std::vector<RetrievalCandidate> retrieved;
  std::vector<Hypothesis> hypotheses;  ///< best first: deeper stage, then more inliers
  std::vector<PoseCluster> clusters;   ///< of the valid hypotheses, best first
  AmbiguityResult ambiguity;
  Stage deepest_stage = Stage::kRetrieved;
  /// Base pose of the best valid cluster (also set when ambiguous, for diagnostics).
  std::optional<Pose2> pose;
  int best = -1;  ///< index of the winning hypothesis, -1 without a valid one
  FrameTimings timings;
};

/// Scores of a request (RelocalizationScores), worst across its frames: over each frame's winning
/// hypothesis, or its best solved one when it has none, the fewest inliers, the lowest inlier
/// ratio and coverage, and the largest reprojection, height and tilt errors and ambiguity ratio.
struct ScoreSummary
{
  bool any = false;  ///< false when no frame solved a pose (the pose scores are then 0)
  int num_inliers = 0;
  double inlier_ratio = 0.0;
  double mean_reprojection_px = 0.0;
  double height_error_m = 0.0;
  double tilt_error_deg = 0.0;
  double inlier_coverage = 0.0;
  double ambiguity_ratio = 0.0;
};

ScoreSummary summarizeScores(const std::vector<FrameResult> & frames);

/// Single-frame visual relocalization against a map database (plan §7.7-7.9, §7.11): retrieve the
/// top-k keyframes, group them into places, match each place 2D-3D, solve the pose (upright
/// LO-RANSAC with P3P fallback), run the sanity checks, cluster the valid poses and test for
/// ambiguity. Not reentrant: one localize() at a time per instance.
class Localizer
{
public:
  Localizer(std::shared_ptr<const LocalizationMap> map, const LocalizerParams & params);

  const LocalizationMap & map() const { return *map_; }
  const LocalizerParams & params() const { return params_; }
  /// Thresholds can change between frames; the thread count is fixed at construction.
  void setParams(const LocalizerParams & params);

  /// Throws std::invalid_argument for an empty image without features.
  FrameResult localize(const QueryFrame & frame);

private:
  struct Place
  {
    std::vector<int64_t> keyframe_ids;
  };
  std::vector<Place> groupPlaces(const std::vector<RetrievalCandidate> & retrieved) const;
  Hypothesis solvePlace(
    const Place & place, const QueryFrame & frame, const Features & features) const;

  std::shared_ptr<const LocalizationMap> map_;
  LocalizerParams params_;
  std::unique_ptr<IMatcher> matcher_;
  ThreadPool pool_;
};

}  // namespace landfall

#endif  // LANDFALL_QUERY_LOCALIZER_HPP_
