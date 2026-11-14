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
#include "landfall/grid/scan_verifier.hpp"
#include "landfall/interfaces/retrieval_index.hpp"
#include "landfall/query/clustering.hpp"
#include "landfall/query/decision.hpp"
#include "landfall/query/localization_map.hpp"
#include "landfall/query/sanity.hpp"
#include "landfall/util/thread_pool.hpp"

namespace landfall
{

/// `scan_check.mode`: whether poses must also fit the LiDAR scan (plan §7.12).
enum class ScanCheckMode : uint8_t {
  kRequired,  ///< no scan, no pose
  kOptional,  ///< checked when a scan is there, vision only otherwise
  kDisabled,  ///< vision only (camera-only robots, explicit opt-in)
};

/// Parses "required", "optional" or "disabled".
std::optional<ScanCheckMode> parseScanCheckMode(const std::string & name);

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
  /// Ambiguity: a second valid cluster with S2 >= min_inliers and S2 / S1 at or above this. 0: any
  /// such competitor (with the LiDAR check, a far place that also fits the scan is a real one).
  double max_score_ratio = 0.0;
  ScanCheckMode scan_mode = ScanCheckMode::kRequired;
  ScanCheckParams scan;
  bool refine_with_scan = false;  ///< use the scan check's best pose of its search window
  int threads = 0;                ///< places solved in parallel; 0: hardware concurrency
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
  /// LiDAR scan taken near the image, its laser pose expressed in the base frame *at the image
  /// stamp* (the odometry between the two stamps folded in).
  std::optional<PlanarScan> scan;
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
  /// Share of the inliers matched in keyframes whose scan fitted a second place within 1 m (the
  /// build's kFlagScanAmbiguous): their landmarks are as uncertain along it.
  double twin_support = 0.0;
  Pose3 T_map_cam = Pose3::Identity();
  SanityResult sanity;  ///< metrics; valid when solved
  bool scan_checked = false;
  ScanCheckResult scan;  ///< valid when scan_checked
  Pose2 pose;            ///< base pose: the sanity check's, refined by the scan check if enabled
  std::vector<Vec2> inlier_pixels;
  std::vector<Vec2> outlier_pixels;
  int cluster = -1;  ///< index into FrameResult::clusters for valid hypotheses
};

/// What the LiDAR scan tells apart at the winning pose (scan_check.check_alternatives): the
/// relocalizer's acceptance depends on it (plan §7.12).
struct ScanUniqueness
{
  bool measured = false;
  /// scanFit at the pose less at the best place beyond 1 m or 30 degrees (bestAlternative); 1
  /// without such a place.
  double alternative_gap = 1.0;
  /// The test within 1 m ran: it decides only for poses with fewer than sanity.min_inliers inliers.
  bool twin_checked = false;
  /// Within 1 m (matchScan's window): the scan fits best at least 15 cm from the pose, or a second
  /// place fits within 1 nat of the best. Along that direction the scan can't vouch for the pose.
  bool twin = false;
  double twin_distance = 0.0;  ///< m from the pose to the other fit (0 without one)
  /// How much worse the other fit is than the pose's (negative: better), in matchScan's nats.
  double twin_nats = 0.0;
  /// alternative_gap >= scan_check.unique_margin, and the twin test ran and found none.
  bool unique = false;
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
  /// Valid hypotheses passed the scan check (else sanity only: no scan in optional mode, or
  /// disabled).
  bool scan_checked = false;
  Stage deepest_stage = Stage::kRetrieved;
  /// Base pose of the best valid cluster (also set when ambiguous, for diagnostics). Unset when the
  /// cluster has fewer than sanity.min_inliers inliers and the scan doesn't tell its pose apart:
  /// it is refused (its hypothesis says why), as is `best`.
  std::optional<Pose2> pose;
  /// Where the scan fits better than at that pose (scan_check.check_alternatives): the place
  /// vision found is then one of two that look alike, and the frame is ambiguous.
  std::optional<ScanAlternative> scan_alternative;
  /// What the scan tells apart at the best valid cluster's pose (also when it is then refused).
  ScanUniqueness uniqueness;
  /// Ambiguous because the pose rests on landmarks of keyframes whose scans fitted a second place
  /// within 1 m (scan_check.max_twin_support).
  bool twin_ambiguous = false;
  int best = -1;  ///< index of the winning hypothesis, -1 without a valid one
  FrameTimings timings;
};

/// Scores of a request (RelocalizationScores), worst across its frames: over each frame's winning
/// hypothesis, or its best solved one when it has none, the fewest inliers, the lowest inlier
/// ratio, coverage and scan hit ratio, and the largest reprojection, height and tilt errors, scan
/// violation ratio and ambiguity ratio.
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
  bool scan = false;  ///< a scan score exists
  double scan_hit_ratio = 0.0;
  double scan_violation_ratio = 0.0;
};

ScoreSummary summarizeScores(const std::vector<FrameResult> & frames);

/// Single-frame relocalization against a map database (plan §7.7-7.12): retrieve the top-k
/// keyframes, group them into places, match each place 2D-3D, solve the pose (upright LO-RANSAC with
/// P3P fallback), run the sanity checks and the LiDAR scan check, then cluster the poses that passed
/// and test for ambiguity, so a look-alike place that fails LiDAR is no competitor. Not reentrant:
/// one localize() at a time per instance.
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
  /// `scan` is the projected scan when the scan check applies, nullptr without one.
  Hypothesis solvePlace(
    const Place & place, const QueryFrame & frame, const Features & features, bool check_scan,
    const ProjectedScan * scan) const;
  /// What the scan tells apart at the winner's pose, given the best place beyond 1 m; the test
  /// within 1 m only with `near` (about 40 ms).
  ScanUniqueness measureUniqueness(
    const ProjectedScan & scan, const Hypothesis & winner, const ScanAlternative & alternative,
    bool near) const;

  std::shared_ptr<const LocalizationMap> map_;
  LocalizerParams params_;
  std::unique_ptr<IMatcher> matcher_;
  std::unique_ptr<ScanVerifier> verifier_;
  ThreadPool pool_;
};

}  // namespace landfall

#endif  // LANDFALL_QUERY_LOCALIZER_HPP_
