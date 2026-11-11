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

#include "landfall/build/map_builder.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <utility>

#include "landfall/build/tracks.hpp"
#include "landfall/db/codec.hpp"
#include "landfall/db/database.hpp"
#include "landfall/features/retrieval.hpp"
#include "landfall/geometry/two_view.hpp"
#include "landfall/grid/likelihood_field.hpp"
#include "landfall/grid/occupancy_grid.hpp"
#include "landfall/grid/scan_matcher.hpp"
#include "landfall/record/image_quality.hpp"
#include "landfall/util/json.hpp"
#include "landfall/util/thread_pool.hpp"
#include "landfall/version.hpp"

namespace landfall
{

namespace
{

constexpr int kThumbnailSize = 160;
constexpr int kThumbnailQuality = 80;
// The triangulation threshold follows the image size: 0.6 % of the width (3.84 px at 640 px).
constexpr double kAutoTriangulationFraction = 0.006;
// The post-BA re-filter is this much tighter than the triangulation threshold.
constexpr double kAutoRefilterFactor = 0.75;

// SplitMix64: decorrelated per-pair seeds from one build seed.
uint64_t mixSeed(uint64_t seed, uint64_t a, uint64_t b)
{
  uint64_t z = seed + 0x9e3779b97f4a7c15ULL * (a * 1000003ULL + b + 1);
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

std::string utcNow()
{
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buffer;
}

std::vector<uint8_t> thumbnail(const cv::Mat & image)
{
  const double scale =
    std::min(1.0, static_cast<double>(kThumbnailSize) / std::max(image.cols, image.rows));
  cv::Mat small;
  cv::resize(image, small, cv::Size(), scale, scale, cv::INTER_AREA);
  if (small.channels() == 4) {
    cv::cvtColor(small, small, cv::COLOR_BGRA2BGR);
  }
  std::vector<uint8_t> jpeg;
  cv::imencode(".jpg", small, jpeg, {cv::IMWRITE_JPEG_QUALITY, kThumbnailQuality});
  return jpeg;
}

// Reprojection error in pixels; infinite behind the camera.
double reprojectionPx(const Pose3 & T_cam_map, const Vec3 & X, const Vec2 & x, double focal)
{
  const Vec3 p = T_cam_map * X;
  if (p.z() <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return (p.hnormalized() - x).norm() * focal;
}

double descriptorDistance(const cv::Mat & a, const cv::Mat & b, DescriptorType type)
{
  return cv::norm(a, b, type == DescriptorType::kBinary ? cv::NORM_HAMMING : cv::NORM_L2);
}

// Runs fn over [0, n) in chunks so progress can be reported between them.
void chunked(
  ThreadPool & pool, std::size_t n, const std::function<void(std::size_t)> & fn,
  const std::function<void(std::size_t)> & report)
{
  const std::size_t chunk = std::max<std::size_t>(64, n / 50);
  for (std::size_t begin = 0; begin < n; begin += chunk) {
    const std::size_t count = std::min(chunk, n - begin);
    pool.parallelFor(count, [&](std::size_t i) { fn(begin + i); });
    report(begin + count);
  }
}

struct Landmark
{
  Vec3 position = Vec3::Zero();
  std::vector<TrackNode> observations;
  double mean_reprojection_px = 0.0;
  double max_angle_deg = 0.0;
};

class Stopwatch
{
public:
  double lap()
  {
    const auto now = std::chrono::steady_clock::now();
    const double seconds = std::chrono::duration<double>(now - last_).count();
    last_ = now;
    return seconds;
  }

private:
  std::chrono::steady_clock::time_point last_ = std::chrono::steady_clock::now();
};

void validate(const BuildInput & input)
{
  if (input.cameras.size() != 1) {
    throw std::runtime_error("a map is built from exactly one camera in this version");
  }
  if (input.keyframes.size() < 2) {
    throw std::runtime_error(
      "at least two keyframes are needed, got " + std::to_string(input.keyframes.size()));
  }
  if (input.sessions.empty()) {
    throw std::runtime_error("no recording session given");
  }
  if (input.map.grid.width <= 0 || input.map.grid.height <= 0) {
    throw std::runtime_error("no occupancy map given");
  }
  for (std::size_t i = 0; i < input.keyframes.size(); ++i) {
    const BuildKeyframe & k = input.keyframes[i];
    if (k.session < 0 || k.session >= static_cast<int>(input.sessions.size()) || k.camera != 0) {
      throw std::runtime_error("keyframe " + std::to_string(i) + " has an invalid session/camera");
    }
    if (!k.features && !k.load_image) {
      throw std::runtime_error("keyframe " + std::to_string(i) + " has neither image nor features");
    }
  }
}

}  // namespace

const char * toString(BuildStage stage)
{
  switch (stage) {
    case BuildStage::kPoses:
      return "poses";
    case BuildStage::kFeatures:
      return "features";
    case BuildStage::kVocabulary:
      return "vocabulary";
    case BuildStage::kPairs:
      return "pairs";
    case BuildStage::kMatching:
      return "matching";
    case BuildStage::kTracks:
      return "tracks";
    case BuildStage::kTriangulation:
      return "triangulation";
    case BuildStage::kBundleAdjustment:
      return "bundle_adjustment";
    case BuildStage::kWriting:
      return "writing";
  }
  return "unknown";
}

BuildStats buildMap(
  const BuildInput & input, const BuildConfigFile & file, const std::string & db_path,
  const BuildProgress & progress, const BackendRegistry * registry)
{
  validate(input);
  if (std::filesystem::exists(db_path)) {
    throw std::runtime_error("output database already exists: " + db_path);
  }
  const BuilderConfig & config = file.builder;
  const auto report = [&](BuildStage stage, std::size_t done, std::size_t total) {
    if (progress) {
      progress(stage, done, total);
    }
  };
  BuildStats stats;
  stats.stage_seconds.assign(kBuildStageCount, 0.0);
  Stopwatch watch;
  ThreadPool pool(config.threads);
  const BuildCamera & camera = input.cameras.front();
  const double focal = camera.model.focal();
  const auto n = input.keyframes.size();
  stats.keyframes = static_cast<int>(n);
  stats.sessions = static_cast<int>(input.sessions.size());

  // --- Keyframe poses: scans matched to the map, odometry between keyframes (plan §7.6) --------
  // The recording's localization (e.g. AMCL, off by tens of centimetres along an aisle while it
  // reports a few) is trusted only where the scan can't tell; the pose graph of these priors and
  // the odometry gives the poses that pair selection, triangulation and the adjustment start from.
  report(BuildStage::kPoses, 0, n);
  std::vector<uint32_t> flags(n, 0);
  std::vector<BaKeyframe> ba_keyframes(n);
  bool any_scan = false;
  bool any_odometry = false;
  for (std::size_t i = 0; i < n; ++i) {
    const BuildKeyframe & k = input.keyframes[i];
    BaKeyframe & b = ba_keyframes[i];
    b.prior = k.prior;
    b.covariance = k.covariance;
    b.T_base_cam = k.T_base_cam;
    b.focal_px = focal;
    b.session = k.session;
    if (config.poses.odometry && k.odom) {
      b.odom = k.odom;
      any_odometry = true;
    }
    any_scan = any_scan || k.scan.has_value();
  }
  if (config.poses.scan_matching && any_scan) {
    const auto field = std::make_shared<const LikelihoodField>(input.map.grid);
    ScanCheckParams projection;
    projection.max_range =
      config.poses.max_range > 0.0 ? config.poses.max_range : std::numeric_limits<double>::max();
    projection.check_alternatives = false;
    const ScanVerifier projector(field, projection);
    std::vector<std::optional<ScanMatch>> matches(n);
    chunked(
      pool, n,
      [&](std::size_t i) {
        const BuildKeyframe & k = input.keyframes[i];
        if (!k.scan) {
          return;
        }
        PlanarScan scan = *k.scan;
        scan.range_max -= 1e-3;  // readings at the scanner's maximum are no returns
        const ProjectedScan projected = projector.project(scan, Vec3(0.0, 0.0, -1.0));
        matches[i] = matchScan(*field, projected.endpoints, k.prior, config.poses.scan);
      },
      [&](std::size_t done) { report(BuildStage::kPoses, done, n); });
    // Fused with the recorded pose: both count where they agree; where the scan contradicts the
    // recording (a localization that drifted), the recording only fills in what the scan can't
    // tell.
    const auto information = [](double std_xy, double std_yaw_deg) -> Mat3 {
      const double std_yaw = std_yaw_deg * M_PI / 180.0;
      return Vec3(1.0 / (std_xy * std_xy), 1.0 / (std_xy * std_xy), 1.0 / (std_yaw * std_yaw))
        .asDiagonal();
    };
    const Mat3 trusted =
      information(config.poses.recorded_std_xy, config.poses.recorded_std_yaw_deg);
    const Mat3 doubted =
      information(config.poses.disagreement_std_xy, config.poses.disagreement_std_yaw_deg);
    // A recorded pose the scan agrees with is trusted only as far as the scan could check it: in
    // each of the scan's principal directions, at the scan's own std, between recorded_std_* and
    // disagreement_std_* (along an aisle the scan can't see, the recording barely counts).
    const auto confirmed = [&](const Mat3 & scan_covariance) -> Mat3 {
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> xy(
        0.5 * (scan_covariance.topLeftCorner<2, 2>() +
               scan_covariance.topLeftCorner<2, 2>().transpose()));
      Eigen::Vector2d inverse_variance;
      for (int d = 0; d < 2; ++d) {
        const double std_xy = std::clamp(
          std::sqrt(std::max(xy.eigenvalues()(d), 0.0)), config.poses.recorded_std_xy,
          config.poses.disagreement_std_xy);
        inverse_variance(d) = 1.0 / (std_xy * std_xy);
      }
      const double std_yaw = std::clamp(
        std::sqrt(std::max(scan_covariance(2, 2), 0.0)),
        config.poses.recorded_std_yaw_deg * M_PI / 180.0,
        config.poses.disagreement_std_yaw_deg * M_PI / 180.0);
      Mat3 result = Mat3::Zero();
      result.topLeftCorner<2, 2>() =
        xy.eigenvectors() * inverse_variance.asDiagonal() * xy.eigenvectors().transpose();
      result(2, 2) = 1.0 / (std_yaw * std_yaw);
      return result;
    };
    const double floor_yaw = config.ba.prior_floor_yaw_deg * M_PI / 180.0;
    const Vec3 floors(
      config.ba.prior_floor_xy * config.ba.prior_floor_xy,
      config.ba.prior_floor_xy * config.ba.prior_floor_xy, floor_yaw * floor_yaw);
    for (std::size_t i = 0; i < n; ++i) {
      if (!matches[i]) {
        continue;
      }
      const ScanMatch & m = *matches[i];
      if (!m.valid) {
        flags[i] |= db::KeyframeRecord::kFlagScanUnmatched;
        ++stats.poses_scan_unmatched;
        continue;
      }
      const Pose2 & recorded = input.keyframes[i].prior;
      const Mat3 scan_information = m.covariance.inverse();
      const Vec3 offset(
        m.pose.x - recorded.x, m.pose.y - recorded.y, normalizeAngle(m.pose.yaw - recorded.yaw));
      const Mat3 agreement = m.covariance + trusted.inverse();
      const bool agrees = offset.dot(agreement.ldlt().solve(offset)) <= config.poses.agreement_chi2;
      if (!agrees) {
        ++stats.poses_scan_contradicted;
      }
      const Mat3 total = scan_information + (agrees ? confirmed(m.covariance) : doubted);
      const Vec3 fused = total.ldlt().solve(scan_information * offset);
      Mat3 covariance = total.inverse();
      covariance = 0.5 * (covariance + covariance.transpose());
      covariance.diagonal() += floors;
      BaKeyframe & b = ba_keyframes[i];
      b.prior = {recorded.x + fused.x(), recorded.y + fused.y(), recorded.yaw + fused.z()};
      b.prior_sqrt_information = Eigen::LLT<Mat3>(covariance.inverse()).matrixU();
      flags[i] |= db::KeyframeRecord::kFlagScanMatched;
      ++stats.poses_scan_matched;
    }
    if (stats.poses_scan_unmatched > 0) {
      stats.warnings.push_back(
        std::to_string(stats.poses_scan_unmatched) +
        " keyframe(s) had a scan that didn't match the map; their recorded pose was used");
    }
  }
  std::vector<Pose2> start(n);
  for (std::size_t i = 0; i < n; ++i) {
    start[i] = ba_keyframes[i].prior;
  }
  if (stats.poses_scan_matched > 0 || any_odometry) {
    const BaResult graph = adjustPosePriors(ba_keyframes, {}, {}, config.ba);
    if (graph.usable) {
      start = graph.poses;
      stats.poses_odometry_links = graph.odometry_links;
    } else {
      stats.warnings.push_back(
        "keyframe pose graph failed (" + graph.message + "); the priors were used");
    }
  }
  double correction_sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const Pose2 & recorded = input.keyframes[i].prior;
    const double shift = std::hypot(start[i].x - recorded.x, start[i].y - recorded.y);
    correction_sum += shift;
    stats.poses_max_correction_m = std::max(stats.poses_max_correction_m, shift);
    stats.poses_max_correction_yaw_deg = std::max(
      stats.poses_max_correction_yaw_deg,
      std::abs(normalizeAngle(start[i].yaw - recorded.yaw)) * 180.0 / M_PI);
  }
  stats.poses_mean_correction_m = correction_sum / static_cast<double>(n);
  report(BuildStage::kPoses, n, n);
  stats.stage_seconds[static_cast<int>(BuildStage::kPoses)] = watch.lap();

  // --- Features, learned global descriptors and thumbnails -------------------------------------
  BackendRegistry builtin;
  registerBuiltinBackends(builtin);
  const BackendRegistry & backends = registry != nullptr ? *registry : builtin;
  // How learned backends run: one ONNX Runtime thread per image, the pool runs images in parallel.
  const auto withRuntime = [&config](BackendOptions options) {
    options[kModelsDirOption] = config.models_dir;
    options[kProviderOption] = config.execution_provider;
    options[kThreadsOption] = "1";
    return options;
  };
  const auto extractor =
    backends.createLocalFeatures(config.local_backend, withRuntime(config.local_options));
  const bool use_bow = config.global_backend == "bow";
  std::unique_ptr<IGlobalDescriptor> learned_global;
  if (!use_bow) {
    if (!backends.hasGlobalDescriptor(config.global_backend)) {
      throw std::runtime_error(
        "global backend '" + config.global_backend +
        "' is not available (learned backends need a build with ONNX Runtime)");
    }
    learned_global =
      backends.createGlobalDescriptor(config.global_backend, withRuntime(config.global_options));
  }
  std::vector<Features> features(n);
  std::vector<std::vector<float>> dense(n);
  std::vector<std::vector<uint8_t>> thumbnails(n);
  report(BuildStage::kFeatures, 0, n);
  chunked(
    pool, n,
    [&](std::size_t i) {
      const BuildKeyframe & k = input.keyframes[i];
      if (k.features && !learned_global) {
        features[i] = *k.features;
        return;
      }
      // An image: for the features, or (precomputed features) for a learned global descriptor.
      const cv::Mat image = k.load_image ? k.load_image() : cv::Mat();
      if (image.empty()) {
        throw std::runtime_error("keyframe " + std::to_string(i) + ": image could not be loaded");
      }
      if (image.cols != camera.model.width() || image.rows != camera.model.height()) {
        throw std::runtime_error(
          "keyframe " + std::to_string(i) + ": image size " + std::to_string(image.cols) + "x" +
          std::to_string(image.rows) + " differs from the camera's " +
          std::to_string(camera.model.width()) + "x" + std::to_string(camera.model.height()));
      }
      if (k.features) {
        features[i] = *k.features;
      } else {
        features[i] = extractor->extract(toGray(image));
        thumbnails[i] = thumbnail(image);
      }
      if (learned_global) {
        dense[i] = learned_global->compute(image, &features[i]).dense;
      }
    },
    [&](std::size_t done) { report(BuildStage::kFeatures, done, n); });
  const DescriptorType descriptor_type = features.front().type;
  for (const auto & f : features) {
    if (f.type != descriptor_type) {
      throw std::runtime_error("keyframes have different descriptor types");
    }
  }

  // Undistorted normalized coordinates of every keypoint (nullopt where undistortion fails).
  std::vector<std::vector<Vec2>> normalized(n);
  std::vector<std::vector<char>> usable(n);
  pool.parallelFor(n, [&](std::size_t i) {
    const auto & kps = features[i].keypoints;
    normalized[i].resize(kps.size());
    usable[i].assign(kps.size(), 0);
    for (std::size_t j = 0; j < kps.size(); ++j) {
      usable[i][j] = camera.model.undistort(Vec2(kps[j].x, kps[j].y), &normalized[i][j]) ? 1 : 0;
    }
  });
  stats.stage_seconds[static_cast<int>(BuildStage::kFeatures)] = watch.lap();

  // --- Vocabulary and bag-of-words (bow only) --------------------------------------------------
  report(BuildStage::kVocabulary, 0, 1);
  bool any_features = false;
  for (const auto & f : features) {
    any_features = any_features || !f.descriptors.empty();
  }
  if (!any_features) {
    throw std::runtime_error("no features were found in any keyframe");
  }
  VocabularyParams vocabulary_params = config.vocabulary;
  vocabulary_params.seed = config.seed;
  std::shared_ptr<VocabularyTree> vocabulary;
  std::unique_ptr<BowGlobalDescriptor> bow_backend;
  std::vector<BowVector> bow(n);
  if (use_bow) {
    cv::Mat all_descriptors;
    for (const auto & f : features) {
      if (!f.descriptors.empty()) {
        all_descriptors.push_back(f.descriptors);
      }
    }
    vocabulary =
      std::make_shared<VocabularyTree>(VocabularyTree::train(all_descriptors, vocabulary_params));
    all_descriptors.release();
    std::vector<std::vector<uint32_t>> words(n);
    pool.parallelFor(
      n, [&](std::size_t i) { words[i] = vocabulary->words(features[i].descriptors); });
    vocabulary->computeIdf(words);
    pool.parallelFor(n, [&](std::size_t i) { bow[i] = vocabulary->bow(features[i].descriptors); });
    bow_backend = std::make_unique<BowGlobalDescriptor>(vocabulary);
  }
  // The keyframes' global descriptors, in either representation.
  const auto globalDescriptor = [&](std::size_t i) {
    return use_bow ? GlobalDescriptor{{}, bow[i]} : GlobalDescriptor{dense[i], {}};
  };
  report(BuildStage::kVocabulary, 1, 1);
  stats.stage_seconds[static_cast<int>(BuildStage::kVocabulary)] = watch.lap();

  // --- Pair selection --------------------------------------------------------------------------
  report(BuildStage::kPairs, 0, 1);
  std::unique_ptr<IRetrievalIndex> index;
  if (use_bow) {
    index = std::make_unique<BowIndex>();
  } else {
    index = std::make_unique<DenseIndex>();
  }
  for (std::size_t i = 0; i < n; ++i) {
    index->add(static_cast<int64_t>(i), globalDescriptor(i));
  }
  std::vector<std::vector<int>> retrieved(n);
  pool.parallelFor(n, [&](std::size_t i) {
    for (const auto & candidate :
         index->search(globalDescriptor(i), config.pairs.retrieval_k + 1)) {
      if (candidate.id != static_cast<int64_t>(i)) {
        retrieved[i].push_back(static_cast<int>(candidate.id));
      }
    }
  });
  std::vector<Pose3> prior_cameras(n);
  std::vector<KeyframeView> views(n);
  for (std::size_t i = 0; i < n; ++i) {
    prior_cameras[i] = cameraPose(start[i], input.keyframes[i].T_base_cam);
    views[i] = keyframeView(prior_cameras[i]);
  }
  const auto pairs = selectPairs(views, retrieved, config.pairs);
  stats.pairs_candidate = static_cast<int>(pairs.size());
  report(BuildStage::kPairs, 1, 1);
  stats.stage_seconds[static_cast<int>(BuildStage::kPairs)] = watch.lap();

  // --- Matching and two-view verification ------------------------------------------------------
  const std::string matcher_name =
    config.matcher == "auto" ? defaultMatcherFor(descriptor_type) : config.matcher;
  const auto matcher = backends.createMatcher(matcher_name, withRuntime(config.matcher_options));
  std::vector<PairMatches> verified(pairs.size());
  report(BuildStage::kMatching, 0, pairs.size());
  chunked(
    pool, pairs.size(),
    [&](std::size_t p) {
      const auto [a, b] = pairs[p];
      verified[p].a = a;
      verified[p].b = b;
      const Matches matches = matcher->match(features[a], features[b]);
      std::vector<Vec2> x1;
      std::vector<Vec2> x2;
      std::vector<std::pair<int, int>> kept;
      for (const Match & m : matches) {
        if (usable[a][m.query] != 0 && usable[b][m.train] != 0) {
          x1.push_back(normalized[a][m.query]);
          x2.push_back(normalized[b][m.train]);
          kept.emplace_back(m.query, m.train);
        }
      }
      if (static_cast<int>(kept.size()) < config.pair_min_inliers) {
        return;
      }
      const Pose3 T_b_a = prior_cameras[b].inverse() * prior_cameras[a];
      TwoViewParams params;
      params.max_error_px = config.pair_max_epipolar_error_px;
      params.focal_px = focal;
      params.seed = mixSeed(config.seed, static_cast<uint64_t>(a), static_cast<uint64_t>(b));
      const bool rotation_only = T_b_a.translation().norm() < config.pair_min_baseline;
      const auto geometry = rotation_only ? estimateRelativeRotation(x1, x2, params)
                                          : estimateRelativePose(x1, x2, params);
      if (!geometry || static_cast<int>(geometry->inliers.size()) < config.pair_min_inliers) {
        return;
      }
      const double deviation =
        rotationAngle(geometry->R_21 * T_b_a.linear().transpose()) * 180.0 / M_PI;
      if (deviation > config.pair_max_rotation_deviation_deg) {
        return;
      }
      for (int i : geometry->inliers) {
        verified[p].matches.push_back(kept[i]);
      }
    },
    [&](std::size_t done) { report(BuildStage::kMatching, done, pairs.size()); });
  double inlier_sum = 0.0;
  for (const auto & v : verified) {
    if (!v.matches.empty()) {
      ++stats.pairs_verified;
      inlier_sum += static_cast<double>(v.matches.size());
    }
  }
  stats.mean_pair_inliers = stats.pairs_verified > 0 ? inlier_sum / stats.pairs_verified : 0.0;
  stats.stage_seconds[static_cast<int>(BuildStage::kMatching)] = watch.lap();

  // --- Tracks ----------------------------------------------------------------------------------
  report(BuildStage::kTracks, 0, 1);
  std::vector<int> keypoint_counts(n);
  for (std::size_t i = 0; i < n; ++i) {
    keypoint_counts[i] = static_cast<int>(features[i].keypoints.size());
  }
  TrackStats track_stats;
  const auto tracks =
    buildTracks(keypoint_counts, verified, config.triangulation.min_track_length, &track_stats);
  verified.clear();
  stats.tracks = static_cast<int>(tracks.size());
  stats.track_conflicts = track_stats.conflicts;
  report(BuildStage::kTracks, 1, 1);
  stats.stage_seconds[static_cast<int>(BuildStage::kTracks)] = watch.lap();

  // --- Triangulation with the prior poses ------------------------------------------------------
  TriangulationParams tri = config.triangulation;
  if (tri.max_reprojection_px <= 0.0) {
    tri.max_reprojection_px = kAutoTriangulationFraction * camera.model.width();
  }
  tri.focal_px = focal;
  stats.triangulation_threshold_px = tri.max_reprojection_px;

  // Triangulates every track against the given camera poses; returns the accepted landmarks.
  const auto triangulateAll = [&](
                                const std::vector<Pose3> & cameras_map,
                                const TriangulationParams & params, bool count_rejections) {
    std::vector<Pose3> T_cam_map(n);
    for (std::size_t i = 0; i < n; ++i) {
      T_cam_map[i] = cameras_map[i].inverse();
    }
    std::vector<TriangulationResult> results(tracks.size());
    chunked(
      pool, tracks.size(),
      [&](std::size_t t) {
        std::vector<TrackObservation> observations;
        observations.reserve(tracks[t].size());
        for (const TrackNode & node : tracks[t]) {
          observations.push_back({node.keyframe, normalized[node.keyframe][node.keypoint]});
        }
        results[t] = triangulateTrack(T_cam_map, observations, params);
      },
      [&](std::size_t done) { report(BuildStage::kTriangulation, done, tracks.size()); });
    std::vector<Landmark> landmarks;
    for (std::size_t t = 0; t < tracks.size(); ++t) {
      const TriangulationResult & r = results[t];
      if (count_rejections) {
        switch (r.status) {
          case TriangulationStatus::kOk:
            ++stats.triangulated;
            break;
          case TriangulationStatus::kSmallAngle:
            ++stats.rejected_small_angle;
            break;
          case TriangulationStatus::kDepthOutOfRange:
            ++stats.rejected_depth;
            break;
          case TriangulationStatus::kBelowFloor:
            ++stats.rejected_below_floor;
            break;
          case TriangulationStatus::kTooFewObservations:
            ++stats.rejected_reprojection;
            break;
        }
      }
      if (r.status != TriangulationStatus::kOk) {
        continue;
      }
      Landmark landmark;
      landmark.position = r.position;
      landmark.mean_reprojection_px = r.mean_reprojection_px;
      landmark.max_angle_deg = r.max_angle_deg;
      for (int i : r.inliers) {
        landmark.observations.push_back(tracks[t][i]);
      }
      landmarks.push_back(std::move(landmark));
    }
    return landmarks;
  };
  report(BuildStage::kTriangulation, 0, tracks.size());
  std::vector<Landmark> landmarks = triangulateAll(prior_cameras, tri, true);
  stats.stage_seconds[static_cast<int>(BuildStage::kTriangulation)] = watch.lap();

  // --- Pose-prior bundle adjustment and re-triangulation ---------------------------------------
  // The priors and odometry of the pose graph, the landmarks added, starting from its poses.
  std::vector<Pose2> refined = start;
  report(BuildStage::kBundleAdjustment, 0, 1);
  if (config.ba.enabled && !landmarks.empty()) {
    stats.ba_run = true;
    for (std::size_t i = 0; i < n; ++i) {
      ba_keyframes[i].initial = start[i];
    }
    std::vector<Vec3> positions;
    std::vector<BaObservation> ba_observations;
    for (std::size_t l = 0; l < landmarks.size(); ++l) {
      positions.push_back(landmarks[l].position);
      for (const TrackNode & node : landmarks[l].observations) {
        ba_observations.push_back(
          {node.keyframe, static_cast<int>(l), normalized[node.keyframe][node.keypoint]});
      }
    }
    const BaResult ba = adjustPosePriors(ba_keyframes, positions, ba_observations, config.ba);
    stats.ba_usable = ba.usable;
    stats.ba_iterations = ba.iterations;
    stats.ba_initial_cost = ba.initial_cost;
    stats.ba_final_cost = ba.final_cost;
    if (!ba.usable) {
      stats.warnings.push_back("bundle adjustment failed (" + ba.message + "); priors kept");
    } else {
      std::vector<Pose3> refined_cameras(n);
      double shift_sum = 0.0;
      for (std::size_t i = 0; i < n; ++i) {
        refined[i] = ba.poses[i];
        // The adjustment's own shift, from where it started.
        const Pose2 & before = start[i];
        const Vec2 shift(refined[i].x - before.x, refined[i].y - before.y);
        shift_sum += shift.norm();
        stats.ba_mean_shift_vector += shift;
        stats.ba_max_shift_m = std::max(stats.ba_max_shift_m, shift.norm());
        stats.ba_max_shift_yaw_deg = std::max(
          stats.ba_max_shift_yaw_deg,
          std::abs(normalizeAngle(refined[i].yaw - before.yaw)) * 180.0 / M_PI);
        if (ba.at_bound[i] != 0) {
          flags[i] |= db::KeyframeRecord::kFlagBaShiftAtBound;
          ++stats.ba_keyframes_at_bound;
        }
        refined_cameras[i] = cameraPose(refined[i], input.keyframes[i].T_base_cam);
      }
      stats.ba_mean_shift_m = shift_sum / static_cast<double>(n);
      stats.ba_mean_shift_vector /= static_cast<double>(n);
      if (stats.ba_mean_shift_vector.norm() > 0.02) {
        stats.warnings.push_back(
          "bundle adjustment moved the keyframes by " +
          std::to_string(stats.ba_mean_shift_vector.norm()) +
          " m on average in one direction: check the recording's localization");
      }
      if (stats.ba_keyframes_at_bound > 0) {
        stats.warnings.push_back(
          std::to_string(stats.ba_keyframes_at_bound) +
          " keyframe(s) reached the BA shift bound (flagged)");
      }
      TriangulationParams refilter = tri;
      refilter.max_reprojection_px = config.ba_max_reprojection_px > 0.0
                                       ? config.ba_max_reprojection_px
                                       : kAutoRefilterFactor * tri.max_reprojection_px;
      landmarks = triangulateAll(refined_cameras, refilter, false);
    }
  }
  report(BuildStage::kBundleAdjustment, 1, 1);
  stats.stage_seconds[static_cast<int>(BuildStage::kBundleAdjustment)] = watch.lap();

  // --- Final statistics, pruning, covisibility -------------------------------------------------
  std::vector<Pose3> final_cameras(n);
  std::vector<Pose3> final_T_cam_map(n);
  for (std::size_t i = 0; i < n; ++i) {
    final_cameras[i] = cameraPose(refined[i], input.keyframes[i].T_base_cam);
    final_T_cam_map[i] = final_cameras[i].inverse();
  }
  double squared_sum = 0.0;
  for (const Landmark & landmark : landmarks) {
    for (const TrackNode & node : landmark.observations) {
      const double e = reprojectionPx(
        final_T_cam_map[node.keyframe], landmark.position, normalized[node.keyframe][node.keypoint],
        focal);
      squared_sum += e * e;
    }
    stats.observations += static_cast<int>(landmark.observations.size());
  }
  stats.landmarks = static_cast<int>(landmarks.size());
  stats.mean_track_length =
    stats.landmarks > 0 ? static_cast<double>(stats.observations) / stats.landmarks : 0.0;
  stats.reprojection_rms_px =
    stats.observations > 0 ? std::sqrt(squared_sum / stats.observations) : 0.0;
  if (stats.landmarks == 0) {
    stats.warnings.emplace_back("no landmark could be triangulated");
  }

  // Keypoint index remapping: keep only observed keypoints when pruning.
  std::vector<std::vector<int>> keep_index(n);
  for (std::size_t i = 0; i < n; ++i) {
    keep_index[i].assign(features[i].keypoints.size(), config.prune_unobserved_features ? -1 : 0);
    if (!config.prune_unobserved_features) {
      std::iota(keep_index[i].begin(), keep_index[i].end(), 0);
    }
  }
  if (config.prune_unobserved_features) {
    for (const Landmark & landmark : landmarks) {
      for (const TrackNode & node : landmark.observations) {
        keep_index[node.keyframe][node.keypoint] = 0;
      }
    }
    for (std::size_t i = 0; i < n; ++i) {
      int next = 0;
      for (int & index : keep_index[i]) {
        if (index == 0) {
          index = next++;
        }
      }
    }
  }
  std::map<std::pair<int, int>, int> covisibility;
  for (const Landmark & landmark : landmarks) {
    const auto & obs = landmark.observations;
    for (std::size_t a = 0; a < obs.size(); ++a) {
      for (std::size_t b = a + 1; b < obs.size(); ++b) {
        ++covisibility[{
          std::min(obs[a].keyframe, obs[b].keyframe), std::max(obs[a].keyframe, obs[b].keyframe)}];
      }
    }
  }

  // --- Database --------------------------------------------------------------------------------
  report(BuildStage::kWriting, 0, 1);
  // Written under a temporary name and renamed when complete, so a failed build leaves nothing.
  const std::string partial = db_path + ".partial";
  const auto removePartial = [&partial] {
    std::error_code ignored;
    std::filesystem::remove(partial, ignored);
    std::filesystem::remove(partial + "-journal", ignored);
  };
  removePartial();
  try {
    db::Database database = db::Database::create(partial);
    db::Database::Transaction transaction(database);
    database.setMeta("landfall_version", version());
    database.setMeta("created_utc", utcNow());
    database.setMeta("builder_config", canonicalYaml(file));

    db::MapRecord map;
    map.yaml_sha256 = input.map.yaml_sha256;
    map.image_sha256 = input.map.image_sha256;
    map.grid_sha256 = gridSha256(input.map.grid);
    map.resolution = input.map.grid.resolution;
    map.origin_x = input.map.grid.origin.x;
    map.origin_y = input.map.grid.origin.y;
    map.origin_yaw = input.map.grid.origin.yaw;
    map.width = input.map.grid.width;
    map.height = input.map.grid.height;
    map.yaml_text = input.map.yaml_text;
    map.grid = encodeGridPng(input.map.grid);
    database.insertMap(map);

    db::CameraRecord camera_record;
    camera_record.frame_id = camera.frame_id;
    camera_record.model = toString(camera.model.model());
    camera_record.width = camera.model.width();
    camera_record.height = camera.model.height();
    camera_record.intrinsics = {
      camera.model.fx(), camera.model.fy(), camera.model.cx(), camera.model.cy()};
    camera_record.distortion = camera.model.coefficients();
    camera_record.T_base_cam = camera.T_base_cam;
    const int64_t camera_id = database.insertCamera(camera_record);

    std::vector<int64_t> session_ids;
    session_ids.reserve(input.sessions.size());
    for (const BuildSession & s : input.sessions) {
      session_ids.push_back(
        database.insertSession({0, s.name, s.source, s.pose_source, s.started_ns, s.notes}));
    }

    const auto addBackend = [&](const std::string & role, const BackendId & id) {
      return database.insertBackend(
        {0, role, id.name, id.version, id.model_sha256, id.config, configSha256(id)});
    };
    const int64_t local_id = addBackend("local", extractor->id());
    const int64_t global_id =
      addBackend("global", use_bow ? bow_backend->id() : learned_global->id());
    addBackend("matcher", matcher->id());
    if (use_bow) {
      const auto vocabulary_data = vocabulary->serialize();
      BackendId vocabulary_backend{
        "vocab_tree", "1",
        "branching: " + std::to_string(vocabulary_params.branching) +
          "\ndepth: " + std::to_string(vocabulary_params.depth) + "\niterations: " +
          std::to_string(vocabulary_params.iterations) + "\nmax_training_descriptors: " +
          std::to_string(vocabulary_params.max_training_descriptors) +
          "\nseed: " + std::to_string(vocabulary_params.seed) + "\n",
        ""};
      const int64_t vocabulary_id = addBackend("vocabulary", vocabulary_backend);
      database.insertVocabulary(
        {vocabulary_id, vocabulary_params.branching, vocabulary_params.depth, vocabulary_data});
    }

    std::vector<int64_t> keyframe_ids(n);
    Mat3 unknown_covariance;
    unknown_covariance.setConstant(std::numeric_limits<double>::quiet_NaN());
    for (std::size_t i = 0; i < n; ++i) {
      const BuildKeyframe & k = input.keyframes[i];
      db::KeyframeRecord record;
      record.session_id = session_ids[k.session];
      record.camera_id = camera_id;
      record.stamp_ns = k.stamp_ns;
      record.prior = k.prior;
      record.prior_covariance = k.covariance ? *k.covariance : unknown_covariance;
      record.refined = refined[i];
      record.T_map_cam = final_cameras[i];
      record.flags = flags[i];
      record.quality_json = k.quality_json;
      record.thumbnail_jpeg = thumbnails[i];
      keyframe_ids[i] = database.insertKeyframe(record);
    }
    for (std::size_t i = 0; i < n; ++i) {
      Features stored;
      stored.type = features[i].type;
      if (config.prune_unobserved_features) {
        std::vector<int> rows;
        for (std::size_t j = 0; j < keep_index[i].size(); ++j) {
          if (keep_index[i][j] >= 0) {
            rows.push_back(static_cast<int>(j));
            stored.keypoints.push_back(features[i].keypoints[j]);
          }
        }
        stored.descriptors = cv::Mat(
          static_cast<int>(rows.size()), features[i].descriptors.cols,
          features[i].descriptors.type());
        for (std::size_t r = 0; r < rows.size(); ++r) {
          features[i].descriptors.row(rows[r]).copyTo(stored.descriptors.row(static_cast<int>(r)));
        }
      } else {
        stored = features[i];
      }
      database.insertLocalFeatures(db::encodeLocalFeatures(keyframe_ids[i], local_id, stored));
      database.insertGlobalDescriptor(db::encodeGlobalDescriptor(
        keyframe_ids[i], global_id, globalDescriptor(i), config.global_dtype));
    }
    for (const Landmark & landmark : landmarks) {
      // Representative descriptor: the medoid of the track's descriptors.
      const auto & obs = landmark.observations;
      std::size_t medoid = 0;
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t a = 0; a < obs.size(); ++a) {
        double sum = 0.0;
        for (std::size_t b = 0; b < obs.size(); ++b) {
          if (a != b) {
            sum += descriptorDistance(
              features[obs[a].keyframe].descriptors.row(obs[a].keypoint),
              features[obs[b].keyframe].descriptors.row(obs[b].keypoint), descriptor_type);
          }
        }
        if (sum < best) {
          best = sum;
          medoid = a;
        }
      }
      db::LandmarkRecord record;
      record.position = landmark.position;
      record.track_length = static_cast<int>(obs.size());
      record.mean_reprojection_px = landmark.mean_reprojection_px;
      record.max_angle_deg = landmark.max_angle_deg;
      record.descriptor =
        db::descriptorRowBytes(features[obs[medoid].keyframe].descriptors, obs[medoid].keypoint);
      const int64_t landmark_id = database.insertLandmark(record);
      for (const TrackNode & node : obs) {
        database.insertObservation(
          {keyframe_ids[node.keyframe], keep_index[node.keyframe][node.keypoint], landmark_id});
      }
    }
    for (const auto & [pair, shared] : covisibility) {
      if (shared >= config.covisibility_min_shared) {
        database.insertCovisibility({keyframe_ids[pair.first], keyframe_ids[pair.second], shared});
      }
    }
    stats.stage_seconds[static_cast<int>(BuildStage::kWriting)] = watch.lap();
    database.setMeta("build_report", toJson(stats));
    database.setMeta("content_sha256", database.contentSha256());
    transaction.commit();
  } catch (...) {
    removePartial();
    throw;
  }
  std::filesystem::rename(partial, db_path);
  report(BuildStage::kWriting, 1, 1);
  return stats;
}

std::string toJson(const BuildStats & s)
{
  JsonWriter json;
  json.beginObject()
    .field("keyframes", s.keyframes)
    .field("sessions", s.sessions)
    .field("pairs_candidate", s.pairs_candidate)
    .field("pairs_verified", s.pairs_verified)
    .field("mean_pair_inliers", s.mean_pair_inliers)
    .field("tracks", s.tracks)
    .field("track_conflicts", s.track_conflicts)
    .field("triangulated", s.triangulated)
    .field("rejected_small_angle", s.rejected_small_angle)
    .field("rejected_depth", s.rejected_depth)
    .field("rejected_below_floor", s.rejected_below_floor)
    .field("rejected_reprojection", s.rejected_reprojection)
    .field("landmarks", s.landmarks)
    .field("observations", s.observations)
    .field("mean_track_length", s.mean_track_length)
    .field("reprojection_rms_px", s.reprojection_rms_px)
    .field("triangulation_threshold_px", s.triangulation_threshold_px)
    .field("ba_run", s.ba_run)
    .field("ba_usable", s.ba_usable)
    .field("ba_iterations", s.ba_iterations)
    .field("ba_initial_cost", s.ba_initial_cost)
    .field("ba_final_cost", s.ba_final_cost)
    .field("ba_mean_shift_m", s.ba_mean_shift_m)
    .field("ba_max_shift_m", s.ba_max_shift_m)
    .field("ba_max_shift_yaw_deg", s.ba_max_shift_yaw_deg)
    .field("ba_mean_shift_x_m", s.ba_mean_shift_vector.x())
    .field("ba_mean_shift_y_m", s.ba_mean_shift_vector.y())
    .field("ba_keyframes_at_bound", s.ba_keyframes_at_bound)
    .field("poses_scan_matched", s.poses_scan_matched)
    .field("poses_scan_unmatched", s.poses_scan_unmatched)
    .field("poses_scan_contradicted", s.poses_scan_contradicted)
    .field("poses_odometry_links", s.poses_odometry_links)
    .field("poses_mean_correction_m", s.poses_mean_correction_m)
    .field("poses_max_correction_m", s.poses_max_correction_m)
    .field("poses_max_correction_yaw_deg", s.poses_max_correction_yaw_deg);
  json.key("stage_seconds").beginObject();
  for (int i = 0; i < kBuildStageCount && i < static_cast<int>(s.stage_seconds.size()); ++i) {
    json.field(toString(static_cast<BuildStage>(i)), s.stage_seconds[i]);
  }
  json.endObject();
  json.key("warnings").beginArray();
  for (const auto & warning : s.warnings) {
    json.value(warning);
  }
  json.endArray().endObject();
  return json.str();
}

}  // namespace landfall
