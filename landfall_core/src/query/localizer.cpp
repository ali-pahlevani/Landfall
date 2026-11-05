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

#include "landfall/query/localizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace landfall
{

namespace
{

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

std::unique_ptr<IMatcher> makeMatcher(DescriptorType type, const LocalizerParams & params)
{
  if (type == DescriptorType::kBinary) {
    return std::make_unique<RatioMatcher>(params.ratio);
  }
  return std::make_unique<MutualNearestMatcher>(params.mnn);
}

cv::Mat toGray(const cv::Mat & image)
{
  if (image.channels() == 1) {
    return image;
  }
  cv::Mat gray;
  cv::cvtColor(image, gray, image.channels() == 4 ? cv::COLOR_BGRA2GRAY : cv::COLOR_BGR2GRAY);
  return gray;
}

int find(std::vector<int> & parent, int i)
{
  while (parent[i] != i) {
    parent[i] = parent[parent[i]];
    i = parent[i];
  }
  return i;
}

double toRad(double degrees) { return degrees * M_PI / 180.0; }

}  // namespace

Localizer::Localizer(std::shared_ptr<const LocalizationMap> map, const LocalizerParams & params)
: map_(std::move(map)), params_(params), pool_(params.threads)
{
  if (!map_) {
    throw std::invalid_argument("Localizer: no map");
  }
  matcher_ = makeMatcher(map_->descriptorType(), params_);
}

void Localizer::setParams(const LocalizerParams & params)
{
  params_ = params;
  matcher_ = makeMatcher(map_->descriptorType(), params_);
}

std::vector<Localizer::Place> Localizer::groupPlaces(
  const std::vector<RetrievalCandidate> & retrieved) const
{
  const int n = static_cast<int>(retrieved.size());
  std::vector<int> parent(static_cast<std::size_t>(n));
  std::iota(parent.begin(), parent.end(), 0);
  const double min_cos = std::cos(toRad(params_.place_max_angle_deg));
  for (int a = 0; a < n; ++a) {
    const MapKeyframe * ka = map_->keyframe(retrieved[a].id);
    for (int b = a + 1; b < n; ++b) {
      const MapKeyframe * kb = map_->keyframe(retrieved[b].id);
      if (ka == nullptr || kb == nullptr) {
        continue;
      }
      bool same_place = map_->covisibility(ka->id, kb->id) >= params_.place_min_shared;
      if (!same_place) {
        const double distance = (ka->T_map_cam.translation() - kb->T_map_cam.translation()).norm();
        const double cos_angle = ka->T_map_cam.linear().col(2).dot(kb->T_map_cam.linear().col(2));
        same_place = distance <= params_.place_max_distance && cos_angle >= min_cos;
      }
      if (same_place) {
        // The better-ranked root wins, so a place is ordered by its best rank.
        const int ra = find(parent, a);
        const int rb = find(parent, b);
        parent[std::max(ra, rb)] = std::min(ra, rb);
      }
    }
  }
  std::map<int, std::size_t> place_of_root;
  std::vector<Place> places;
  for (int i = 0; i < n; ++i) {
    const int root = find(parent, i);
    auto it = place_of_root.find(root);
    if (it == place_of_root.end()) {
      it = place_of_root.emplace(root, places.size()).first;
      places.emplace_back();
    }
    places[it->second].keyframe_ids.push_back(retrieved[i].id);
  }
  return places;
}

Hypothesis Localizer::solvePlace(
  const Place & place, const QueryFrame & frame, const Features & features) const
{
  Hypothesis h;
  h.keyframe_ids = place.keyframe_ids;

  // Best database match of every query feature, then one query feature per landmark.
  struct Candidate
  {
    int64_t landmark = -1;
    float distance = 0.0F;
  };
  std::map<int, Candidate> by_query;
  for (const int64_t id : place.keyframe_ids) {
    const auto stored = map_->features(id);
    for (const Match & m : matcher_->match(features, stored->features)) {
      const int64_t landmark = stored->landmarks[static_cast<std::size_t>(m.train)];
      if (landmark < 0 || map_->landmark(landmark) == nullptr) {
        continue;
      }
      auto [it, inserted] = by_query.try_emplace(m.query, Candidate{landmark, m.distance});
      if (
        !inserted && (m.distance < it->second.distance ||
                      (m.distance == it->second.distance && landmark < it->second.landmark))) {
        it->second = {landmark, m.distance};
      }
    }
  }
  std::map<int64_t, std::pair<int, float>> by_landmark;
  for (const auto & [query, c] : by_query) {
    auto [it, inserted] = by_landmark.try_emplace(c.landmark, query, c.distance);
    if (!inserted && c.distance < it->second.second) {
      it->second = {query, c.distance};
    }
  }
  std::vector<std::pair<int, int64_t>> pairs;  // (query feature, landmark), by query feature
  pairs.reserve(by_landmark.size());
  for (const auto & [landmark, q] : by_landmark) {
    pairs.emplace_back(q.first, landmark);
  }
  std::sort(pairs.begin(), pairs.end());

  Correspondences correspondences;
  std::vector<Vec2> pixels;
  for (const auto & [query, landmark] : pairs) {
    const Keypoint & k = features.keypoints[static_cast<std::size_t>(query)];
    const Vec2 pixel(k.x, k.y);
    Vec2 normalized;
    if (!frame.camera.undistort(pixel, &normalized)) {
      continue;
    }
    correspondences.normalized.push_back(normalized);
    correspondences.points.push_back(*map_->landmark(landmark));
    pixels.push_back(pixel);
  }
  h.correspondences = static_cast<int>(pixels.size());
  const int min_inliers = params_.sanity.min_inliers;
  if (h.correspondences < min_inliers) {
    std::ostringstream reason;
    reason << h.correspondences << " 2D-3D matches < " << min_inliers;
    h.rejection = reason.str();
    return h;
  }

  PnpOptions options;
  options.threshold_px =
    params_.pnp_threshold_px > 0.0 ? params_.pnp_threshold_px : 0.01 * frame.camera.width();
  options.focal_px = frame.camera.focal();
  options.max_iterations = params_.pnp_max_iterations;
  options.success_prob = params_.pnp_success_prob;
  options.seed = params_.seed;
  const Vec3 gravity_cam = frame.T_base_cam.linear().transpose() * frame.gravity.gravity_base;
  const auto solution =
    solvePnp(params_.pnp, correspondences, gravity_cam.normalized(), options, min_inliers);
  const int inliers = solution ? static_cast<int>(solution->inliers.size()) : 0;
  if (!solution || inliers < min_inliers) {
    std::ostringstream reason;
    reason << "pose with " << inliers << " inliers < " << min_inliers;
    h.rejection = reason.str();
    return h;
  }
  h.solved = true;
  h.stage = Stage::kSolved;
  h.method = solution->method;
  h.T_map_cam = solution->T_cam_map.inverse();
  std::vector<char> is_inlier(pixels.size(), 0);
  for (const int i : solution->inliers) {
    is_inlier[static_cast<std::size_t>(i)] = 1;
  }
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    (is_inlier[i] != 0 ? h.inlier_pixels : h.outlier_pixels).push_back(pixels[i]);
  }

  SanityParams sanity = params_.sanity;
  if (sanity.max_mean_reprojection_px <= 0.0) {
    sanity.max_mean_reprojection_px = 0.5 * options.threshold_px;
  }
  SanityInput input;
  input.T_map_cam = h.T_map_cam;
  input.T_base_cam = frame.T_base_cam;
  input.gravity = frame.gravity;
  input.num_correspondences = h.correspondences;
  input.inlier_pixels = h.inlier_pixels;
  input.mean_reprojection_px = solution->mean_reprojection_px;
  input.image_size = cv::Size(frame.camera.width(), frame.camera.height());
  h.sanity = checkSanity(input, sanity, &map_->field());
  if (h.sanity.passed) {
    h.stage = Stage::kSanityPassed;
  } else {
    h.rejection = h.sanity.reason;
  }
  return h;
}

FrameResult Localizer::localize(const QueryFrame & frame)
{
  Stopwatch watch;
  Stopwatch total;
  FrameResult result;
  cv::Mat gray;
  Features extracted;
  if (!frame.features) {
    if (frame.image.empty()) {
      throw std::invalid_argument("Localizer: empty query image");
    }
    gray = toGray(frame.image);
    extracted = map_->localFeatures().extract(gray);
  }
  const Features & features = frame.features ? *frame.features : extracted;
  result.num_features = static_cast<int>(features.keypoints.size());
  result.timings.features_s = watch.lap();

  const GlobalDescriptor global = map_->globalDescriptor().compute(gray, &features);
  result.retrieved = map_->retrievalIndex().search(global, params_.top_k);
  const std::vector<Place> places = groupPlaces(result.retrieved);
  result.timings.retrieval_s = watch.lap();

  std::vector<Hypothesis> hypotheses(places.size());
  pool_.parallelFor(
    places.size(), [&](std::size_t i) { hypotheses[i] = solvePlace(places[i], frame, features); });
  // Best first: deeper stage, more inliers, more matches, better retrieval rank.
  std::vector<std::size_t> order(hypotheses.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&hypotheses](std::size_t a, std::size_t b) {
    const Hypothesis & x = hypotheses[a];
    const Hypothesis & y = hypotheses[b];
    if (x.stage != y.stage) {
      return x.stage > y.stage;
    }
    if (x.sanity.num_inliers != y.sanity.num_inliers) {
      return x.sanity.num_inliers > y.sanity.num_inliers;
    }
    return x.correspondences > y.correspondences;
  });
  for (const std::size_t i : order) {
    result.hypotheses.push_back(std::move(hypotheses[i]));
    result.deepest_stage = std::max(result.deepest_stage, result.hypotheses.back().stage);
  }

  std::vector<int> valid;
  std::vector<Pose2> poses;
  std::vector<double> scores;
  for (int i = 0; i < static_cast<int>(result.hypotheses.size()); ++i) {
    const Hypothesis & h = result.hypotheses[static_cast<std::size_t>(i)];
    if (h.stage == Stage::kSanityPassed) {
      valid.push_back(i);
      poses.push_back(h.sanity.pose);
      scores.push_back(h.sanity.num_inliers);
    }
  }
  result.clusters = clusterPoses(poses, scores, params_.clustering);
  for (std::size_t c = 0; c < result.clusters.size(); ++c) {
    for (int & member : result.clusters[c].members) {
      member = valid[static_cast<std::size_t>(member)];
      result.hypotheses[static_cast<std::size_t>(member)].cluster = static_cast<int>(c);
    }
  }
  result.ambiguity =
    checkAmbiguity(result.clusters, {params_.max_score_ratio, params_.sanity.min_inliers});
  if (!result.clusters.empty()) {
    result.best = result.clusters.front().members.front();
    result.pose = result.clusters.front().pose;
  }
  result.timings.solve_s = watch.lap();
  result.timings.total_s = total.lap();
  return result;
}

ScoreSummary summarizeScores(const std::vector<FrameResult> & frames)
{
  ScoreSummary s;
  for (const FrameResult & f : frames) {
    s.ambiguity_ratio = std::max(s.ambiguity_ratio, f.ambiguity.score_ratio);
    const Hypothesis * h = nullptr;
    if (f.best >= 0) {
      h = &f.hypotheses[static_cast<std::size_t>(f.best)];
    } else if (!f.hypotheses.empty() && f.hypotheses.front().solved) {
      h = &f.hypotheses.front();
    }
    if (h == nullptr) {
      continue;
    }
    const SanityResult & m = h->sanity;
    if (!s.any) {
      s.num_inliers = m.num_inliers;
      s.inlier_ratio = m.inlier_ratio;
      s.inlier_coverage = m.inlier_coverage;
    }
    s.any = true;
    s.num_inliers = std::min(s.num_inliers, m.num_inliers);
    s.inlier_ratio = std::min(s.inlier_ratio, m.inlier_ratio);
    s.inlier_coverage = std::min(s.inlier_coverage, m.inlier_coverage);
    s.mean_reprojection_px = std::max(s.mean_reprojection_px, m.mean_reprojection_px);
    s.height_error_m = std::max(s.height_error_m, m.height_error_m);
    s.tilt_error_deg = std::max(s.tilt_error_deg, m.tilt_error_deg);
  }
  return s;
}

}  // namespace landfall
