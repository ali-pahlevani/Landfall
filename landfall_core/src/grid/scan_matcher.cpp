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

#include "landfall/grid/scan_matcher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

namespace landfall
{

namespace
{

constexpr int kCoarseBeams = 180;           // beams scored by the coarse search
constexpr double kCoarseYawStepDeg = 0.5;   // coarse heading step; the position step is one cell
constexpr double kTiePenalty = 0.02;        // score lost at the window's corner (near-ties only)
constexpr double kMinNormalGradient = 0.3;  // weaker distance gradients give no surface normal
// Beam fits multiplied before taking their logarithm. Each is at least 1 / (1 + (cap / sigma)^2)
// with the field's distance cap (1 m by default): 1e-6 for a 1 mm sigma, so a product of 16 is
// still far from the smallest double.
constexpr std::size_t kProductChunk = 16;

double toRad(double degrees) { return degrees * M_PI / 180.0; }

// The likelihood field's distances: per cell, or interpolated between cell centres.
class Distances
{
public:
  explicit Distances(const LikelihoodField & field)
  : grid_(field.grid()), values_(field.distances()), max_(field.maxDistance())
  {
  }

  double cell(int64_t col, int64_t row) const
  {
    if (col < 0 || row < 0 || col >= grid_.width || row >= grid_.height) {
      return max_;
    }
    return values_
      [static_cast<std::size_t>(row) * static_cast<std::size_t>(grid_.width) +
       static_cast<std::size_t>(col)];
  }

  // Bilinear between the four nearest cell centres; cells outside the grid are max_distance away.
  double interpolated(double x, double y) const
  {
    const double u = (x - grid_.origin.x) / grid_.resolution - 0.5;
    const double v = (y - grid_.origin.y) / grid_.resolution - 0.5;
    if (!std::isfinite(u) || !std::isfinite(v)) {
      return max_;
    }
    const double u0 = std::floor(u);
    const double v0 = std::floor(v);
    const double a = u - u0;
    const double b = v - v0;
    const auto c = static_cast<int64_t>(u0);
    const auto r = static_cast<int64_t>(v0);
    return (1.0 - a) * (1.0 - b) * cell(c, r) + a * (1.0 - b) * cell(c + 1, r) +
           (1.0 - a) * b * cell(c, r + 1) + a * b * cell(c + 1, r + 1);
  }

  const OccupancyGrid & grid() const { return grid_; }

private:
  const OccupancyGrid & grid_;
  const std::vector<float> & values_;
  double max_;
};

// Fit of one beam: 1 on an obstacle, falling off over beam_sigma (heavy-tailed, so beams on
// unmapped objects cost little).
double kernel(double distance, double inv_sigma)
{
  const double t = distance * inv_sigma;
  return 1.0 / (1.0 + t * t);
}

Vec2 transformPoint(const Pose2 & pose, double c, double s, const Vec2 & e)
{
  return {pose.x + c * e.x() - s * e.y(), pose.y + s * e.x() + c * e.y()};
}

// Mean beam fit at a pose on the interpolated field, less the tie-breaking pull to the guess.
double fineScore(
  const Distances & d, const std::vector<Vec2> & endpoints, const Pose2 & pose, double inv_sigma,
  double penalty)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  double sum = 0.0;
  for (const Vec2 & e : endpoints) {
    const Vec2 p = transformPoint(pose, c, s, e);
    sum += kernel(d.interpolated(p.x(), p.y()), inv_sigma);
  }
  return sum / static_cast<double>(endpoints.size()) - penalty;
}

// Log-likelihood of every beam at a pose: -log(1 + (d / sigma)^2) / 2 each (Cauchy errors).
double logLikelihood(
  const Distances & d, const std::vector<Vec2> & endpoints, const Pose2 & pose, double inv_sigma)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  double sum = 0.0;
  for (const Vec2 & e : endpoints) {
    const Vec2 p = transformPoint(pose, c, s, e);
    const double t = d.interpolated(p.x(), p.y()) * inv_sigma;
    sum -= 0.5 * std::log1p(t * t);
  }
  return sum;
}

double hitRatio(
  const Distances & d, const std::vector<Vec2> & endpoints, const Pose2 & pose, double tolerance)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  int hits = 0;
  for (const Vec2 & e : endpoints) {
    const Vec2 p = transformPoint(pose, c, s, e);
    hits += d.interpolated(p.x(), p.y()) <= tolerance ? 1 : 0;
  }
  return static_cast<double>(hits) / static_cast<double>(endpoints.size());
}

}  // namespace

ScanMatch matchScan(
  const LikelihoodField & field, const std::vector<Vec2> & endpoints, const Pose2 & guess,
  const ScanMatchParams & params)
{
  ScanMatch result;
  result.pose = guess;
  result.beams = static_cast<int>(endpoints.size());
  const OccupancyGrid & grid = field.grid();
  if (grid.resolution <= 0.0 || grid.width <= 0 || grid.height <= 0) {
    result.reason = "no map";
    return result;
  }
  if (result.beams < params.min_beams) {
    result.reason =
      std::to_string(result.beams) + " valid beams < " + std::to_string(params.min_beams);
    return result;
  }
  const Distances d(field);
  const double inv_sigma = 1.0 / params.beam_sigma;
  const double yaw_window = toRad(params.search_yaw_deg);
  const auto penalty = [&](const Pose2 & p) {
    const double r = std::hypot(p.x - guess.x, p.y - guess.y) / std::max(params.search_xy, 1e-6);
    const double a = std::abs(normalizeAngle(p.yaw - guess.yaw)) / std::max(yaw_window, 1e-9);
    return 0.5 * kTiePenalty * (r * r + a * a);
  };
  result.guess_hit_ratio = hitRatio(d, endpoints, guess, params.hit_tolerance);

  // --- Coarse: one-cell translation steps by shifting cell indices, 0.5 deg heading steps ------
  const std::size_t stride = std::max<std::size_t>(1, endpoints.size() / kCoarseBeams);
  std::vector<Vec2> coarse;
  for (std::size_t i = 0; i < endpoints.size(); i += stride) {
    coarse.push_back(endpoints[i]);
  }
  const double res = grid.resolution;
  const int cells = std::max(0, static_cast<int>(std::lround(params.search_xy / res)));
  const int yaw_steps =
    std::max(0, static_cast<int>(std::lround(params.search_yaw_deg / kCoarseYawStepDeg)));
  double best_score = -1.0;
  int best_i = 0;
  int best_j = 0;
  int best_k = 0;
  // Every pose of the window with its log-likelihood: the window covariance below.
  struct Scored
  {
    int i;
    int j;
    int k;
    double log_likelihood;
  };
  std::vector<Scored> window;
  window.reserve(
    static_cast<std::size_t>(2 * cells + 1) * static_cast<std::size_t>(2 * cells + 1) *
    static_cast<std::size_t>(2 * yaw_steps + 1));
  std::vector<int64_t> cols(coarse.size());
  std::vector<int64_t> rows(coarse.size());
  for (int k = -yaw_steps; k <= yaw_steps; ++k) {
    const double yaw = guess.yaw + toRad(k * kCoarseYawStepDeg);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    for (std::size_t b = 0; b < coarse.size(); ++b) {
      const Vec2 p = transformPoint({guess.x, guess.y, yaw}, c, s, coarse[b]);
      cols[b] = static_cast<int64_t>(std::floor((p.x() - grid.origin.x) / res));
      rows[b] = static_cast<int64_t>(std::floor((p.y() - grid.origin.y) / res));
    }
    for (int j = -cells; j <= cells; ++j) {
      for (int i = -cells; i <= cells; ++i) {
        double sum = 0.0;
        double log_sum = 0.0;
        double product = 1.0;
        for (std::size_t b = 0; b < coarse.size(); ++b) {
          const double fit = kernel(d.cell(cols[b] + i, rows[b] + j), inv_sigma);
          sum += fit;
          product *= fit;
          if ((b + 1) % kProductChunk == 0) {
            log_sum += std::log(product);
            product = 1.0;
          }
        }
        log_sum += std::log(product);
        const Pose2 candidate{guess.x + i * res, guess.y + j * res, yaw};
        // Beams with Cauchy errors: -log(1 + (d / sigma)^2) / 2 each, the log of the fit's root.
        window.push_back({i, j, k, 0.5 * log_sum});
        const double score = sum / static_cast<double>(coarse.size()) - penalty(candidate);
        if (score > best_score) {
          best_score = score;
          best_i = i;
          best_j = j;
          best_k = k;
        }
      }
    }
  }
  if (
    (cells > 0 && (std::abs(best_i) == cells || std::abs(best_j) == cells)) ||
    (yaw_steps > 0 && std::abs(best_k) == yaw_steps)) {
    result.reason = "best fit on the edge of the search window";
    return result;
  }
  // --- Fine: two passes on the interpolated field, every beam -----------------------------------
  const auto refine = [&](Pose2 pose, bool pull_to_guess) {
    for (const auto & [half_xy, step_xy, half_yaw_deg, step_yaw_deg] :
         {std::array<double, 4>{res, 0.2 * res, 0.5, 0.1},
          std::array<double, 4>{0.2 * res, 0.05 * res, 0.1, 0.025}}) {
      const Pose2 center = pose;
      double best_fine = -1e9;
      const int n_xy = static_cast<int>(std::lround(half_xy / step_xy));
      const int n_yaw = static_cast<int>(std::lround(half_yaw_deg / step_yaw_deg));
      for (int k = -n_yaw; k <= n_yaw; ++k) {
        for (int j = -n_xy; j <= n_xy; ++j) {
          for (int i = -n_xy; i <= n_xy; ++i) {
            const Pose2 candidate{
              center.x + i * step_xy, center.y + j * step_xy, center.yaw + toRad(k * step_yaw_deg)};
            const double score = fineScore(
              d, endpoints, candidate, inv_sigma, pull_to_guess ? penalty(candidate) : 0.0);
            if (score > best_fine) {
              best_fine = score;
              pose = candidate;
            }
          }
        }
      }
    }
    pose.yaw = normalizeAngle(pose.yaw);
    return pose;
  };
  const Pose2 best = refine(
    {guess.x + best_i * res, guess.y + best_j * res, guess.yaw + toRad(best_k * kCoarseYawStepDeg)},
    true);
  result.pose = best;
  result.hit_ratio = hitRatio(d, endpoints, best, params.hit_tolerance);

  // --- Covariance: each hit beam constrains the pose along its surface's normal -----------------
  const double c = std::cos(best.yaw);
  const double s = std::sin(best.yaw);
  Mat3 information = Mat3::Zero();
  int used = 0;
  for (const Vec2 & e : endpoints) {
    const Vec2 p = transformPoint(best, c, s, e);
    if (d.interpolated(p.x(), p.y()) > params.hit_tolerance) {
      continue;
    }
    // The surface's normal from the free side: step back toward the robot by one cell, where the
    // distance grows away from the surface, and take the field's gradient there.
    Vec2 back = p - Vec2(best.x, best.y);
    const double length = back.norm();
    if (length < 1e-6) {
      continue;
    }
    const Vec2 q = p - (res / length) * back;
    const double h = 0.5 * res;
    const Vec2 gradient(
      (d.interpolated(q.x() + h, q.y()) - d.interpolated(q.x() - h, q.y())) / (2.0 * h),
      (d.interpolated(q.x(), q.y() + h) - d.interpolated(q.x(), q.y() - h)) / (2.0 * h));
    if (gradient.norm() < kMinNormalGradient) {
      continue;
    }
    const Vec2 n = gradient.normalized();
    const Vec2 de_dyaw(-s * e.x() - c * e.y(), c * e.x() - s * e.y());
    const Vec3 jacobian(n.x(), n.y(), n.dot(de_dyaw));
    information += jacobian * jacobian.transpose();
    ++used;
  }
  if (used > 0) {
    const double scale =
      std::min(1.0, static_cast<double>(params.effective_beams) / static_cast<double>(used));
    information *= scale / (params.beam_sigma * params.beam_sigma);
  }
  // A very weak prior (10 m, 180 deg) keeps the inverse finite where nothing constrains the pose.
  information += Vec3(1e-2, 1e-2, 1.0 / (M_PI * M_PI)).asDiagonal();
  result.covariance = information.inverse();

  // --- Plus the spread of the fit beyond the best fit's neighbourhood ---------------------------
  // The normals describe one sharp minimum. Walls several cells thick flatten it, and repeated
  // structure (shelf legs along an aisle, a wall mapped twice) gives near-equal minima tens of
  // centimetres apart, which one scan can't tell apart. So every pose of the window counts with
  // its likelihood, as in correlative scan matching: a beam's error is Cauchy (like a Gaussian of
  // beam_sigma for small misfits, while a beam a metre off, on an unmapped object, costs 3 nats,
  // not hundreds), and the beams count as effective_beams independent ones. The poses next to the
  // best (one step in each of x, y and heading) are the minimum the normals already describe;
  // sampled every cell they would only add the step, so they count in the total weight only.
  {
    double best_log_likelihood = -std::numeric_limits<double>::infinity();
    for (const Scored & w : window) {
      best_log_likelihood = std::max(best_log_likelihood, w.log_likelihood);
    }
    const double independent = std::min(
      1.0, static_cast<double>(params.effective_beams) / static_cast<double>(coarse.size()));
    const Vec3 at_best(best.x - guess.x, best.y - guess.y, normalizeAngle(best.yaw - guess.yaw));
    double weight_sum = 0.0;
    Mat3 spread = Mat3::Zero();
    for (const Scored & w : window) {
      const double weight = std::exp(independent * (w.log_likelihood - best_log_likelihood));
      weight_sum += weight;
      if (
        std::abs(w.i - best_i) <= 1 && std::abs(w.j - best_j) <= 1 && std::abs(w.k - best_k) <= 1) {
        continue;
      }
      const Vec3 error = Vec3(w.i * res, w.j * res, toRad(w.k * kCoarseYawStepDeg)) - at_best;
      spread += weight * error * error.transpose();
    }
    result.covariance += spread / weight_sum;
  }
  result.covariance = 0.5 * (result.covariance + result.covariance.transpose());

  // --- The best other place: separate maxima of the window, the best three refined -------------
  {
    const int side = 2 * cells + 1;
    const auto at = [&](int i, int j, int k) {
      return window
        [(static_cast<std::size_t>(k + yaw_steps) * static_cast<std::size_t>(side) +
          static_cast<std::size_t>(j + cells)) *
           static_cast<std::size_t>(side) +
         static_cast<std::size_t>(i + cells)]
          .log_likelihood;
    };
    std::vector<std::pair<double, std::array<int, 3>>> peaks;
    for (int k = -yaw_steps + 1; k < yaw_steps; ++k) {
      for (int j = -cells + 1; j < cells; ++j) {
        for (int i = -cells + 1; i < cells; ++i) {
          if (res * std::hypot(i - best_i, j - best_j) < params.alternative_distance) {
            continue;
          }
          const double value = at(i, j, k);
          bool peak = true;
          for (int dk = -1; dk <= 1 && peak; ++dk) {
            for (int dj = -1; dj <= 1 && peak; ++dj) {
              for (int di = -1; di <= 1 && peak; ++di) {
                peak = (di == 0 && dj == 0 && dk == 0) || at(i + di, j + dj, k + dk) <= value;
              }
            }
          }
          if (peak) {
            peaks.push_back({value, {i, j, k}});
          }
        }
      }
    }
    constexpr std::size_t kRefinedPeaks = 3;
    const std::size_t kept = std::min(peaks.size(), kRefinedPeaks);
    std::partial_sort(
      peaks.begin(), peaks.begin() + static_cast<std::ptrdiff_t>(kept), peaks.end(),
      [](const auto & a, const auto & b) { return a.first > b.first; });
    const double best_log_likelihood = logLikelihood(d, endpoints, best, inv_sigma);
    double alternative_log_likelihood = -std::numeric_limits<double>::infinity();
    for (std::size_t n = 0; n < kept; ++n) {
      const auto & [i, j, k] = peaks[n].second;
      const Pose2 candidate = refine(
        {guess.x + i * res, guess.y + j * res, guess.yaw + toRad(k * kCoarseYawStepDeg)}, false);
      if (std::hypot(candidate.x - best.x, candidate.y - best.y) < params.alternative_distance) {
        continue;  // slid back into the best fit's peak
      }
      const double value = logLikelihood(d, endpoints, candidate, inv_sigma);
      if (value > alternative_log_likelihood) {
        alternative_log_likelihood = value;
        result.alternative = candidate;
      }
    }
    if (result.alternative) {
      const double independent = std::min(
        1.0, static_cast<double>(params.effective_beams) / static_cast<double>(endpoints.size()));
      result.alternative_nats = independent * (best_log_likelihood - alternative_log_likelihood);
    }
  }

  if (result.hit_ratio < params.min_hit_ratio) {
    result.reason = "hit ratio " + std::to_string(result.hit_ratio) + " < " +
                    std::to_string(params.min_hit_ratio);
    return result;
  }
  result.valid = true;
  return result;
}

}  // namespace landfall
