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

#include "landfall/geometry/floor_refinement.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace landfall
{
namespace
{

using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat26 = Eigen::Matrix<double, 2, 6>;

// The median length of a 2D residual whose axes are Gaussian with standard deviation s is
// s * sqrt(2 ln 2) (Rayleigh distribution).
constexpr double kRayleighMedian = 1.1774100225154747;

// A point closer to the camera plane than this (normalized depth, metres) counts as behind it.
constexpr double kMinDepth = 1e-6;

Mat3 skew(const Vec3 & v)
{
  Mat3 m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

Mat3 expRotation(const Vec3 & w)
{
  const double angle = w.norm();
  if (angle < 1e-12) {
    return Mat3::Identity() + skew(w);
  }
  return Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();
}

// The base pose is (R, t) in the map frame. Updates perturb it in the map frame:
// t' = t + dt, R' = exp(dtheta) R, so dtheta.z() turns the base about the vertical (its yaw) and
// dt.x(), dt.y() move it on the floor. Residuals are in pixels.
class Problem
{
public:
  Problem(
    const std::vector<Vec2> & normalized, const std::vector<Vec3> & points,
    const std::vector<int> & indices, const FloorPrior & prior,
    const FloorRefinementOptions & options)
  : x_(normalized),
    X_(points),
    indices_(indices),
    R_cam_base_(prior.T_base_cam.linear().transpose()),
    t_base_cam_(prior.T_base_cam.translation()),
    gravity_base_(prior.gravity_base.normalized()),
    height_(prior.height),
    inv_height_std_(prior.height_std > 0.0 ? 1.0 / prior.height_std : 0.0),
    inv_tilt_std_(prior.tilt_std > 0.0 ? 1.0 / prior.tilt_std : 0.0),
    f_(options.focal_px),
    c2_(options.threshold_px * options.threshold_px)
  {
  }

  // Pixel residual of correspondence i, and its Jacobian with respect to (dt, dtheta).
  bool residual(int i, const Mat3 & R, const Vec3 & t, Vec2 * r, Mat26 * J) const
  {
    const Vec3 v = X_[static_cast<std::size_t>(i)] - t;
    const Mat3 A = R_cam_base_ * R.transpose();  // map directions into the camera frame
    const Vec3 p = A * v - R_cam_base_ * t_base_cam_;
    if (p.z() < kMinDepth) {
      return false;
    }
    const double iz = 1.0 / p.z();
    *r = f_ * (Vec2(p.x() * iz, p.y() * iz) - x_[static_cast<std::size_t>(i)]);
    if (J != nullptr) {
      Eigen::Matrix<double, 2, 3> dproj;
      dproj << f_ * iz, 0.0, -f_ * p.x() * iz * iz, 0.0, f_ * iz, -f_ * p.y() * iz * iz;
      J->leftCols<3>() = -dproj * A;
      J->rightCols<3>() = dproj * A * skew(v);
    }
    return true;
  }

  // Robust (Cauchy) weight of a residual: 1 for small ones, falling off beyond the threshold.
  double weight(const Vec2 & r) const { return 1.0 / (1.0 + r.squaredNorm() / c2_); }

  // Whitened prior residuals (height, tilt along map x and y) and their Jacobian.
  void prior(const Mat3 & R, const Vec3 & t, Vec3 * r, Eigen::Matrix<double, 3, 6> * J) const
  {
    const Vec3 down = R * gravity_base_;  // the base's measured down, in the map: ideally -z
    r->x() = (t.z() - height_) * inv_height_std_;
    r->y() = down.x() * inv_tilt_std_;
    r->z() = down.y() * inv_tilt_std_;
    if (J != nullptr) {
      J->setZero();
      (*J)(0, 2) = inv_height_std_;
      // d(down) / d(dtheta) = -[down]x
      const Mat3 d_down = -skew(down);
      J->block<1, 3>(1, 3) = d_down.row(0) * inv_tilt_std_;
      J->block<1, 3>(2, 3) = d_down.row(1) * inv_tilt_std_;
    }
  }

  // Robust cost with the residuals whitened by the reprojection noise, plus the prior.
  double cost(const Mat3 & R, const Vec3 & t, double inv_noise2) const
  {
    double total = 0.0;
    for (const int i : indices_) {
      Vec2 r;
      // Behind the camera: the cost of a residual far beyond the threshold.
      const double r2 = residual(i, R, t, &r, nullptr) ? r.squaredNorm() : 1e6 * c2_;
      total += c2_ * std::log1p(r2 / c2_) * inv_noise2;
    }
    Vec3 p;
    prior(R, t, &p, nullptr);
    return total + p.squaredNorm();
  }

  // Gauss-Newton normal equations (IRLS weights) at the pose.
  void normalEquations(
    const Mat3 & R, const Vec3 & t, double inv_noise2, Mat6 * H, Vec6 * g, int * used) const
  {
    H->setZero();
    g->setZero();
    *used = 0;
    for (const int i : indices_) {
      Vec2 r;
      Mat26 J;
      if (!residual(i, R, t, &r, &J)) {
        continue;
      }
      ++*used;
      const double w = weight(r) * inv_noise2;
      *H += w * J.transpose() * J;
      *g += w * J.transpose() * r;
    }
    Vec3 p;
    Eigen::Matrix<double, 3, 6> Jp;
    prior(R, t, &p, &Jp);
    *H += Jp.transpose() * Jp;
    *g += Jp.transpose() * p;
  }

  // Reprojection noise from the residuals: robust to the few wrong correspondences that fall
  // within the threshold.
  double noise(const Mat3 & R, const Vec3 & t, double min_noise, double * mean) const
  {
    std::vector<double> lengths;
    lengths.reserve(indices_.size());
    double sum = 0.0;
    for (const int i : indices_) {
      Vec2 r;
      if (residual(i, R, t, &r, nullptr)) {
        lengths.push_back(r.norm());
        sum += lengths.back();
      }
    }
    if (lengths.empty()) {
      *mean = 0.0;
      return min_noise;
    }
    *mean = sum / static_cast<double>(lengths.size());
    const auto middle = lengths.begin() + static_cast<std::ptrdiff_t>(lengths.size() / 2);
    std::nth_element(lengths.begin(), middle, lengths.end());
    return std::max(min_noise, *middle / kRayleighMedian);
  }

  int inliers(const Mat3 & R, const Vec3 & t) const
  {
    int count = 0;
    for (std::size_t i = 0; i < x_.size(); ++i) {
      Vec2 r;
      if (residual(static_cast<int>(i), R, t, &r, nullptr) && r.squaredNorm() < c2_) {
        ++count;
      }
    }
    return count;
  }

private:
  const std::vector<Vec2> & x_;
  const std::vector<Vec3> & X_;
  const std::vector<int> & indices_;
  Mat3 R_cam_base_;
  Vec3 t_base_cam_;
  Vec3 gravity_base_;
  double height_;
  double inv_height_std_;
  double inv_tilt_std_;
  double f_;
  double c2_;
};

}  // namespace

FloorRefinement refineOnFloor(
  const std::vector<Vec2> & normalized, const std::vector<Vec3> & points,
  const std::vector<int> & indices, const Pose3 & T_map_base, const FloorPrior & prior,
  const FloorRefinementOptions & options)
{
  FloorRefinement result;
  if (
    normalized.size() != points.size() || indices.size() < 3 || !(options.focal_px > 0.0) ||
    !(options.threshold_px > 0.0) || !(prior.gravity_base.norm() > 0.0)) {
    return result;
  }
  for (const int i : indices) {
    if (i < 0 || static_cast<std::size_t>(i) >= points.size()) {
      return result;
    }
  }
  const Problem problem(normalized, points, indices, prior, options);
  Mat3 R = T_map_base.linear();
  Vec3 t = T_map_base.translation();
  double mean = 0.0;
  // The noise whitens the residuals against the prior; it is estimated at the starting pose (the
  // free solution) and again where the refinement ends, for the covariance.
  double noise = problem.noise(R, t, options.min_noise_px, &mean);
  double inv_noise2 = 1.0 / (noise * noise);
  double current = problem.cost(R, t, inv_noise2);
  double lambda = 1e-4;
  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    Mat6 H;
    Vec6 g;
    int used = 0;
    problem.normalEquations(R, t, inv_noise2, &H, &g, &used);
    if (used < 3) {
      return result;
    }
    bool improved = false;
    Vec6 step = Vec6::Zero();
    for (int attempt = 0; attempt < 8 && !improved; ++attempt) {
      Mat6 A = H;
      A.diagonal() += lambda * H.diagonal() + Vec6::Constant(1e-12);
      step = A.ldlt().solve(-g);
      if (!step.allFinite()) {
        break;
      }
      const Mat3 R_candidate = expRotation(step.tail<3>()) * R;
      const Vec3 t_candidate = t + step.head<3>();
      const double candidate = problem.cost(R_candidate, t_candidate, inv_noise2);
      if (candidate < current) {
        R = R_candidate;
        t = t_candidate;
        current = candidate;
        lambda = std::max(lambda * 0.1, 1e-10);
        improved = true;
      } else {
        lambda *= 10.0;
      }
    }
    result.iterations = iteration + 1;
    if (!improved || step.norm() < 1e-10) {
      break;
    }
  }
  // Keep the rotation orthonormal after the updates.
  R = Eigen::Quaterniond(R).normalized().toRotationMatrix();

  noise = problem.noise(R, t, options.min_noise_px, &mean);
  inv_noise2 = 1.0 / (noise * noise);
  Mat6 H;
  Vec6 g;
  int used = 0;
  problem.normalEquations(R, t, inv_noise2, &H, &g, &used);
  const Eigen::LLT<Mat6> llt(H);
  if (used < 3 || llt.info() != Eigen::Success) {
    return result;
  }
  const Mat6 covariance = llt.solve(Mat6::Identity());
  // x, y and yaw: dt.x(), dt.y() and dtheta.z().
  const std::array<int, 3> planar = {0, 1, 5};
  for (std::size_t a = 0; a < 3; ++a) {
    for (std::size_t b = 0; b < 3; ++b) {
      result.covariance(static_cast<int>(a), static_cast<int>(b)) =
        covariance(planar[a], planar[b]);
    }
  }
  result.covariance = 0.5 * (result.covariance + result.covariance.transpose());
  if (
    !result.covariance.allFinite() ||
    Eigen::SelfAdjointEigenSolver<Mat3>(result.covariance).eigenvalues().minCoeff() <= 0.0) {
    return result;
  }
  result.T_map_base.linear() = R;
  result.T_map_base.translation() = t;
  result.pose = toPose2(result.T_map_base);
  result.noise_px = noise;
  result.mean_reprojection_px = mean;
  result.inliers = problem.inliers(R, t);
  result.valid = true;
  return result;
}

}  // namespace landfall
