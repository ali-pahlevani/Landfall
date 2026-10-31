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

#include "landfall/geometry/two_view.hpp"

#include <PoseLib/camera_pose.h>
#include <PoseLib/robust/bundle.h>
#include <PoseLib/robust/ransac.h>
#include <PoseLib/robust/utils.h>
#include <PoseLib/types.h>

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace landfall
{

namespace
{

void checkInput(const std::vector<Vec2> & x1, const std::vector<Vec2> & x2)
{
  if (x1.size() != x2.size()) {
    throw std::invalid_argument("two-view: correspondence lists differ in length");
  }
}

// Rotation R minimizing sum |b2 - R b1|^2 over the given pairs (Kabsch).
Mat3 kabsch(
  const std::vector<Vec3> & b1, const std::vector<Vec3> & b2, const std::vector<int> & set)
{
  Mat3 H = Mat3::Zero();
  for (int i : set) {
    H += b2[i] * b1[i].transpose();
  }
  const Eigen::JacobiSVD<Mat3> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3 D = Mat3::Identity();
  D(2, 2) = (svd.matrixU() * svd.matrixV().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
  return svd.matrixU() * D * svd.matrixV().transpose();
}

std::vector<int> rotationInliers(
  const std::vector<Vec3> & b1, const std::vector<Vec3> & b2, const Mat3 & R, double max_angle)
{
  const double min_cos = std::cos(max_angle);
  std::vector<int> inliers;
  for (int i = 0; i < static_cast<int>(b1.size()); ++i) {
    if (b2[i].dot(R * b1[i]) >= min_cos) {
      inliers.push_back(i);
    }
  }
  return inliers;
}

}  // namespace

std::optional<TwoViewGeometry> estimateRelativePose(
  const std::vector<Vec2> & x1, const std::vector<Vec2> & x2, const TwoViewParams & params)
{
  checkInput(x1, x2);
  if (x1.size() < 5) {
    return std::nullopt;
  }
  const std::vector<poselib::Point2D> a(x1.begin(), x1.end());
  const std::vector<poselib::Point2D> b(x2.begin(), x2.end());
  poselib::RansacOptions ransac;
  ransac.max_iterations = static_cast<std::size_t>(params.max_iterations);
  ransac.min_iterations = static_cast<std::size_t>(params.min_iterations);
  ransac.success_prob = params.success_prob;
  ransac.seed = params.seed;
  ransac.max_epipolar_error = params.max_error_px / params.focal_px;

  poselib::CameraPose pose;
  std::vector<char> mask;
  const poselib::RansacStats stats = poselib::ransac_relpose(a, b, ransac, &pose, &mask);
  if (stats.num_inliers < 5) {
    return std::nullopt;
  }
  std::vector<poselib::Point2D> a_in;
  std::vector<poselib::Point2D> b_in;
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (mask[i] != 0) {
      a_in.push_back(a[i]);
      b_in.push_back(b[i]);
    }
  }
  poselib::BundleOptions bundle;
  bundle.loss_type = poselib::BundleOptions::LossType::CAUCHY;
  bundle.loss_scale = 0.5 * ransac.max_epipolar_error;
  poselib::refine_relpose(a_in, b_in, &pose, bundle);
  poselib::get_inliers(pose, a, b, ransac.max_epipolar_error * ransac.max_epipolar_error, &mask);

  TwoViewGeometry geometry;
  geometry.R_21 = pose.R();
  geometry.t_21 = pose.t.norm() > 0.0 ? Vec3(pose.t.normalized()) : Vec3::Zero();
  for (int i = 0; i < static_cast<int>(mask.size()); ++i) {
    if (mask[i] != 0) {
      geometry.inliers.push_back(i);
    }
  }
  if (geometry.inliers.size() < 5 || !geometry.R_21.allFinite()) {
    return std::nullopt;
  }
  return geometry;
}

std::optional<TwoViewGeometry> estimateRelativeRotation(
  const std::vector<Vec2> & x1, const std::vector<Vec2> & x2, const TwoViewParams & params)
{
  checkInput(x1, x2);
  const int n = static_cast<int>(x1.size());
  if (n < 2) {
    return std::nullopt;
  }
  std::vector<Vec3> b1(n);
  std::vector<Vec3> b2(n);
  for (int i = 0; i < n; ++i) {
    b1[i] = x1[i].homogeneous().normalized();
    b2[i] = x2[i].homogeneous().normalized();
  }
  const double max_angle = params.max_error_px / params.focal_px;
  std::mt19937_64 rng(params.seed);
  std::uniform_int_distribution<int> pick(0, n - 1);

  std::vector<int> best;
  Mat3 best_R = Mat3::Identity();
  double needed = params.max_iterations;
  for (int iteration = 0; iteration < params.max_iterations; ++iteration) {
    if (iteration >= params.min_iterations && iteration >= needed) {
      break;
    }
    const int i = pick(rng);
    const int j = pick(rng);
    if (i == j || b1[i].cross(b1[j]).norm() < 1e-6) {
      continue;
    }
    Mat3 R = kabsch(b1, b2, {i, j});
    auto inliers = rotationInliers(b1, b2, R, max_angle);
    if (inliers.size() <= best.size()) {
      continue;
    }
    // Local optimization: refit on the inliers until the set stops growing.
    while (inliers.size() >= 2) {
      const Mat3 refit = kabsch(b1, b2, inliers);
      auto grown = rotationInliers(b1, b2, refit, max_angle);
      if (grown.size() <= inliers.size()) {
        break;
      }
      R = refit;
      inliers = std::move(grown);
    }
    best = std::move(inliers);
    best_R = R;
    const double ratio = static_cast<double>(best.size()) / n;
    const double fail = 1.0 - ratio * ratio;
    needed = fail <= 0.0 ? 0.0 : std::log(1.0 - params.success_prob) / std::log(fail);
  }
  if (best.size() < 2) {
    return std::nullopt;
  }
  TwoViewGeometry geometry;
  geometry.R_21 = kabsch(b1, b2, best);
  geometry.inliers = rotationInliers(b1, b2, geometry.R_21, max_angle);
  if (geometry.inliers.size() < 2) {
    geometry.R_21 = best_R;
    geometry.inliers = best;
  }
  return geometry;
}

}  // namespace landfall
