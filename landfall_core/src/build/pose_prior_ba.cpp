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

#include "landfall/build/pose_prior_ba.hpp"

#include <ceres/ceres.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace landfall
{

namespace
{

double toRad(double degrees) { return degrees * M_PI / 180.0; }

// Reprojection of a free landmark into a keyframe whose base pose (x, y, yaw) is a parameter.
struct Reprojection
{
  Reprojection(const Pose3 & T_base_cam, const Vec2 & observed, double focal)
  : R_cam_base(T_base_cam.linear().transpose()),
    t_base_cam(T_base_cam.translation()),
    x(observed),
    f(focal)
  {
  }

  template <typename T>
  bool operator()(const T * const pose, const T * const point, T * residual) const
  {
    using std::cos;
    using std::sin;
    const T c = cos(pose[2]);
    const T s = sin(pose[2]);
    const T dx = point[0] - pose[0];
    const T dy = point[1] - pose[1];
    const Eigen::Matrix<T, 3, 1> in_base(c * dx + s * dy, -s * dx + c * dy, point[2]);
    const Eigen::Matrix<T, 3, 1> in_cam = R_cam_base.cast<T>() * (in_base - t_base_cam.cast<T>());
    if (in_cam.z() < T(1e-3)) {
      return false;
    }
    residual[0] = T(f) * (in_cam.x() / in_cam.z() - T(x.x()));
    residual[1] = T(f) * (in_cam.y() / in_cam.z() - T(x.y()));
    return true;
  }

  Mat3 R_cam_base;
  Vec3 t_base_cam;
  Vec2 x;
  double f;
};

// Whitened deviation of a base pose from its recorded prior.
struct PriorResidual
{
  PriorResidual(const Pose2 & prior, const Mat3 & sqrt_information)
  : prior(prior), sqrt_information(sqrt_information)
  {
  }

  template <typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    const Eigen::Matrix<T, 3, 1> d(
      pose[0] - T(prior.x), pose[1] - T(prior.y), pose[2] - T(prior.yaw));
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residual);
    r = sqrt_information.cast<T>() * d;
    return true;
  }

  Pose2 prior;
  Mat3 sqrt_information;
};

// Relative motion of two consecutive keyframes against their odometry, whitened.
struct OdometryResidual
{
  OdometryResidual(const Pose2 & delta, const Vec3 & inverse_std)
  : delta(delta), inverse_std(inverse_std)
  {
  }

  template <typename T>
  bool operator()(const T * const a, const T * const b, T * residual) const
  {
    using std::atan2;
    using std::cos;
    using std::sin;
    const T c = cos(a[2]);
    const T s = sin(a[2]);
    const T dx = b[0] - a[0];
    const T dy = b[1] - a[1];
    const T dyaw = b[2] - a[2] - T(delta.yaw);
    residual[0] = (c * dx + s * dy - T(delta.x)) * T(inverse_std.x());
    residual[1] = (-s * dx + c * dy - T(delta.y)) * T(inverse_std.y());
    residual[2] = atan2(sin(dyaw), cos(dyaw)) * T(inverse_std.z());
    return true;
  }

  Pose2 delta;
  Vec3 inverse_std;
};

// U with U^T U = (covariance + floors)^-1, or the configured prior std's when set. Falls back to the
// diagonal for a covariance that isn't positive semi-definite (localization stacks occasionally
// publish those).
Mat3 sqrtInformation(const std::optional<Mat3> & covariance, const PosePriorBaParams & params)
{
  if (params.prior_std_xy > 0.0 && params.prior_std_yaw_deg > 0.0) {
    return Vec3(
             1.0 / params.prior_std_xy, 1.0 / params.prior_std_xy,
             1.0 / toRad(params.prior_std_yaw_deg))
      .asDiagonal();
  }
  const Vec3 floors(
    params.prior_floor_xy * params.prior_floor_xy, params.prior_floor_xy * params.prior_floor_xy,
    toRad(params.prior_floor_yaw_deg) * toRad(params.prior_floor_yaw_deg));
  Mat3 sigma = floors.asDiagonal();
  if (covariance && covariance->allFinite()) {
    const Mat3 symmetric = 0.5 * (*covariance + covariance->transpose());
    const Eigen::SelfAdjointEigenSolver<Mat3> eigen(symmetric, Eigen::EigenvaluesOnly);
    if (eigen.eigenvalues().minCoeff() >= 0.0) {
      sigma += symmetric;
    } else {
      sigma += symmetric.diagonal().cwiseMax(0.0).asDiagonal();
    }
  }
  const Eigen::LLT<Mat3> llt(sigma.inverse());
  return llt.matrixU();
}

}  // namespace

Pose3 cameraPose(const Pose2 & map_base, const Pose3 & T_base_cam)
{
  return toPose3(map_base) * T_base_cam;
}

namespace
{

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

}  // namespace

OdometryCheck checkOdometry(const std::vector<OdometryStep> & steps, double min_turn_rad)
{
  std::vector<double> straight;
  std::vector<std::pair<double, double>> turning;  // |e|^2, turn
  for (const auto & step : steps) {
    const double ex = step.odometry.x - step.reference.x;
    const double ey = step.odometry.y - step.reference.y;
    const double turn = std::abs(normalizeAngle(step.reference.yaw));
    if (turn < 5.0 * M_PI / 180.0) {
      straight.push_back(ex * ex + ey * ey);
    } else if (turn >= min_turn_rad) {
      turning.emplace_back(ex * ex + ey * ey, turn);
    }
  }
  const double chi2_median = 2.0 * std::log(2.0);
  const double floor_variance = median(straight) / chi2_median;
  std::vector<double> per_rad;
  per_rad.reserve(turning.size());
  for (const auto & [error2, turn] : turning) {
    per_rad.push_back(std::max(0.0, error2 / chi2_median - floor_variance) / (turn * turn));
  }
  OdometryCheck check;
  check.straight_steps = static_cast<int>(straight.size());
  check.straight_std_m = std::sqrt(floor_variance);
  check.turns = static_cast<int>(turning.size());
  check.turn_std_per_rad = std::sqrt(median(per_rad));
  return check;
}

BaResult adjustPosePriors(
  const std::vector<BaKeyframe> & keyframes, const std::vector<Vec3> & landmarks,
  const std::vector<BaObservation> & observations, const PosePriorBaParams & params)
{
  BaResult result;
  result.poses.reserve(keyframes.size());
  for (const auto & k : keyframes) {
    result.poses.push_back(k.prior);
  }
  result.landmarks = landmarks;
  result.at_bound.assign(keyframes.size(), 0);

  // Per keyframe: the prior's square-root information and the hard bounds around the prior. A
  // prior with its own information may move up to 3 of its standard deviations, at least the
  // configured shift.
  const double max_yaw = toRad(params.max_shift_yaw_deg);
  std::vector<Mat3> sqrt_info(keyframes.size());
  std::vector<std::array<double, 3>> bounds(keyframes.size());
  for (std::size_t k = 0; k < keyframes.size(); ++k) {
    bounds[k] = {params.max_shift_xy, params.max_shift_xy, max_yaw};
    if (keyframes[k].prior_sqrt_information) {
      sqrt_info[k] = *keyframes[k].prior_sqrt_information;
      const Mat3 covariance = (sqrt_info[k].transpose() * sqrt_info[k]).inverse();
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> xy(
        0.5 * (covariance.topLeftCorner<2, 2>() + covariance.topLeftCorner<2, 2>().transpose()),
        Eigen::EigenvaluesOnly);
      const double std_xy = std::sqrt(std::max(xy.eigenvalues().maxCoeff(), 0.0));
      const double std_yaw = std::sqrt(std::max(covariance(2, 2), 0.0));
      bounds[k] = {
        std::max(params.max_shift_xy, 3.0 * std_xy), std::max(params.max_shift_xy, 3.0 * std_xy),
        std::max(max_yaw, 3.0 * std_yaw)};
    } else {
      sqrt_info[k] = sqrtInformation(keyframes[k].covariance, params);
    }
  }

  // The start: the initial pose (its heading unwrapped next to the prior's), inside the bounds.
  std::vector<std::array<double, 3>> pose_blocks(keyframes.size());
  for (std::size_t k = 0; k < keyframes.size(); ++k) {
    const Pose2 & prior = keyframes[k].prior;
    const Pose2 start = keyframes[k].initial.value_or(prior);
    const double values[3] = {start.x, start.y, prior.yaw + normalizeAngle(start.yaw - prior.yaw)};
    const double centers[3] = {prior.x, prior.y, prior.yaw};
    for (int i = 0; i < 3; ++i) {
      pose_blocks[k][i] =
        std::clamp(values[i], centers[i] - bounds[k][i], centers[i] + bounds[k][i]);
    }
  }
  std::vector<std::array<double, 3>> point_blocks(landmarks.size());
  for (std::size_t l = 0; l < landmarks.size(); ++l) {
    point_blocks[l] = {landmarks[l].x(), landmarks[l].y(), landmarks[l].z()};
  }

  ceres::Problem problem;
  for (const auto & o : observations) {
    if (
      o.keyframe < 0 || o.keyframe >= static_cast<int>(keyframes.size()) || o.landmark < 0 ||
      o.landmark >= static_cast<int>(landmarks.size())) {
      throw std::out_of_range("adjustPosePriors: observation refers to a missing element");
    }
    const BaKeyframe & k = keyframes[o.keyframe];
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<Reprojection, 2, 3, 3>(
        new Reprojection(k.T_base_cam, o.normalized, k.focal_px)),
      new ceres::CauchyLoss(params.loss_scale_px), pose_blocks[o.keyframe].data(),
      point_blocks[o.landmark].data());
  }
  for (std::size_t k = 0; k < keyframes.size(); ++k) {
    double * pose = pose_blocks[k].data();
    ceres::LossFunction * loss =
      keyframes[k].prior_sqrt_information ? new ceres::HuberLoss(params.prior_loss_scale) : nullptr;
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<PriorResidual, 3, 3>(
        new PriorResidual(keyframes[k].prior, sqrt_info[k])),
      loss, pose);
    const Pose2 & prior = keyframes[k].prior;
    const double centers[3] = {prior.x, prior.y, prior.yaw};
    for (int i = 0; i < 3; ++i) {
      problem.SetParameterLowerBound(pose, i, centers[i] - bounds[k][i]);
      problem.SetParameterUpperBound(pose, i, centers[i] + bounds[k][i]);
    }
  }
  const OdometryParams & odo = params.odometry;
  for (std::size_t k = 1; k < keyframes.size(); ++k) {
    const BaKeyframe & a = keyframes[k - 1];
    const BaKeyframe & b = keyframes[k];
    if (a.session != b.session || !a.odom || !b.odom) {
      continue;
    }
    const Pose2 delta = between(*a.odom, *b.odom);
    const double distance = std::hypot(delta.x, delta.y);
    if (distance > odo.max_step) {
      continue;
    }
    const double std_xy =
      odo.std_xy + odo.std_xy_per_m * distance + odo.std_xy_per_rad * std::abs(delta.yaw);
    const double std_yaw = toRad(odo.std_yaw_deg) + odo.std_yaw_per_rad * std::abs(delta.yaw);
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<OdometryResidual, 3, 3, 3>(
        new OdometryResidual(delta, Vec3(1.0 / std_xy, 1.0 / std_xy, 1.0 / std_yaw))),
      new ceres::CauchyLoss(2.0), pose_blocks[k - 1].data(), pose_blocks[k].data());
    ++result.odometry_links;
  }

  ceres::Solver::Options options;
  // Landmarks: the Schur complement on them. A pose graph alone: sparse normal equations.
  options.linear_solver_type =
    observations.empty() ? ceres::SPARSE_NORMAL_CHOLESKY : ceres::SPARSE_SCHUR;
  std::string invalid;
  if (!options.IsValid(&invalid)) {
    options.linear_solver_type = observations.empty() ? ceres::DENSE_QR : ceres::DENSE_SCHUR;
  }
  options.max_num_iterations = params.max_iterations;
  options.num_threads = 1;
  options.logging_type = ceres::SILENT;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  result.initial_cost = summary.initial_cost;
  result.final_cost = summary.final_cost;
  result.iterations = static_cast<int>(summary.iterations.size());
  result.message = summary.BriefReport();
  if (!summary.IsSolutionUsable()) {
    return result;
  }
  result.usable = true;
  for (std::size_t k = 0; k < keyframes.size(); ++k) {
    const auto & p = pose_blocks[k];
    result.poses[k] = {p[0], p[1], p[2]};  // heading unwrapped next to the prior's
    const Pose2 & prior = keyframes[k].prior;
    const double tolerance = 1e-9;
    result.at_bound[k] = std::abs(p[0] - prior.x) >= bounds[k][0] - tolerance ||
                             std::abs(p[1] - prior.y) >= bounds[k][1] - tolerance ||
                             std::abs(p[2] - prior.yaw) >= bounds[k][2] - tolerance
                           ? 1
                           : 0;
  }
  for (std::size_t l = 0; l < landmarks.size(); ++l) {
    result.landmarks[l] = Vec3(point_blocks[l][0], point_blocks[l][1], point_blocks[l][2]);
  }
  return result;
}

}  // namespace landfall
