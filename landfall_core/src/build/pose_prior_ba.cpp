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
#include <stdexcept>

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

  std::vector<std::array<double, 3>> pose_blocks(keyframes.size());
  for (std::size_t k = 0; k < keyframes.size(); ++k) {
    pose_blocks[k] = {keyframes[k].prior.x, keyframes[k].prior.y, keyframes[k].prior.yaw};
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
  const double max_yaw = toRad(params.max_shift_yaw_deg);
  for (std::size_t k = 0; k < keyframes.size(); ++k) {
    double * pose = pose_blocks[k].data();
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<PriorResidual, 3, 3>(
        new PriorResidual(keyframes[k].prior, sqrtInformation(keyframes[k].covariance, params))),
      nullptr, pose);
    const double bounds[3] = {params.max_shift_xy, params.max_shift_xy, max_yaw};
    for (int i = 0; i < 3; ++i) {
      problem.SetParameterLowerBound(pose, i, pose[i] - bounds[i]);
      problem.SetParameterUpperBound(pose, i, pose[i] + bounds[i]);
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  std::string invalid;
  if (!options.IsValid(&invalid)) {
    options.linear_solver_type = ceres::DENSE_SCHUR;
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
    result.poses[k] = {p[0], p[1], p[2]};
    const Pose2 & prior = keyframes[k].prior;
    const double tolerance = 1e-9;
    result.at_bound[k] = std::abs(p[0] - prior.x) >= params.max_shift_xy - tolerance ||
                             std::abs(p[1] - prior.y) >= params.max_shift_xy - tolerance ||
                             std::abs(p[2] - prior.yaw) >= max_yaw - tolerance
                           ? 1
                           : 0;
  }
  for (std::size_t l = 0; l < landmarks.size(); ++l) {
    result.landmarks[l] = Vec3(point_blocks[l][0], point_blocks[l][1], point_blocks[l][2]);
  }
  return result;
}

}  // namespace landfall
