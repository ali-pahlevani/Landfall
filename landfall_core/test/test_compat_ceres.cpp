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

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <vector>

#include "landfall/compat/ceres.hpp"

namespace
{

// Residual between a rotated source vector and its target; the rotation is a unit quaternion
// stored as Eigen coefficients (x, y, z, w).
struct RotationResidual
{
  RotationResidual(const Eigen::Vector3d & source, const Eigen::Vector3d & target)
  : source_(source), target_(target)
  {
  }

  template <typename T>
  bool operator()(const T * const quaternion_xyzw, T * residual) const
  {
    const Eigen::Map<const Eigen::Quaternion<T>> q(quaternion_xyzw);
    const Eigen::Matrix<T, 3, 1> rotated = q * source_.cast<T>();
    for (int i = 0; i < 3; ++i) {
      residual[i] = rotated[i] - T(target_[i]);
    }
    return true;
  }

  Eigen::Vector3d source_;
  Eigen::Vector3d target_;
};

// Pulls each coordinate of a 3-vector toward a target value.
struct PointToTargetResidual
{
  explicit PointToTargetResidual(const Eigen::Vector3d & target) : target_(target) {}

  template <typename T>
  bool operator()(const T * const point, T * residual) const
  {
    for (int i = 0; i < 3; ++i) {
      residual[i] = point[i] - T(target_[i]);
    }
    return true;
  }

  Eigen::Vector3d target_;
};

ceres::Solver::Summary solve(ceres::Problem & problem)
{
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 100;
  options.function_tolerance = 1e-14;
  options.gradient_tolerance = 1e-14;
  options.parameter_tolerance = 1e-14;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  return summary;
}

}  // namespace

TEST(CompatCeres, QuaternionManifoldRecoversRotationAndStaysUnit)
{
  const Eigen::Quaterniond truth(
    Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
  const std::vector<Eigen::Vector3d> sources = {
    {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {0.3, -0.4, 0.8}};

  std::array<double, 4> q_xyzw = {0.0, 0.0, 0.0, 1.0};  // identity
  ceres::Problem problem;
  for (const auto & source : sources) {
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RotationResidual, 3, 4>(
        new RotationResidual(source, truth * source)),
      nullptr, q_xyzw.data());
  }
  landfall::compat::setEigenQuaternionManifold(problem, q_xyzw.data());

  const ceres::Solver::Summary summary = solve(problem);
  ASSERT_TRUE(summary.IsSolutionUsable()) << summary.BriefReport();

  const Eigen::Quaterniond estimate(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
  EXPECT_NEAR(estimate.norm(), 1.0, 1e-12);
  EXPECT_LT(estimate.angularDistance(truth), 1e-8);
}

TEST(CompatCeres, SubsetManifoldHoldsConstantCoordinates)
{
  const Eigen::Vector3d target(10.0, 20.0, 30.0);
  std::array<double, 3> point = {1.0, 2.0, 3.0};

  ceres::Problem problem;
  problem.AddResidualBlock(
    new ceres::AutoDiffCostFunction<PointToTargetResidual, 3, 3>(new PointToTargetResidual(target)),
    nullptr, point.data());
  landfall::compat::setSubsetManifold(problem, point.data(), 3, {1});

  const ceres::Solver::Summary summary = solve(problem);
  ASSERT_TRUE(summary.IsSolutionUsable()) << summary.BriefReport();

  // Free coordinates converge to solver precision; the constant one must not move at all.
  EXPECT_NEAR(point[0], 10.0, 1e-6);
  EXPECT_DOUBLE_EQ(point[1], 2.0);
  EXPECT_NEAR(point[2], 30.0, 1e-6);
}

TEST(CompatCeres, ReportsWhichApiIsInUse)
{
  RecordProperty("ceres_version", CERES_VERSION_STRING);
  RecordProperty("uses_manifold", LANDFALL_CERES_HAS_MANIFOLD);
  SUCCEED() << "Ceres " << CERES_VERSION_STRING
            << (LANDFALL_CERES_HAS_MANIFOLD ? " (Manifold API)" : " (LocalParameterization API)");
}
