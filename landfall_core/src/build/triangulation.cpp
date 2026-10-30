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

#include "landfall/build/triangulation.hpp"

#include <ceres/ceres.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace landfall
{
namespace
{

// Reprojection residual of a point seen by a fixed camera, in normalized units.
struct ReprojectionError
{
  ReprojectionError(const Pose3 & T_cam_map, const Vec2 & observed)
  : R(T_cam_map.linear()), t(T_cam_map.translation()), x(observed)
  {
  }

  template <typename T>
  bool operator()(const T * const point, T * residual) const
  {
    const Eigen::Matrix<T, 3, 1> X(point[0], point[1], point[2]);
    const Eigen::Matrix<T, 3, 1> p = R.cast<T>() * X + t.cast<T>();
    residual[0] = p.x() / p.z() - T(x.x());
    residual[1] = p.y() / p.z() - T(x.y());
    return true;
  }

  Mat3 R;
  Vec3 t;
  Vec2 x;
};

// DLT on normalized coordinates; nullopt for a degenerate configuration.
std::optional<Vec3> dlt(
  const std::vector<Pose3> & poses, const std::vector<TrackObservation> & observations,
  const std::vector<int> & selected)
{
  Eigen::MatrixXd A(2 * selected.size(), 4);
  for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(selected.size()); ++k) {
    const TrackObservation & o = observations[selected[k]];
    const Eigen::Matrix<double, 3, 4> P = poses[o.view].affine();
    A.row(2 * k) = o.normalized.x() * P.row(2) - P.row(0);
    A.row(2 * k + 1) = o.normalized.y() * P.row(2) - P.row(1);
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  const Eigen::Vector4d X = svd.matrixV().col(3);
  if (std::abs(X(3)) < 1e-12 || !X.allFinite()) {
    return std::nullopt;
  }
  return X.head<3>() / X(3);
}

// Reprojection error in pixels; infinite behind the camera.
double errorPx(const Pose3 & T_cam_map, const Vec2 & x, const Vec3 & X, double focal)
{
  const Vec3 p = T_cam_map * X;
  if (p.z() <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return (p.hnormalized() - x).norm() * focal;
}

Vec3 refine(
  const std::vector<Pose3> & poses, const std::vector<TrackObservation> & observations,
  const std::vector<int> & selected, const Vec3 & initial, const TriangulationParams & params)
{
  double point[3] = {initial.x(), initial.y(), initial.z()};
  ceres::Problem problem;
  for (int i : selected) {
    const TrackObservation & o = observations[i];
    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(
        new ReprojectionError(poses[o.view], o.normalized)),
      new ceres::HuberLoss(params.huber_px / params.focal_px), point);
  }
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 20;
  options.logging_type = ceres::SILENT;
  options.num_threads = 1;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  const Vec3 refined(point[0], point[1], point[2]);
  return summary.IsSolutionUsable() && refined.allFinite() ? refined : initial;
}

}  // namespace

TriangulationResult triangulateTrack(
  const std::vector<Pose3> & T_cam_map, const std::vector<TrackObservation> & observations,
  const TriangulationParams & params)
{
  TriangulationResult result;
  const int n = static_cast<int>(observations.size());
  if (n < std::max(params.min_track_length, 2)) {
    return result;
  }

  // RANSAC over pairs (exhaustive; tracks are short): the pair whose two-view point explains
  // the most observations seeds the inlier set.
  std::vector<int> best_set;
  double best_error = std::numeric_limits<double>::infinity();
  for (int a = 0; a < n; ++a) {
    for (int b = a + 1; b < n; ++b) {
      if (observations[a].view == observations[b].view) {
        continue;
      }
      const auto X = dlt(T_cam_map, observations, {a, b});
      if (!X) {
        continue;
      }
      std::vector<int> set;
      double total = 0.0;
      for (int i = 0; i < n; ++i) {
        const double e =
          errorPx(T_cam_map[observations[i].view], observations[i].normalized, *X, params.focal_px);
        if (e <= params.max_reprojection_px) {
          set.push_back(i);
          total += e;
        }
      }
      if (set.size() > best_set.size() || (set.size() == best_set.size() && total < best_error)) {
        best_set = std::move(set);
        best_error = total;
      }
    }
  }

  // Refit on the set, then drop the worst observation while any is out of tolerance.
  std::vector<int> selected = best_set;
  Vec3 X = Vec3::Zero();
  while (true) {
    if (static_cast<int>(selected.size()) < params.min_track_length || selected.size() < 2) {
      return result;
    }
    const auto initial = dlt(T_cam_map, observations, selected);
    if (!initial) {
      return result;
    }
    X = params.refine ? refine(T_cam_map, observations, selected, *initial, params) : *initial;
    int worst = -1;
    double worst_error = 0.0;
    for (std::size_t k = 0; k < selected.size(); ++k) {
      const TrackObservation & o = observations[selected[k]];
      const double e = errorPx(T_cam_map[o.view], o.normalized, X, params.focal_px);
      if (e > worst_error) {
        worst_error = e;
        worst = static_cast<int>(k);
      }
    }
    if (worst_error <= params.max_reprojection_px) {
      break;
    }
    selected.erase(selected.begin() + worst);
  }

  result.position = X;
  result.inliers = selected;
  double sum = 0.0;
  double max_angle = 0.0;
  bool depth_ok = true;
  for (std::size_t k = 0; k < selected.size(); ++k) {
    const TrackObservation & o = observations[selected[k]];
    const Pose3 & pose = T_cam_map[o.view];
    sum += errorPx(pose, o.normalized, X, params.focal_px);
    const double depth = (pose * X).z();
    depth_ok = depth_ok && depth >= params.min_depth && depth <= params.max_depth;
    const Vec3 ray_k = X - pose.inverse().translation();
    for (std::size_t j = k + 1; j < selected.size(); ++j) {
      const Vec3 ray_j = X - T_cam_map[observations[selected[j]].view].inverse().translation();
      const double c = std::clamp(ray_k.normalized().dot(ray_j.normalized()), -1.0, 1.0);
      max_angle = std::max(max_angle, std::acos(c) * 180.0 / M_PI);
    }
  }
  result.mean_reprojection_px = sum / static_cast<double>(selected.size());
  result.max_angle_deg = max_angle;
  if (!depth_ok) {
    result.status = TriangulationStatus::kDepthOutOfRange;
  } else if (max_angle < params.min_angle_deg) {
    result.status = TriangulationStatus::kSmallAngle;
  } else if (X.z() < params.min_height) {
    result.status = TriangulationStatus::kBelowFloor;
  } else {
    result.status = TriangulationStatus::kOk;
  }
  return result;
}

}  // namespace landfall
