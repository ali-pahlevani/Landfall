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

#include "landfall/geometry/pnp.hpp"

#include <PoseLib/camera_pose.h>
#include <PoseLib/robust/bundle.h>
#include <PoseLib/robust/ransac.h>
#include <PoseLib/robust/ransac_impl.h>
#include <PoseLib/robust/sampling.h>
#include <PoseLib/robust/utils.h>
#include <PoseLib/solvers/up2p.h>
#include <PoseLib/types.h>

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>

#include "landfall/geometry/gravity.hpp"

namespace landfall
{
namespace
{

using poselib::CameraPose;

const Vec3 kGravityMap(0.0, 0.0, -1.0);

double normalizedThreshold(const PnpOptions & options)
{
  return options.threshold_px / options.focal_px;
}

poselib::RansacOptions ransacOptions(const PnpOptions & options)
{
  poselib::RansacOptions ransac;
  ransac.max_iterations = static_cast<std::size_t>(std::max(options.max_iterations, 1));
  ransac.min_iterations =
    static_cast<std::size_t>(std::clamp(options.min_iterations, 0, options.max_iterations));
  ransac.success_prob = options.success_prob;
  ransac.max_reproj_error = normalizedThreshold(options);
  ransac.seed = options.seed;
  return ransac;
}

Mat3 yRotation(double theta) { return Eigen::AngleAxisd(theta, Vec3::UnitY()).toRotationMatrix(); }

bool finite(const CameraPose & pose) { return pose.q.allFinite() && pose.t.allFinite(); }

// Local optimization of the upright model: yaw about gravity and translation (4 DoF), Cauchy
// loss, Levenberg-Marquardt over the current inliers. R = Rc^T * Ry(theta) * Rw keeps gravity.
void refineUpright(
  const std::vector<Vec2> & x, const std::vector<Vec3> & X, const Mat3 & Rc, const Mat3 & Rw,
  double threshold, CameraPose * pose)
{
  const Mat3 M = Rc * pose->R() * Rw.transpose();
  double theta = std::atan2(M(0, 2), M(0, 0));
  Vec3 t = pose->t;
  const double sq_threshold = threshold * threshold;

  std::vector<int> inliers;
  for (std::size_t i = 0; i < x.size(); ++i) {
    const Vec3 p = pose->apply(X[i]);
    if (p.z() > 0.0 && (p.hnormalized() - x[i]).squaredNorm() < sq_threshold) {
      inliers.push_back(static_cast<int>(i));
    }
  }
  if (inliers.size() < 3) {
    return;
  }
  const auto cost = [&](double th, const Vec3 & tr) {
    const Mat3 R = Rc.transpose() * yRotation(th) * Rw;
    double total = 0.0;
    for (int i : inliers) {
      const Vec3 p = R * X[i] + tr;
      const double r2 = p.z() > 0.0 ? (p.hnormalized() - x[i]).squaredNorm() : 1e6 * sq_threshold;
      total += std::log1p(r2 / sq_threshold);
    }
    return total;
  };

  double lambda = 1e-3;
  double current = cost(theta, t);
  for (int iteration = 0; iteration < 10; ++iteration) {
    const Mat3 Ry = yRotation(theta);
    Mat3 dRy;
    dRy << -std::sin(theta), 0.0, std::cos(theta), 0.0, 0.0, 0.0, -std::cos(theta), 0.0,
      -std::sin(theta);
    const Mat3 R = Rc.transpose() * Ry * Rw;
    const Mat3 dR = Rc.transpose() * dRy * Rw;
    Eigen::Matrix4d JtJ = Eigen::Matrix4d::Zero();
    Eigen::Vector4d Jtr = Eigen::Vector4d::Zero();
    for (int i : inliers) {
      const Vec3 p = R * X[i] + t;
      if (p.z() <= 0.0) {
        continue;
      }
      const Vec2 r = p.hnormalized() - x[i];
      const double weight = 1.0 / (1.0 + r.squaredNorm() / sq_threshold);
      Eigen::Matrix<double, 2, 3> dproj;
      dproj << 1.0 / p.z(), 0.0, -p.x() / (p.z() * p.z()), 0.0, 1.0 / p.z(),
        -p.y() / (p.z() * p.z());
      Eigen::Matrix<double, 2, 4> J;
      J.col(0) = dproj * (dR * X[i]);
      J.rightCols<3>() = dproj;
      JtJ += weight * J.transpose() * J;
      Jtr += weight * J.transpose() * r;
    }
    bool improved = false;
    for (int attempt = 0; attempt < 5 && !improved; ++attempt) {
      Eigen::Matrix4d A = JtJ;
      A.diagonal() *= 1.0 + lambda;
      const Eigen::Vector4d step = A.ldlt().solve(-Jtr);
      if (!step.allFinite()) {
        return;
      }
      const double candidate_theta = theta + step(0);
      const Vec3 candidate_t = t + step.tail<3>();
      const double candidate = cost(candidate_theta, candidate_t);
      if (candidate < current) {
        theta = candidate_theta;
        t = candidate_t;
        current = candidate;
        lambda = std::max(lambda * 0.1, 1e-10);
        improved = true;
        if (step.norm() < 1e-12) {
          iteration = 10;
        }
      } else {
        lambda *= 10.0;
      }
    }
    if (!improved) {
      break;
    }
  }
  const Mat3 R = Rc.transpose() * yRotation(theta) * Rw;
  *pose = CameraPose(R, t);
}

// Estimator for poselib::ransac (see PoseLib/robust/ransac_impl.h for the interface).
class UprightEstimator
{
public:
  UprightEstimator(
    const poselib::RansacOptions & options, const std::vector<Vec2> & x,
    const std::vector<Vec3> & X, const Vec3 & gravity_cam)
  : num_data(x.size()),
    options_(options),
    x_(x),
    X_(X),
    gravity_cam_(gravity_cam.normalized()),
    Rc_(rotationBetween(gravity_cam_, Vec3::UnitY())),
    Rw_(rotationBetween(kGravityMap, Vec3::UnitY())),
    sampler_(num_data, sample_sz, options.seed, false, 0)
  {
    xs_.resize(sample_sz);
    Xs_.resize(sample_sz);
    sample_.resize(sample_sz);
  }

  void generate_models(std::vector<CameraPose> * models)
  {
    sampler_.generate_sample(&sample_);
    for (std::size_t k = 0; k < sample_sz; ++k) {
      xs_[k] = x_[sample_[k]].homogeneous().normalized();
      Xs_[k] = X_[sample_[k]];
    }
    models->clear();
    poselib::up2p(xs_, Xs_, gravity_cam_, kGravityMap, models);
    models->erase(
      std::remove_if(
        models->begin(), models->end(), [](const CameraPose & p) { return !finite(p); }),
      models->end());
  }

  double score_model(const CameraPose & pose, std::size_t * inlier_count) const
  {
    return poselib::compute_msac_score(
      pose, x_, X_, options_.max_reproj_error * options_.max_reproj_error, inlier_count);
  }

  void refine_model(CameraPose * pose) const
  {
    refineUpright(x_, X_, Rc_, Rw_, options_.max_reproj_error, pose);
  }

  const std::size_t sample_sz = 2;
  const std::size_t num_data;

private:
  const poselib::RansacOptions & options_;
  const std::vector<Vec2> & x_;
  const std::vector<Vec3> & X_;
  Vec3 gravity_cam_;
  Mat3 Rc_;
  Mat3 Rw_;
  poselib::RandomSampler sampler_;
  std::vector<Vec3> xs_;
  std::vector<Vec3> Xs_;
  std::vector<std::size_t> sample_;
};

std::vector<int> inlierIndices(
  const CameraPose & pose, const Correspondences & c, double sq_threshold)
{
  std::vector<char> mask;
  poselib::get_inliers(pose, c.normalized, c.points, sq_threshold, &mask);
  std::vector<int> inliers;
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (mask[i] != 0) {
      inliers.push_back(static_cast<int>(i));
    }
  }
  return inliers;
}

// Final unconstrained refinement and inlier bookkeeping shared by every method.
std::optional<PnpSolution> finish(
  const Correspondences & c, CameraPose pose, const PnpOptions & options, PnpMethod method,
  int iterations)
{
  if (!finite(pose)) {
    return std::nullopt;
  }
  const double threshold = normalizedThreshold(options);
  std::vector<int> inliers = inlierIndices(pose, c, threshold * threshold);
  if (options.final_refinement) {
    // Twice: the refined pose may recover inliers the RANSAC model missed.
    for (int pass = 0; pass < 2 && inliers.size() >= 4; ++pass) {
      std::vector<Vec2> x;
      std::vector<Vec3> X;
      for (int i : inliers) {
        x.push_back(c.normalized[i]);
        X.push_back(c.points[i]);
      }
      poselib::BundleOptions bundle;
      bundle.loss_type = poselib::BundleOptions::LossType::CAUCHY;
      bundle.loss_scale = threshold;
      bundle.max_iterations = 100;
      CameraPose refined = pose;
      poselib::bundle_adjust(x, X, &refined, bundle);
      if (!finite(refined)) {
        break;
      }
      pose = refined;
      const std::vector<int> updated = inlierIndices(pose, c, threshold * threshold);
      const bool same = updated == inliers;
      inliers = updated;
      if (same) {
        break;
      }
    }
  }
  if (inliers.size() < 4) {
    return std::nullopt;
  }
  PnpSolution solution;
  solution.T_cam_map.linear() = pose.R();
  solution.T_cam_map.translation() = pose.t;
  solution.inliers = std::move(inliers);
  solution.method = method;
  solution.iterations = iterations;
  double sum = 0.0;
  for (int i : solution.inliers) {
    sum += (pose.apply(c.points[i]).hnormalized() - c.normalized[i]).norm();
  }
  solution.mean_reprojection_px =
    sum / static_cast<double>(solution.inliers.size()) * options.focal_px;
  return solution;
}

bool valid(const Correspondences & c, std::size_t minimum)
{
  return c.normalized.size() == c.points.size() && c.normalized.size() >= minimum;
}

}  // namespace

std::optional<PnpStrategy> parsePnpStrategy(const std::string & name)
{
  if (name == "upright") {
    return PnpStrategy::kUpright;
  }
  if (name == "p3p") {
    return PnpStrategy::kP3P;
  }
  if (name == "upright_then_p3p") {
    return PnpStrategy::kUprightThenP3P;
  }
  if (name == "opencv") {
    return PnpStrategy::kOpenCv;
  }
  return std::nullopt;
}

std::optional<PnpSolution> solveUprightPnp(
  const Correspondences & correspondences, const Vec3 & gravity_cam, const PnpOptions & options)
{
  if (!valid(correspondences, 4) || !(gravity_cam.norm() > 0.0)) {
    return std::nullopt;
  }
  const poselib::RansacOptions ransac = ransacOptions(options);
  UprightEstimator estimator(
    ransac, correspondences.normalized, correspondences.points, gravity_cam);
  CameraPose pose;
  const poselib::RansacStats stats = poselib::ransac<UprightEstimator>(estimator, ransac, &pose);
  if (stats.num_inliers < 4) {
    return std::nullopt;
  }
  return finish(
    correspondences, pose, options, PnpMethod::kUpright, static_cast<int>(stats.iterations));
}

std::optional<PnpSolution> solveP3P(
  const Correspondences & correspondences, const PnpOptions & options)
{
  if (!valid(correspondences, 4)) {
    return std::nullopt;
  }
  CameraPose pose;
  std::vector<char> mask;
  const poselib::RansacStats stats = poselib::ransac_pnp(
    correspondences.normalized, correspondences.points, ransacOptions(options), &pose, &mask);
  if (stats.num_inliers < 4) {
    return std::nullopt;
  }
  return finish(
    correspondences, pose, options, PnpMethod::kP3P, static_cast<int>(stats.iterations));
}

std::optional<PnpSolution> solveOpenCvPnp(
  const Correspondences & correspondences, const PnpOptions & options)
{
  if (!valid(correspondences, 6)) {
    return std::nullopt;
  }
  std::vector<cv::Point3d> object;
  std::vector<cv::Point2d> image;
  for (std::size_t i = 0; i < correspondences.points.size(); ++i) {
    const Vec3 & X = correspondences.points[i];
    const Vec2 & x = correspondences.normalized[i];
    object.emplace_back(X.x(), X.y(), X.z());
    image.emplace_back(x.x() * options.focal_px, x.y() * options.focal_px);
  }
  cv::Mat K = (cv::Mat_<double>(3, 3) << options.focal_px, 0, 0, 0, options.focal_px, 0, 0, 0, 1);
  cv::UsacParams params;
  params.threshold = options.threshold_px;
  params.confidence = options.success_prob;
  params.maxIterations = options.max_iterations;
  params.randomGeneratorState = static_cast<int>(options.seed & 0x7fffffff);
  cv::Mat rvec;
  cv::Mat tvec;
  std::vector<int> inliers;
  if (!cv::solvePnPRansac(object, image, K, cv::noArray(), rvec, tvec, inliers, params)) {
    return std::nullopt;
  }
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Mat3 rotation;
  for (int r = 0; r < 3; ++r) {
    for (int col = 0; col < 3; ++col) {
      rotation(r, col) = R.at<double>(r, col);
    }
  }
  const CameraPose pose(rotation, Vec3(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)));
  return finish(correspondences, pose, options, PnpMethod::kOpenCv, options.max_iterations);
}

std::optional<PnpSolution> solvePnp(
  PnpStrategy strategy, const Correspondences & correspondences, const Vec3 & gravity_cam,
  const PnpOptions & options, int min_inliers)
{
  switch (strategy) {
    case PnpStrategy::kUpright:
      return solveUprightPnp(correspondences, gravity_cam, options);
    case PnpStrategy::kP3P:
      return solveP3P(correspondences, options);
    case PnpStrategy::kOpenCv:
      return solveOpenCvPnp(correspondences, options);
    case PnpStrategy::kUprightThenP3P:
      break;
  }
  auto upright = solveUprightPnp(correspondences, gravity_cam, options);
  if (upright && static_cast<int>(upright->inliers.size()) >= min_inliers) {
    return upright;
  }
  auto p3p = solveP3P(correspondences, options);
  if (!p3p) {
    return upright;
  }
  if (!upright || p3p->inliers.size() > upright->inliers.size()) {
    return p3p;
  }
  return upright;
}

}  // namespace landfall
