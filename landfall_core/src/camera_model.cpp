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

#include "landfall/camera_model.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace landfall
{

std::optional<DistortionModel> parseDistortionModel(const std::string & name)
{
  if (name.empty() || name == "none") {
    return DistortionModel::kNone;
  }
  if (name == "plumb_bob") {
    return DistortionModel::kPlumbBob;
  }
  if (name == "rational_polynomial") {
    return DistortionModel::kRationalPolynomial;
  }
  if (name == "equidistant") {
    return DistortionModel::kEquidistant;
  }
  return std::nullopt;
}

std::string toString(DistortionModel model)
{
  switch (model) {
    case DistortionModel::kNone:
      return "none";
    case DistortionModel::kPlumbBob:
      return "plumb_bob";
    case DistortionModel::kRationalPolynomial:
      return "rational_polynomial";
    case DistortionModel::kEquidistant:
      return "equidistant";
  }
  return "unknown";
}

CameraModel::CameraModel(
  int width, int height, double fx, double fy, double cx, double cy, DistortionModel model,
  std::vector<double> coefficients)
: width_(width),
  height_(height),
  fx_(fx),
  fy_(fy),
  cx_(cx),
  cy_(cy),
  model_(model),
  coefficients_(std::move(coefficients))
{
  if (width <= 0 || height <= 0 || !(fx > 0.0) || !(fy > 0.0)) {
    throw std::invalid_argument("camera model: image size and focal lengths must be positive");
  }
  const std::size_t n = coefficients_.size();
  bool count_ok = n == 0;
  switch (model_) {
    case DistortionModel::kNone:
      break;
    case DistortionModel::kPlumbBob:
      count_ok = count_ok || n == 4 || n == 5;
      break;
    case DistortionModel::kRationalPolynomial:
      count_ok = count_ok || n == 8;
      break;
    case DistortionModel::kEquidistant:
      count_ok = count_ok || n == 4;
      break;
  }
  if (!count_ok) {
    throw std::invalid_argument(
      "camera model: " + std::to_string(n) + " distortion coefficients don't fit model " +
      toString(model_));
  }
  // Pad to the full length so the formulas below don't need bounds checks.
  if (model_ == DistortionModel::kPlumbBob) {
    coefficients_.resize(5, 0.0);
  } else if (model_ == DistortionModel::kRationalPolynomial) {
    coefficients_.resize(8, 0.0);
  } else if (model_ == DistortionModel::kEquidistant) {
    coefficients_.resize(4, 0.0);
  }
}

Vec2 CameraModel::distort(const Vec2 & p) const
{
  const auto & d = coefficients_;
  switch (model_) {
    case DistortionModel::kNone:
      return p;
    case DistortionModel::kPlumbBob:
    case DistortionModel::kRationalPolynomial: {
      const double x = p.x();
      const double y = p.y();
      const double r2 = x * x + y * y;
      const double r4 = r2 * r2;
      const double r6 = r4 * r2;
      double radial = 1.0 + d[0] * r2 + d[1] * r4 + d[4] * r6;
      if (model_ == DistortionModel::kRationalPolynomial) {
        radial /= 1.0 + d[5] * r2 + d[6] * r4 + d[7] * r6;
      }
      return {
        x * radial + 2.0 * d[2] * x * y + d[3] * (r2 + 2.0 * x * x),
        y * radial + d[2] * (r2 + 2.0 * y * y) + 2.0 * d[3] * x * y};
    }
    case DistortionModel::kEquidistant: {
      const double r = p.norm();
      if (r < 1e-12) {
        return p;
      }
      const double theta = std::atan(r);
      const double t2 = theta * theta;
      const double theta_d = theta * (1.0 + t2 * (d[0] + t2 * (d[1] + t2 * (d[2] + t2 * d[3]))));
      return p * (theta_d / r);
    }
  }
  return p;
}

bool CameraModel::project(const Vec3 & point, Vec2 * pixel) const
{
  if (!(point.z() > 1e-9)) {
    return false;
  }
  const Vec2 d = distort(Vec2(point.x() / point.z(), point.y() / point.z()));
  *pixel = Vec2(fx_ * d.x() + cx_, fy_ * d.y() + cy_);
  return true;
}

bool CameraModel::undistort(const Vec2 & pixel, Vec2 * normalized) const
{
  const Vec2 target((pixel.x() - cx_) / fx_, (pixel.y() - cy_) / fy_);
  if (model_ == DistortionModel::kNone) {
    *normalized = target;
    return true;
  }
  if (model_ == DistortionModel::kEquidistant) {
    // Solve theta_d(theta) = |target| for theta, then scale the direction by tan(theta).
    const auto & d = coefficients_;
    const double theta_d = target.norm();
    if (theta_d < 1e-12) {
      *normalized = target;
      return true;
    }
    double theta = theta_d;
    for (int i = 0; i < 20; ++i) {
      const double t2 = theta * theta;
      const double f =
        theta * (1.0 + t2 * (d[0] + t2 * (d[1] + t2 * (d[2] + t2 * d[3])))) - theta_d;
      const double df =
        1.0 + t2 * (3.0 * d[0] + t2 * (5.0 * d[1] + t2 * (7.0 * d[2] + t2 * 9.0 * d[3])));
      if (std::abs(df) < 1e-12) {
        return false;
      }
      const double step = f / df;
      theta -= step;
      if (std::abs(step) < 1e-15) {
        break;
      }
    }
    if (!std::isfinite(theta) || theta < 0.0 || theta >= M_PI_2) {
      return false;
    }
    *normalized = target * (std::tan(theta) / theta_d);
    return std::abs(distort(*normalized).x() - target.x()) < 1e-9 &&
           std::abs(distort(*normalized).y() - target.y()) < 1e-9;
  }
  // Newton iterations with a numerical Jacobian; the residual itself is exact.
  Vec2 p = target;
  constexpr double kStep = 1e-7;
  for (int i = 0; i < 30; ++i) {
    const Vec2 residual = distort(p) - target;
    if (residual.norm() < 1e-14) {
      break;
    }
    Eigen::Matrix2d jacobian;
    jacobian.col(0) = (distort(p + Vec2(kStep, 0.0)) - distort(p - Vec2(kStep, 0.0))) / (2 * kStep);
    jacobian.col(1) = (distort(p + Vec2(0.0, kStep)) - distort(p - Vec2(0.0, kStep))) / (2 * kStep);
    if (std::abs(jacobian.determinant()) < 1e-12) {
      return false;
    }
    p -= jacobian.inverse() * residual;
  }
  *normalized = p;
  return (distort(p) - target).norm() < 1e-9;
}

bool CameraModel::contains(const Vec2 & pixel, double margin) const
{
  return pixel.x() >= margin && pixel.y() >= margin && pixel.x() <= width_ - 1 - margin &&
         pixel.y() <= height_ - 1 - margin;
}

}  // namespace landfall
