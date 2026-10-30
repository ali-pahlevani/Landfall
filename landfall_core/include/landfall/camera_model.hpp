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

#ifndef LANDFALL_CAMERA_MODEL_HPP_
#define LANDFALL_CAMERA_MODEL_HPP_

#include <optional>
#include <string>
#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

/// Distortion models of sensor_msgs/CameraInfo (same coefficient order as ROS and OpenCV).
enum class DistortionModel : uint8_t {
  kNone = 0,
  kPlumbBob = 1,            ///< k1, k2, p1, p2[, k3]
  kRationalPolynomial = 2,  ///< k1, k2, p1, p2, k3, k4, k5, k6
  kEquidistant = 3,         ///< k1, k2, k3, k4 (fisheye)
};

/// Parses a CameraInfo `distortion_model` ("plumb_bob", "rational_polynomial", "equidistant",
/// "none" or empty).
std::optional<DistortionModel> parseDistortionModel(const std::string & name);
std::string toString(DistortionModel model);

/// Pinhole camera with lens distortion. Image points are in pixels of the distorted image;
/// normalized coordinates are undistorted (x/z, y/z) in the optical frame (z forward, y down).
class CameraModel
{
public:
  CameraModel() = default;
  /// Throws std::invalid_argument for non-positive sizes or focal lengths, or a coefficient count
  /// the model doesn't accept. An empty coefficient list means no distortion.
  CameraModel(
    int width, int height, double fx, double fy, double cx, double cy, DistortionModel model,
    std::vector<double> coefficients);

  int width() const { return width_; }
  int height() const { return height_; }
  double fx() const { return fx_; }
  double fy() const { return fy_; }
  double cx() const { return cx_; }
  double cy() const { return cy_; }
  /// Mean focal length, used to convert pixel thresholds to normalized units.
  double focal() const { return 0.5 * (fx_ + fy_); }
  DistortionModel model() const { return model_; }
  const std::vector<double> & coefficients() const { return coefficients_; }

  /// Applies the lens distortion to normalized coordinates.
  Vec2 distort(const Vec2 & normalized) const;
  /// Projects a point in the optical frame to pixels. False if it isn't in front of the camera.
  bool project(const Vec3 & point, Vec2 * pixel) const;
  /// Pixel to undistorted normalized coordinates. False if the inversion doesn't converge.
  bool undistort(const Vec2 & pixel, Vec2 * normalized) const;
  bool contains(const Vec2 & pixel, double margin = 0.0) const;

private:
  int width_ = 0;
  int height_ = 0;
  double fx_ = 0.0;
  double fy_ = 0.0;
  double cx_ = 0.0;
  double cy_ = 0.0;
  DistortionModel model_ = DistortionModel::kNone;
  std::vector<double> coefficients_;
};

}  // namespace landfall

#endif  // LANDFALL_CAMERA_MODEL_HPP_
