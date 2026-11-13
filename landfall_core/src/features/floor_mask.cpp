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

#include "landfall/features/floor_mask.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace landfall
{

cv::Mat floorMask(const CameraModel & camera, const Pose3 & T_base_cam, double max_distance)
{
  const double height = T_base_cam.translation().z();
  if (!(max_distance > 0.0) || !(height > 0.0) || camera.width() <= 0 || camera.height() <= 0) {
    return {};
  }
  const Mat3 rotation = T_base_cam.rotation();
  // A ray r (base frame) meets the floor at the horizontal distance height * |r_xy| / -r_z, so it
  // is hidden when -r_z > |r_xy| * height / max_distance.
  const double slope = height / max_distance;
  const auto hidden = [slope](const Vec3 & ray) { return -ray.z() > slope * ray.head<2>().norm(); };
  const auto ray = [&](double u, double v, Vec3 * out) {
    Vec2 normalized;
    if (!camera.undistort(Vec2(u, v), &normalized)) {
      return false;
    }
    *out = rotation * Vec3(normalized.x(), normalized.y(), 1.0);
    return true;
  };

  // Rays on a coarse grid, interpolated in between: the undistortion is smooth (and the rays are
  // exactly linear in the pixel without distortion), and inverting it for every pixel would take
  // about 0.1 s per image. Pixels next to a grid ray that doesn't exist are computed exactly.
  constexpr int kStep = 8;
  const int width = camera.width();
  const int rows = camera.height();
  const int grid_cols = (width - 1) / kStep + 2;
  const int grid_rows = (rows - 1) / kStep + 2;
  std::vector<Vec3> grid(static_cast<std::size_t>(grid_cols) * static_cast<std::size_t>(grid_rows));
  std::vector<char> valid(grid.size(), 0);
  for (int gy = 0; gy < grid_rows; ++gy) {
    for (int gx = 0; gx < grid_cols; ++gx) {
      const std::size_t i = static_cast<std::size_t>(gy) * grid_cols + gx;
      valid[i] = ray(gx * kStep, gy * kStep, &grid[i]) ? 1 : 0;
    }
  }

  cv::Mat mask(rows, width, CV_8UC1, cv::Scalar(255));
  int hidden_pixels = 0;
  for (int v = 0; v < rows; ++v) {
    const int gy = v / kStep;
    const double fy = static_cast<double>(v - gy * kStep) / kStep;
    auto * line = mask.ptr<uint8_t>(v);
    for (int u = 0; u < width; ++u) {
      const int gx = u / kStep;
      const double fx = static_cast<double>(u - gx * kStep) / kStep;
      const std::size_t i00 = static_cast<std::size_t>(gy) * grid_cols + gx;
      const std::size_t i10 = i00 + 1;
      const std::size_t i01 = i00 + grid_cols;
      const std::size_t i11 = i01 + 1;
      Vec3 r;
      if (valid[i00] != 0 && valid[i10] != 0 && valid[i01] != 0 && valid[i11] != 0) {
        r = (1.0 - fy) * ((1.0 - fx) * grid[i00] + fx * grid[i10]) +
            fy * ((1.0 - fx) * grid[i01] + fx * grid[i11]);
      } else if (!ray(u, v, &r)) {
        continue;
      }
      if (hidden(r)) {
        line[u] = 0;
        ++hidden_pixels;
      }
    }
  }
  return hidden_pixels > 0 ? mask : cv::Mat();
}

cv::Mat detectionMask(
  const std::string & backend, double max_distance, cv::Size size, const ImageGeometry & geometry)
{
  if (!(max_distance > 0.0)) {
    return {};
  }
  if (geometry.camera == nullptr) {
    throw std::invalid_argument(
      backend + ": " + kFloorMaskOption + " needs the camera the image was taken with");
  }
  if (geometry.camera->width() != size.width || geometry.camera->height() != size.height) {
    throw std::invalid_argument(
      backend + ": the image is " + std::to_string(size.width) + "x" + std::to_string(size.height) +
      ", its camera " + std::to_string(geometry.camera->width()) + "x" +
      std::to_string(geometry.camera->height()));
  }
  return floorMask(*geometry.camera, geometry.T_base_cam, max_distance);
}

}  // namespace landfall
