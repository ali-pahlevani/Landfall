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

#include "landfall/grid/occupancy_grid.hpp"

#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <stdexcept>

#include "landfall/util/sha256.hpp"

namespace landfall
{

bool OccupancyGrid::worldToCell(double x, double y, int * col, int * row) const
{
  if (std::isnan(resolution) || resolution <= 0.0) {
    return false;
  }
  const double fx = std::floor((x - origin.x) / resolution);
  const double fy = std::floor((y - origin.y) / resolution);
  if (std::isnan(fx) || std::isnan(fy) || fx < 0.0 || fy < 0.0 || fx >= width || fy >= height) {
    return false;
  }
  *col = static_cast<int>(fx);
  *row = static_cast<int>(fy);
  return true;
}

Vec2 OccupancyGrid::cellCenter(int col, int row) const
{
  return {origin.x + (col + 0.5) * resolution, origin.y + (row + 0.5) * resolution};
}

std::string gridSha256(const OccupancyGrid & grid)
{
  Sha256 sha;
  const auto width = static_cast<uint32_t>(grid.width);
  const auto height = static_cast<uint32_t>(grid.height);
  sha.update(&width, sizeof(width));
  sha.update(&height, sizeof(height));
  sha.update(grid.data.data(), grid.data.size());
  return sha.finishHex();
}

std::vector<uint8_t> encodeGridPng(const OccupancyGrid & grid)
{
  cv::Mat image(grid.height, grid.width, CV_8UC1);
  for (int row = 0; row < grid.height; ++row) {
    auto * pixels = image.ptr<uint8_t>(grid.height - 1 - row);
    for (int col = 0; col < grid.width; ++col) {
      pixels[col] = static_cast<uint8_t>(grid.at(col, row) + 1);
    }
  }
  std::vector<uint8_t> png;
  if (!cv::imencode(".png", image, png)) {
    throw std::runtime_error("failed to encode the occupancy grid as PNG");
  }
  return png;
}

OccupancyGrid decodeGridPng(
  const std::vector<uint8_t> & png, double resolution, const Pose2 & origin)
{
  const cv::Mat image = cv::imdecode(png, cv::IMREAD_UNCHANGED);
  if (image.empty() || image.type() != CV_8UC1) {
    throw std::runtime_error("stored occupancy grid is not an 8-bit single-channel PNG");
  }
  OccupancyGrid grid;
  grid.width = image.cols;
  grid.height = image.rows;
  grid.resolution = resolution;
  grid.origin = origin;
  grid.data.resize(static_cast<std::size_t>(grid.width) * grid.height);
  for (int row = 0; row < grid.height; ++row) {
    const auto * pixels = image.ptr<uint8_t>(grid.height - 1 - row);
    for (int col = 0; col < grid.width; ++col) {
      if (pixels[col] > kCellOccupied + 1) {
        throw std::runtime_error("stored occupancy grid has values outside [-1, 100]");
      }
      grid.data[static_cast<std::size_t>(row) * grid.width + col] =
        static_cast<int8_t>(pixels[col] - 1);
    }
  }
  return grid;
}

}  // namespace landfall
