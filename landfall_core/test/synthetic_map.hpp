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

// A synthetic room for the map-building tests: textured walls (landmarks with ORB-like binary
// descriptors) and a robot driving a loop around the room in both directions.

#ifndef LANDFALL_CORE_SYNTHETIC_MAP_HPP_
#define LANDFALL_CORE_SYNTHETIC_MAP_HPP_

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "landfall/camera_model.hpp"
#include "landfall/grid/occupancy_grid.hpp"
#include "landfall/grid/scan_verifier.hpp"
#include "landfall/types.hpp"
#include "synthetic_scene.hpp"

namespace landfall::test
{

struct SyntheticRoom
{
  std::vector<Vec3> landmarks;
  cv::Mat descriptors;  ///< one 32-byte row per landmark
};

/// Landmarks on the four walls of the rectangle [x0, x1] x [y0, y1], 0.2 .. 2.5 m high.
inline SyntheticRoom makeRoom(int count, double x0, double y0, double x1, double y1, int seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  SyntheticRoom room;
  const double w = x1 - x0;
  const double h = y1 - y0;
  for (int i = 0; i < count; ++i) {
    const double s = unit(rng) * 2.0 * (w + h);  // position along the perimeter
    const double z = 0.2 + 2.3 * unit(rng);
    if (s < w) {
      room.landmarks.emplace_back(x0 + s, y0, z);
    } else if (s < w + h) {
      room.landmarks.emplace_back(x1, y0 + (s - w), z);
    } else if (s < 2.0 * w + h) {
      room.landmarks.emplace_back(x1 - (s - w - h), y1, z);
    } else {
      room.landmarks.emplace_back(x0, y1 - (s - 2.0 * w - h), z);
    }
  }
  room.descriptors.create(count, 32, CV_8UC1);
  cv::RNG(static_cast<uint64_t>(seed) + 1).fill(room.descriptors, cv::RNG::UNIFORM, 0, 256);
  return room;
}

/// Base poses every `spacing` metres around the rectangle [x0, x1] x [y0, y1], first
/// counter-clockwise, then clockwise.
inline std::vector<Pose2> loopBothWays(double x0, double y0, double x1, double y1, double spacing)
{
  std::vector<Pose2> corners = {
    {x0, y0, 0.0}, {x1, y0, M_PI / 2}, {x1, y1, M_PI}, {x0, y1, -M_PI / 2}};
  std::vector<Pose2> path;
  const auto drive = [&](const std::vector<Pose2> & loop) {
    for (std::size_t c = 0; c < loop.size(); ++c) {
      const Pose2 & a = loop[c];
      const Pose2 & b = loop[(c + 1) % loop.size()];
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      const double heading = std::atan2(b.y - a.y, b.x - a.x);
      for (int step = 0; step * spacing < length - 1e-9; ++step) {
        const double d = step * spacing;
        path.push_back(
          {a.x + (b.x - a.x) * d / length, a.y + (b.y - a.y) * d / length,
           normalizeAngle(heading)});
      }
    }
  };
  drive(corners);
  drive(std::vector<Pose2>(corners.rbegin(), corners.rend()));
  return path;
}

inline CameraModel syntheticCamera()
{
  return {kWidth, kHeight, kFocal, kFocal, 319.5, 239.5, DistortionModel::kNone, {}};
}

/// Pixel of a map point in a camera, if it is in front (0.3 .. 15 m) and inside the image.
inline bool projectPoint(
  const CameraModel & camera, const Pose3 & T_cam_map, const Vec3 & X, Vec2 * pixel)
{
  const Vec3 p = T_cam_map * X;
  if (p.z() < 0.3 || p.z() > 15.0) {
    return false;
  }
  return camera.project(p, pixel) && camera.contains(*pixel, 2.0);
}

/// Marks the walls of the rectangle [x0, x1] x [y0, y1] occupied (two cells thick) and its inside
/// free, leaving the rest of the grid as it is.
inline void addRoomToGrid(OccupancyGrid * grid, double x0, double y0, double x1, double y1)
{
  for (int row = 0; row < grid->height; ++row) {
    for (int col = 0; col < grid->width; ++col) {
      const Vec2 c = grid->cellCenter(col, row);
      const double half = grid->resolution;
      const bool inside_outer =
        c.x() >= x0 - half && c.x() <= x1 + half && c.y() >= y0 - half && c.y() <= y1 + half;
      const bool inside_inner =
        c.x() > x0 + half && c.x() < x1 - half && c.y() > y0 + half && c.y() < y1 - half;
      auto & cell = grid->data[static_cast<std::size_t>(row) * grid->width + col];
      if (inside_inner) {
        cell = kCellFree;
      } else if (inside_outer) {
        cell = kCellOccupied;
      }
    }
  }
}

/// A 2D LiDAR scan ray-cast in the grid: `beams` beams over 360 degrees from the laser at
/// T_map_base * T_base_laser. A beam tilted down hits the floor (z = 0); otherwise its horizontal
/// march stops at the first occupied cell. Gaussian range noise; beams that hit nothing within
/// `max_range` come back as +inf.
inline PlanarScan simulateScan(
  const OccupancyGrid & grid, const Pose3 & T_map_base, const Pose3 & T_base_laser, int seed,
  int beams = 360, double max_range = 10.0, double noise = 0.01)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> range_noise(0.0, noise);
  PlanarScan scan;
  scan.angle_min = -M_PI;
  scan.angle_increment = 2.0 * M_PI / beams;
  scan.range_min = 0.05;
  scan.range_max = max_range;
  scan.T_base_laser = T_base_laser;
  const Pose3 T_map_laser = T_map_base * T_base_laser;
  const Vec3 origin = T_map_laser.translation();
  for (int b = 0; b < beams; ++b) {
    const double angle = scan.angle_min + b * scan.angle_increment;
    const Vec3 d = T_map_laser.linear() * Vec3(std::cos(angle), std::sin(angle), 0.0);
    double range = std::numeric_limits<double>::infinity();
    if (d.z() < -1e-9 && origin.z() > 0.0) {
      range = -origin.z() / d.z();
    }
    const double horizontal = std::hypot(d.x(), d.y());
    if (horizontal > 1e-9) {
      const double step = 0.25 * grid.resolution;
      for (double h = step; h / horizontal < std::min(range, max_range); h += step) {
        int col = 0;
        int row = 0;
        const double x = origin.x() + d.x() / horizontal * h;
        const double y = origin.y() + d.y() / horizontal * h;
        if (!grid.worldToCell(x, y, &col, &row)) {
          break;
        }
        if (grid.at(col, row) >= 65) {
          range = std::min(range, h / horizontal);
          break;
        }
      }
    }
    scan.ranges.push_back(
      range <= max_range ? static_cast<float>(range + range_noise(rng))
                         : std::numeric_limits<float>::infinity());
  }
  return scan;
}

}  // namespace landfall::test

#endif  // LANDFALL_CORE_SYNTHETIC_MAP_HPP_
