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
#include <random>
#include <vector>

#include "landfall/camera_model.hpp"
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

}  // namespace landfall::test

#endif  // LANDFALL_CORE_SYNTHETIC_MAP_HPP_
