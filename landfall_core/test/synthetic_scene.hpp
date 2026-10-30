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

// Synthetic ground-robot geometry shared by the geometry tests.

#ifndef LANDFALL_CORE_SYNTHETIC_SCENE_HPP_
#define LANDFALL_CORE_SYNTHETIC_SCENE_HPP_

#include <cmath>
#include <random>

#include "landfall/types.hpp"

namespace landfall::test
{

constexpr double kFocal = 500.0;
constexpr int kWidth = 640;
constexpr int kHeight = 480;

inline double deg(double degrees) { return degrees * M_PI / 180.0; }

/// Optical frame (z forward, x right, y down) relative to a base frame (x forward, z up),
/// mounted at 1 m height, 0.2 m ahead of the base origin, pitched down by `pitch_down`.
inline Pose3 baseToCamera(double pitch_down = deg(5.0))
{
  Mat3 R_base_optical;
  R_base_optical << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Pose3 T = Pose3::Identity();
  T.linear() = Eigen::AngleAxisd(pitch_down, Vec3::UnitY()).toRotationMatrix() * R_base_optical;
  T.translation() = Vec3(0.2, 0.0, 1.0);
  return T;
}

/// Base pose in the map: planar pose plus optional pitch (a ramp) and roll.
inline Pose3 basePose(double x, double y, double yaw, double pitch = 0.0, double roll = 0.0)
{
  Pose3 T = Pose3::Identity();
  T.linear() = (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) * Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
                Eigen::AngleAxisd(roll, Vec3::UnitX()))
                 .toRotationMatrix();
  T.translation() = Vec3(x, y, 0.0);
  return T;
}

/// Camera center error (m) and rotation error (rad) between two map->camera transforms.
inline double centerError(const Pose3 & T_cam_map, const Pose3 & truth)
{
  return (T_cam_map.inverse().translation() - truth.inverse().translation()).norm();
}

inline double rotationError(const Pose3 & T_cam_map, const Pose3 & truth)
{
  return rotationAngle(T_cam_map.linear() * truth.linear().transpose());
}

}  // namespace landfall::test

#endif  // LANDFALL_CORE_SYNTHETIC_SCENE_HPP_
