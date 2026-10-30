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

#ifndef LANDFALL_TYPES_HPP_
#define LANDFALL_TYPES_HPP_

#include <Eigen/Geometry>
#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

namespace landfall
{

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// Rigid transform. `T_a_b` maps points from frame b into frame a: p_a = T_a_b * p_b.
using Pose3 = Eigen::Isometry3d;

/// Planar pose in the map plane: x, y in metres, yaw in radians.
struct Pose2
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

/// Wraps an angle to [-pi, pi).
double normalizeAngle(double angle);
/// a ⊕ b.
Pose2 compose(const Pose2 & a, const Pose2 & b);
Pose2 inverse(const Pose2 & pose);
/// a⁻¹ ⊕ b: pose b expressed in frame a.
Pose2 between(const Pose2 & a, const Pose2 & b);
/// Lifts a planar pose to 3D (z = 0, rotation about z).
Pose3 toPose3(const Pose2 & pose);
/// Projects a 3D pose onto the plane: position x/y and the heading of its x axis.
Pose2 toPose2(const Pose3 & pose);
/// Translation and absolute rotation difference between two planar poses.
double translationDistance(const Pose2 & a, const Pose2 & b);
double rotationDistance(const Pose2 & a, const Pose2 & b);
/// Rotation angle of a 3D rotation, in radians.
double rotationAngle(const Mat3 & rotation);

/// How a descriptor matrix stores its rows.
enum class DescriptorType : uint8_t {
  kBinary = 0,   ///< CV_8U bit strings, Hamming distance (ORB)
  kFloat32 = 1,  ///< CV_32F, L2-normalized rows (SIFT, learned)
};

struct Keypoint
{
  float x = 0.0F;  ///< column in the (distorted) image, pixels
  float y = 0.0F;  ///< row, pixels
  float size = 0.0F;
  float angle = 0.0F;
  float score = 0.0F;
};

struct Features
{
  std::vector<Keypoint> keypoints;
  cv::Mat descriptors;  ///< one row per keypoint
  DescriptorType type = DescriptorType::kBinary;
};

struct Match
{
  int query = 0;  ///< row in the query features
  int train = 0;  ///< row in the database features
  float distance = 0.0F;
};
using Matches = std::vector<Match>;

}  // namespace landfall

#endif  // LANDFALL_TYPES_HPP_
