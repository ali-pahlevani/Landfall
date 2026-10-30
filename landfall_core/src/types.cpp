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

#include "landfall/types.hpp"

#include <algorithm>
#include <cmath>

namespace landfall
{

double normalizeAngle(double angle)
{
  angle = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (angle < 0.0) {
    angle += 2.0 * M_PI;
  }
  return angle - M_PI;
}

Pose2 compose(const Pose2 & a, const Pose2 & b)
{
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, normalizeAngle(a.yaw + b.yaw)};
}

Pose2 inverse(const Pose2 & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {-c * pose.x - s * pose.y, s * pose.x - c * pose.y, normalizeAngle(-pose.yaw)};
}

Pose2 between(const Pose2 & a, const Pose2 & b) { return compose(inverse(a), b); }

Pose3 toPose3(const Pose2 & pose)
{
  Pose3 result = Pose3::Identity();
  result.linear() = Eigen::AngleAxisd(pose.yaw, Vec3::UnitZ()).toRotationMatrix();
  result.translation() = Vec3(pose.x, pose.y, 0.0);
  return result;
}

Pose2 toPose2(const Pose3 & pose)
{
  const Vec3 heading = pose.linear().col(0);
  return {pose.translation().x(), pose.translation().y(), std::atan2(heading.y(), heading.x())};
}

double translationDistance(const Pose2 & a, const Pose2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double rotationDistance(const Pose2 & a, const Pose2 & b)
{
  return std::abs(normalizeAngle(a.yaw - b.yaw));
}

double rotationAngle(const Mat3 & rotation)
{
  const double c = std::clamp((rotation.trace() - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(c);
}

}  // namespace landfall
