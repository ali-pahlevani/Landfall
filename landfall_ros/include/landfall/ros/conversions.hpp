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

#ifndef LANDFALL_ROS_CONVERSIONS_HPP_
#define LANDFALL_ROS_CONVERSIONS_HPP_

#include <cstdint>
#include <opencv2/core.hpp>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "landfall/camera_model.hpp"
#include "landfall/types.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace landfall::ros
{

int64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp);
builtin_interfaces::msg::Time toStamp(int64_t nanoseconds);

Pose3 toPose3(const geometry_msgs::msg::Transform & transform);
Pose3 toPose3(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose toPoseMsg(const Pose3 & pose);

/// x, y, yaw block of a 6x6 row-major pose covariance (x y z roll pitch yaw).
Mat3 planarCovariance(const geometry_msgs::msg::PoseWithCovariance::_covariance_type & covariance);

/// Camera model of a CameraInfo (K and D; the rectification and projection matrices are not
/// used: images are processed unrectified). Throws std::invalid_argument for an unsupported or
/// inconsistent CameraInfo.
CameraModel toCameraModel(const sensor_msgs::msg::CameraInfo & info);

/// 8-bit BGR or grayscale image of a raw image message (bgr8, rgb8, bgra8, rgba8, mono8, mono16
/// and the Bayer encodings cv_bridge converts). Throws std::runtime_error otherwise.
cv::Mat decodeImage(const sensor_msgs::msg::Image & image);
/// Image of a CompressedImage (JPEG or PNG, as published by compressed_image_transport).
cv::Mat decodeImage(const sensor_msgs::msg::CompressedImage & image);

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_CONVERSIONS_HPP_
