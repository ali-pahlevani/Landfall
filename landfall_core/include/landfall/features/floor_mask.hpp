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

#ifndef LANDFALL_FEATURES_FLOOR_MASK_HPP_
#define LANDFALL_FEATURES_FLOOR_MASK_HPP_

#include <opencv2/core.hpp>
#include <string>

#include "landfall/camera_model.hpp"
#include "landfall/interfaces/local_features.hpp"
#include "landfall/types.hpp"

namespace landfall
{

/// Option of every local feature backend: hide the floor nearer than this many metres from the
/// detector (0 or absent: detect everywhere). Seen from a robot, the floor's texture changes
/// shape with every viewpoint, so its keypoints rarely become landmarks but take most of the
/// keypoint budget.
inline constexpr const char * kFloorMaskOption = "floor_mask_distance";

/// Where keypoints may be detected in an image of `camera` mounted at `T_base_cam`: 0 where the
/// pixel's ray meets the floor (the base frame's z = 0 plane) nearer than `max_distance` to the
/// camera, measured along the floor; 255 elsewhere. Empty when nothing is hidden: `max_distance`
/// is 0, the camera is not above the floor, or no pixel sees the floor that near.
cv::Mat floorMask(const CameraModel & camera, const Pose3 & T_base_cam, double max_distance);

/// The detection mask of `backend` (named in errors) for an image `size` taken as `geometry`
/// describes, with the backend's floor_mask_distance `max_distance`. Throws
/// std::invalid_argument when the floor is to be hidden but the camera is unknown or of
/// another size than the image.
cv::Mat detectionMask(
  const std::string & backend, double max_distance, cv::Size size, const ImageGeometry & geometry);

}  // namespace landfall

#endif  // LANDFALL_FEATURES_FLOOR_MASK_HPP_
