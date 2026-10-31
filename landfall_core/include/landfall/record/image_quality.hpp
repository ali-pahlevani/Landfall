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

#ifndef LANDFALL_RECORD_IMAGE_QUALITY_HPP_
#define LANDFALL_RECORD_IMAGE_QUALITY_HPP_

#include <opencv2/core.hpp>
#include <string>

namespace landfall
{

struct ImageQuality
{
  /// Variance of the Laplacian of the grayscale image, downscaled to 640 px width when wider
  /// (so the threshold doesn't depend on the camera resolution).
  double sharpness = 0.0;
  double mean_intensity = 0.0;      ///< 0..255
  double saturated_fraction = 0.0;  ///< pixels at 250 or brighter
};

/// Converts BGR, BGRA or grayscale 8-bit images to grayscale (a no-op for grayscale).
cv::Mat toGray(const cv::Mat & image);

/// Measures an 8-bit grayscale image.
ImageQuality measureImageQuality(const cv::Mat & gray);

/// Compact JSON object of the measurements, stored per keyframe in the database.
std::string toJson(const ImageQuality & quality);

}  // namespace landfall

#endif  // LANDFALL_RECORD_IMAGE_QUALITY_HPP_
