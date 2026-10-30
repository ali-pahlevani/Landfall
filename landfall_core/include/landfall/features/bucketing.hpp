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

#ifndef LANDFALL_FEATURES_BUCKETING_HPP_
#define LANDFALL_FEATURES_BUCKETING_HPP_

#include <opencv2/core.hpp>
#include <vector>

#include "landfall/types.hpp"

namespace landfall
{

struct GridSize
{
  int cols = 8;
  int rows = 6;
};

/// Index of the grid cell containing a pixel (clamped to the image).
int gridCell(float x, float y, cv::Size image_size, const GridSize & grid);

/// Keeps at most `max_keypoints`, spread over the grid: every cell first keeps its strongest
/// keypoints up to an equal share, the leftover budget goes to the strongest remaining ones.
/// Deterministic (ties broken by position). Uniform coverage conditions PnP well.
std::vector<cv::KeyPoint> bucketKeypoints(
  std::vector<cv::KeyPoint> keypoints, cv::Size image_size, const GridSize & grid,
  int max_keypoints);

/// Fraction of grid cells that contain at least one of the pixels.
double gridCoverage(const std::vector<Vec2> & pixels, cv::Size image_size, const GridSize & grid);

}  // namespace landfall

#endif  // LANDFALL_FEATURES_BUCKETING_HPP_
