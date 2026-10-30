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

#ifndef LANDFALL_FEATURES_ORB_HPP_
#define LANDFALL_FEATURES_ORB_HPP_

#include "landfall/features/bucketing.hpp"
#include "landfall/interfaces/local_features.hpp"

namespace landfall
{

struct OrbParams
{
  int max_keypoints = 2000;
  double oversample = 2.0;  ///< detect this many times max_keypoints before bucketing
  double scale_factor = 1.2;
  int levels = 8;
  int fast_threshold = 20;
  int min_fast_threshold = 7;  ///< retried in cells that got less than half their share
  GridSize grid;
};

/// ORB (OpenCV, Harris-ranked FAST + rBRIEF, 256-bit) with grid bucketing as in ORB-SLAM.
class OrbFeatures : public ILocalFeatures
{
public:
  explicit OrbFeatures(const OrbParams & params = OrbParams());
  Features extract(const cv::Mat & gray) const override;
  BackendId id() const override;

private:
  OrbParams params_;
};

}  // namespace landfall

#endif  // LANDFALL_FEATURES_ORB_HPP_
