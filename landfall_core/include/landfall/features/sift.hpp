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

#ifndef LANDFALL_FEATURES_SIFT_HPP_
#define LANDFALL_FEATURES_SIFT_HPP_

#include "landfall/features/bucketing.hpp"
#include "landfall/interfaces/local_features.hpp"

namespace landfall
{

struct SiftParams
{
  int max_keypoints = 2000;
  double contrast_threshold = 0.04;
  double edge_threshold = 10.0;
  double sigma = 1.6;
  int octave_layers = 3;
  GridSize grid;
};

/// SIFT (OpenCV; the patent expired in 2020) with grid bucketing and RootSIFT normalization
/// (L1, square root), as COLMAP uses. Descriptors are 128-D float rows with unit L2 norm.
class SiftFeatures : public ILocalFeatures
{
public:
  explicit SiftFeatures(const SiftParams & params = SiftParams());
  Features extract(const cv::Mat & gray) const override;
  BackendId id() const override;

private:
  SiftParams params_;
};

}  // namespace landfall

#endif  // LANDFALL_FEATURES_SIFT_HPP_
