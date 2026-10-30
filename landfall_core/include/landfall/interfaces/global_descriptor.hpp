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

#ifndef LANDFALL_INTERFACES_GLOBAL_DESCRIPTOR_HPP_
#define LANDFALL_INTERFACES_GLOBAL_DESCRIPTOR_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "landfall/interfaces/local_features.hpp"

namespace landfall
{

/// Sparse bag-of-words vector: (word id, weight), sorted by word id, L1-normalized.
using BowVector = std::vector<std::pair<uint32_t, float>>;

/// Whole-image descriptor used for retrieval. Exactly one representation is filled.
struct GlobalDescriptor
{
  std::vector<float> dense;  ///< learned backends; L2-normalized
  BowVector bow;             ///< bag-of-words backends
};

/// Computes the global descriptor of an image. Thread-safe like ILocalFeatures.
class IGlobalDescriptor
{
public:
  virtual ~IGlobalDescriptor() = default;
  /// `features` are the image's local features; bag-of-words backends need them, learned ones
  /// use the image and may ignore them.
  virtual GlobalDescriptor compute(const cv::Mat & image, const Features * features) const = 0;
  virtual BackendId id() const = 0;
};

}  // namespace landfall

#endif  // LANDFALL_INTERFACES_GLOBAL_DESCRIPTOR_HPP_
