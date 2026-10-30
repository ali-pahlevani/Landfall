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

#ifndef LANDFALL_INTERFACES_LOCAL_FEATURES_HPP_
#define LANDFALL_INTERFACES_LOCAL_FEATURES_HPP_

#include <string>

#include "landfall/types.hpp"

namespace landfall
{

/// Identifies a backend and its exact configuration. Stored in the database, so the relocalizer
/// can refuse a database built with different backends.
struct BackendId
{
  std::string name;     ///< e.g. "orb"
  std::string version;  ///< bumped whenever the backend's output changes
  std::string config;   ///< canonical "key: value" lines, sorted by key
};

/// SHA-256 of the canonical configuration.
std::string configSha256(const BackendId & id);

/// Keypoint detector and descriptor. Implementations are immutable after construction and safe
/// to call from several threads.
class ILocalFeatures
{
public:
  virtual ~ILocalFeatures() = default;
  /// Extracts features from an 8-bit grayscale image.
  virtual Features extract(const cv::Mat & gray) const = 0;
  virtual BackendId id() const = 0;
};

}  // namespace landfall

#endif  // LANDFALL_INTERFACES_LOCAL_FEATURES_HPP_
