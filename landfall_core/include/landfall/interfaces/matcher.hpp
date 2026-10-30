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

#ifndef LANDFALL_INTERFACES_MATCHER_HPP_
#define LANDFALL_INTERFACES_MATCHER_HPP_

#include "landfall/interfaces/local_features.hpp"

namespace landfall
{

/// Matches query features against database features. Thread-safe like ILocalFeatures.
class IMatcher
{
public:
  virtual ~IMatcher() = default;
  /// At most one match per query feature and per database feature.
  virtual Matches match(const Features & query, const Features & database) const = 0;
  virtual BackendId id() const = 0;
};

}  // namespace landfall

#endif  // LANDFALL_INTERFACES_MATCHER_HPP_
