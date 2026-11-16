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

#ifndef LANDFALL_UTIL_CLOSENESS_HPP_
#define LANDFALL_UTIL_CLOSENESS_HPP_

#include <algorithm>

namespace landfall
{

// How close a measured value came to passing its check (V63), between 0 and 1: the relocalizer
// stops waiting for frames that come nowhere near a pose (EarlyStop).

/// For a value that must be at least `limit`: 1 when it is, else value / limit (0 for a limit of
/// 0 or less).
inline double closenessAtLeast(double value, double limit)
{
  if (value >= limit) {
    return 1.0;
  }
  return limit > 0.0 ? std::max(0.0, value / limit) : 0.0;
}

/// For a value that must be at most `limit`: 1 when it is, else limit / value (0 for a limit of 0
/// or less).
inline double closenessAtMost(double value, double limit)
{
  if (value <= limit) {
    return 1.0;
  }
  return limit > 0.0 ? limit / value : 0.0;
}

}  // namespace landfall

#endif  // LANDFALL_UTIL_CLOSENESS_HPP_
