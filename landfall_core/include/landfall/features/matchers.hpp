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

#ifndef LANDFALL_FEATURES_MATCHERS_HPP_
#define LANDFALL_FEATURES_MATCHERS_HPP_

#include "landfall/interfaces/matcher.hpp"

namespace landfall
{

struct RatioMatcherParams
{
  double ratio = 0.8;  ///< Lowe's ratio: best / second-best distance must be below this
  bool mutual = true;  ///< also require the database feature's best match to be the query one
};

/// Brute-force nearest neighbours with Lowe's ratio test (Hamming for binary descriptors, L2 for
/// float ones) and an optional mutual check.
class RatioMatcher : public IMatcher
{
public:
  explicit RatioMatcher(const RatioMatcherParams & params = RatioMatcherParams());
  Matches match(const Features & query, const Features & database) const override;
  BackendId id() const override;

private:
  RatioMatcherParams params_;
};

struct MutualNearestParams
{
  double min_cosine = 0.82;  ///< XFeat's reference threshold
};

/// Mutual nearest neighbours by cosine similarity, for L2-normalized float descriptors.
class MutualNearestMatcher : public IMatcher
{
public:
  explicit MutualNearestMatcher(const MutualNearestParams & params = MutualNearestParams());
  Matches match(const Features & query, const Features & database) const override;
  BackendId id() const override;

private:
  MutualNearestParams params_;
};

}  // namespace landfall

#endif  // LANDFALL_FEATURES_MATCHERS_HPP_
