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

#ifndef LANDFALL_INTERFACES_RETRIEVAL_INDEX_HPP_
#define LANDFALL_INTERFACES_RETRIEVAL_INDEX_HPP_

#include <cstdint>
#include <vector>

#include "landfall/interfaces/global_descriptor.hpp"

namespace landfall
{

struct RetrievalCandidate
{
  int64_t id = 0;      ///< keyframe id
  double score = 0.0;  ///< higher is more similar
};

/// Nearest-neighbour search over the database's global descriptors. Build with add(), then
/// search() from any number of threads.
class IRetrievalIndex
{
public:
  virtual ~IRetrievalIndex() = default;
  virtual void add(int64_t id, const GlobalDescriptor & descriptor) = 0;
  virtual std::size_t size() const = 0;
  /// The k most similar entries, best first; ties broken by smaller id.
  virtual std::vector<RetrievalCandidate> search(const GlobalDescriptor & query, int k) const = 0;
};

}  // namespace landfall

#endif  // LANDFALL_INTERFACES_RETRIEVAL_INDEX_HPP_
