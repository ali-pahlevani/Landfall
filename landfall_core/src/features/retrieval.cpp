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

#include "landfall/features/retrieval.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

#include "../config_text.hpp"
#include "landfall/util/sha256.hpp"

namespace landfall
{
namespace
{

std::vector<RetrievalCandidate> topK(std::vector<RetrievalCandidate> candidates, int k)
{
  const auto better = [](const RetrievalCandidate & a, const RetrievalCandidate & b) {
    return a.score != b.score ? a.score > b.score : a.id < b.id;
  };
  const std::size_t count = std::min(candidates.size(), static_cast<std::size_t>(std::max(k, 0)));
  const auto middle = candidates.begin() + static_cast<std::ptrdiff_t>(count);
  std::partial_sort(candidates.begin(), middle, candidates.end(), better);
  candidates.resize(count);
  return candidates;
}

}  // namespace

void DenseIndex::add(int64_t id, const GlobalDescriptor & descriptor)
{
  if (descriptor.dense.empty()) {
    throw std::invalid_argument("dense index needs dense descriptors");
  }
  if (dimension_ == 0) {
    dimension_ = descriptor.dense.size();
  } else if (descriptor.dense.size() != dimension_) {
    throw std::invalid_argument("descriptor dimension differs from the index");
  }
  ids_.push_back(id);
  rows_.insert(rows_.end(), descriptor.dense.begin(), descriptor.dense.end());
}

std::vector<RetrievalCandidate> DenseIndex::search(const GlobalDescriptor & query, int k) const
{
  if (ids_.empty()) {
    return {};
  }
  if (query.dense.size() != dimension_) {
    throw std::invalid_argument("query dimension differs from the index");
  }
  std::vector<RetrievalCandidate> candidates(ids_.size());
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    const float * row = rows_.data() + i * dimension_;
    double dot = 0.0;
    for (std::size_t j = 0; j < dimension_; ++j) {
      dot += static_cast<double>(row[j]) * query.dense[j];
    }
    candidates[i] = {ids_[i], dot};
  }
  return topK(std::move(candidates), k);
}

void BowIndex::add(int64_t id, const GlobalDescriptor & descriptor)
{
  const auto entry = static_cast<uint32_t>(ids_.size());
  ids_.push_back(id);
  for (const auto & [word, weight] : descriptor.bow) {
    postings_[word].push_back({entry, weight});
  }
}

std::vector<RetrievalCandidate> BowIndex::search(const GlobalDescriptor & query, int k) const
{
  // Only shared words contribute to the L1 score (see bowScore).
  std::vector<double> scores(ids_.size(), 0.0);
  for (const auto & [word, weight] : query.bow) {
    const auto it = postings_.find(word);
    if (it == postings_.end()) {
      continue;
    }
    for (const auto & posting : it->second) {
      scores[posting.entry] +=
        std::abs(weight) + std::abs(posting.weight) - std::abs(weight - posting.weight);
    }
  }
  std::vector<RetrievalCandidate> candidates;
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    if (scores[i] > 0.0) {
      candidates.push_back({ids_[i], 0.5 * scores[i]});
    }
  }
  return topK(std::move(candidates), k);
}

BowGlobalDescriptor::BowGlobalDescriptor(std::shared_ptr<const VocabularyTree> vocabulary)
: vocabulary_(std::move(vocabulary))
{
  if (!vocabulary_ || vocabulary_->empty()) {
    throw std::invalid_argument("bag-of-words descriptor needs a trained vocabulary");
  }
}

GlobalDescriptor BowGlobalDescriptor::compute(
  const cv::Mat & /*image*/, const Features * features) const
{
  if (features == nullptr) {
    throw std::invalid_argument("bag-of-words descriptor needs the image's local features");
  }
  GlobalDescriptor descriptor;
  descriptor.bow = vocabulary_->bow(features->descriptors);
  return descriptor;
}

BackendId BowGlobalDescriptor::id() const
{
  detail::ConfigText config;
  const auto & params = vocabulary_->params();
  const auto data = vocabulary_->serialize();
  config.add("branching", params.branching)
    .add("depth", params.depth)
    .add("vocabulary_sha256", sha256Hex(data.data(), data.size()));
  return {"bow", "1", config.str()};
}

}  // namespace landfall
