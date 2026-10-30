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

#ifndef LANDFALL_FEATURES_RETRIEVAL_HPP_
#define LANDFALL_FEATURES_RETRIEVAL_HPP_

#include <memory>
#include <unordered_map>
#include <vector>

#include "landfall/features/vocab_tree.hpp"
#include "landfall/interfaces/retrieval_index.hpp"

namespace landfall
{

/// Exact cosine search over dense, L2-normalized descriptors.
class DenseIndex : public IRetrievalIndex
{
public:
  void add(int64_t id, const GlobalDescriptor & descriptor) override;
  std::size_t size() const override { return ids_.size(); }
  std::vector<RetrievalCandidate> search(const GlobalDescriptor & query, int k) const override;

private:
  std::size_t dimension_ = 0;
  std::vector<int64_t> ids_;
  std::vector<float> rows_;
};

/// Inverted file over bag-of-words vectors, scored with DBoW's L1 score.
class BowIndex : public IRetrievalIndex
{
public:
  void add(int64_t id, const GlobalDescriptor & descriptor) override;
  std::size_t size() const override { return ids_.size(); }
  std::vector<RetrievalCandidate> search(const GlobalDescriptor & query, int k) const override;

private:
  struct Posting
  {
    uint32_t entry;
    float weight;
  };
  std::vector<int64_t> ids_;
  std::unordered_map<uint32_t, std::vector<Posting>> postings_;
};

/// Bag-of-words global descriptor over a trained vocabulary (the classical backend).
class BowGlobalDescriptor : public IGlobalDescriptor
{
public:
  explicit BowGlobalDescriptor(std::shared_ptr<const VocabularyTree> vocabulary);
  GlobalDescriptor compute(const cv::Mat & image, const Features * features) const override;
  BackendId id() const override;

private:
  std::shared_ptr<const VocabularyTree> vocabulary_;
};

}  // namespace landfall

#endif  // LANDFALL_FEATURES_RETRIEVAL_HPP_
