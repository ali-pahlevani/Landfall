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

#ifndef LANDFALL_FEATURES_VOCAB_TREE_HPP_
#define LANDFALL_FEATURES_VOCAB_TREE_HPP_

#include <cstdint>
#include <vector>

#include "landfall/interfaces/global_descriptor.hpp"

namespace landfall
{

struct VocabularyParams
{
  int branching = 10;   ///< k
  int depth = 5;        ///< L; up to k^L words
  int iterations = 10;  ///< Lloyd iterations per node
  uint64_t seed = 42;
  std::size_t max_training_descriptors = 1000000;  ///< deterministic subsample above this
};

/// Hierarchical k-means vocabulary tree with TF-IDF weighting (Nistér & Stewénius 2006), the
/// algorithm DBoW2 implements: k-majority clustering for binary descriptors, k-means for float
/// ones, k-means++ seeding. Trained per map on the map's own descriptors. Deterministic for a
/// given input and seed on every platform.
class VocabularyTree
{
public:
  VocabularyTree() = default;

  /// Trains on descriptor rows: CV_8U (binary, Hamming) or CV_32F (L2). Throws on empty input.
  static VocabularyTree train(const cv::Mat & descriptors, const VocabularyParams & params);

  bool empty() const { return num_words_ == 0; }
  DescriptorType type() const { return type_; }
  int descriptorSize() const { return columns_; }
  uint32_t numWords() const { return num_words_; }
  const VocabularyParams & params() const { return params_; }

  /// Word of one descriptor row.
  uint32_t word(const cv::Mat & descriptors, int row) const;
  std::vector<uint32_t> words(const cv::Mat & descriptors) const;

  /// IDF weights ln(N / n_i) from the words of the N database images; words that occur in no
  /// image get weight 0. Until called, every word weighs 1.
  void computeIdf(const std::vector<std::vector<uint32_t>> & image_words);

  /// TF-IDF bag of words, L1-normalized.
  BowVector bow(const cv::Mat & descriptors) const;

  std::vector<uint8_t> serialize() const;
  /// Throws std::runtime_error on malformed data.
  static VocabularyTree deserialize(const std::vector<uint8_t> & data);

private:
  struct Node
  {
    uint32_t first_child = 0;
    uint32_t child_count = 0;  ///< 0 for leaves
    int32_t word = -1;
  };

  double distance(const uint8_t * a, const uint8_t * b) const;
  const uint8_t * center(uint32_t node) const { return centers_.data() + node * rowBytes(); }
  std::size_t rowBytes() const;

  DescriptorType type_ = DescriptorType::kBinary;
  int columns_ = 0;
  VocabularyParams params_;
  std::vector<Node> nodes_;
  std::vector<uint8_t> centers_;  ///< one row per node (the root's is unused)
  std::vector<float> idf_;
  uint32_t num_words_ = 0;

  friend class VocabularyBuilder;
};

/// DBoW's L1 score of two L1-normalized vectors: 1 - |a - b|_1 / 2, in [0, 1].
double bowScore(const BowVector & a, const BowVector & b);

}  // namespace landfall

#endif  // LANDFALL_FEATURES_VOCAB_TREE_HPP_
