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

#include "landfall/features/vocab_tree.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>
#include <map>
#include <random>
#include <stdexcept>
#include <utility>

namespace landfall
{
namespace
{

constexpr uint32_t kMagic = 0x5456464C;  // "LFVT"
constexpr uint32_t kFormatVersion = 1;

double hamming(const uint8_t * a, const uint8_t * b, int bytes)
{
  int bits = 0;
  int i = 0;
  for (; i + 8 <= bytes; i += 8) {
    uint64_t x = 0;
    uint64_t y = 0;
    std::memcpy(&x, a + i, 8);
    std::memcpy(&y, b + i, 8);
    bits += __builtin_popcountll(x ^ y);
  }
  for (; i < bytes; ++i) {
    bits += __builtin_popcount(static_cast<unsigned>(a[i] ^ b[i]));
  }
  return bits;
}

double squaredL2(const uint8_t * a, const uint8_t * b, int count)
{
  double sum = 0.0;
  for (int i = 0; i < count; ++i) {
    float x = 0.0F;
    float y = 0.0F;
    const std::size_t at = sizeof(float) * static_cast<std::size_t>(i);
    std::memcpy(&x, a + at, sizeof(float));
    std::memcpy(&y, b + at, sizeof(float));
    sum += static_cast<double>(x - y) * (x - y);
  }
  return sum;
}

// Platform-independent random numbers (std distributions differ between standard libraries).
class Random
{
public:
  explicit Random(uint64_t seed) : engine_(seed) {}
  std::size_t index(std::size_t n) { return static_cast<std::size_t>(engine_() % n); }
  double unit() { return static_cast<double>(engine_() >> 11) * 0x1.0p-53; }

private:
  std::mt19937_64 engine_;
};

template <typename T>
void put(std::vector<uint8_t> & out, const T & value)
{
  const auto * bytes = reinterpret_cast<const uint8_t *>(&value);
  out.insert(out.end(), bytes, bytes + sizeof(T));
}

template <typename T>
T take(const std::vector<uint8_t> & in, std::size_t & offset)
{
  if (offset + sizeof(T) > in.size()) {
    throw std::runtime_error("vocabulary data is truncated");
  }
  T value;
  std::memcpy(&value, in.data() + offset, sizeof(T));
  offset += sizeof(T);
  return value;
}

}  // namespace

class VocabularyBuilder
{
public:
  VocabularyBuilder(VocabularyTree & tree, const cv::Mat & data)
  : tree_(tree), data_(data), random_(tree.params_.seed)
  {
  }

  void run(const std::vector<int> & members)
  {
    tree_.nodes_.emplace_back();
    tree_.centers_.assign(tree_.rowBytes(), 0);  // root center, unused
    build(0, members, 0);
  }

private:
  const uint8_t * row(int i) const { return data_.ptr<uint8_t>(i); }

  double distance(const uint8_t * a, const uint8_t * b) const { return tree_.distance(a, b); }

  void build(uint32_t node, const std::vector<int> & members, int level)
  {
    const int k = tree_.params_.branching;
    if (level >= tree_.params_.depth || members.size() <= 1) {
      tree_.nodes_[node].word = static_cast<int32_t>(tree_.num_words_++);
      return;
    }
    std::vector<std::vector<uint8_t>> centers;
    std::vector<std::vector<int>> clusters;
    if (members.size() <= static_cast<std::size_t>(k)) {
      // Too few descriptors to cluster: each one becomes a child, as in DBoW2.
      for (int m : members) {
        centers.emplace_back(row(m), row(m) + tree_.rowBytes());
        clusters.push_back({m});
      }
    } else {
      cluster(members, k, &centers, &clusters);
    }
    const auto first = static_cast<uint32_t>(tree_.nodes_.size());
    tree_.nodes_[node].first_child = first;
    tree_.nodes_[node].child_count = static_cast<uint32_t>(centers.size());
    for (const auto & c : centers) {
      tree_.nodes_.emplace_back();
      tree_.centers_.insert(tree_.centers_.end(), c.begin(), c.end());
    }
    for (std::size_t c = 0; c < clusters.size(); ++c) {
      build(first + static_cast<uint32_t>(c), clusters[c], level + 1);
    }
  }

  void cluster(
    const std::vector<int> & members, int k, std::vector<std::vector<uint8_t>> * centers,
    std::vector<std::vector<int>> * clusters)
  {
    const std::size_t bytes = tree_.rowBytes();
    // k-means++ seeding.
    std::vector<std::vector<uint8_t>> seeds;
    const int first = members[random_.index(members.size())];
    seeds.emplace_back(row(first), row(first) + bytes);
    std::vector<double> nearest(members.size(), std::numeric_limits<double>::max());
    while (static_cast<int>(seeds.size()) < k) {
      double total = 0.0;
      for (std::size_t i = 0; i < members.size(); ++i) {
        const double d = distance(row(members[i]), seeds.back().data());
        nearest[i] = std::min(nearest[i], d * d);
        total += nearest[i];
      }
      if (total <= 0.0) {
        break;  // all remaining descriptors coincide with a seed
      }
      double target = random_.unit() * total;
      std::size_t pick = members.size() - 1;
      for (std::size_t i = 0; i < members.size(); ++i) {
        target -= nearest[i];
        if (target <= 0.0) {
          pick = i;
          break;
        }
      }
      seeds.emplace_back(row(members[pick]), row(members[pick]) + bytes);
    }

    // Lloyd iterations.
    std::vector<int> assignment(members.size(), -1);
    for (int iteration = 0; iteration <= tree_.params_.iterations; ++iteration) {
      bool changed = false;
      for (std::size_t i = 0; i < members.size(); ++i) {
        int best = 0;
        double best_distance = std::numeric_limits<double>::max();
        for (std::size_t c = 0; c < seeds.size(); ++c) {
          const double d = distance(row(members[i]), seeds[c].data());
          if (d < best_distance) {
            best_distance = d;
            best = static_cast<int>(c);
          }
        }
        changed = changed || assignment[i] != best;
        assignment[i] = best;
      }
      if (!changed || iteration == tree_.params_.iterations) {
        break;
      }
      updateCenters(members, assignment, &seeds);
    }

    for (std::size_t c = 0; c < seeds.size(); ++c) {
      std::vector<int> cluster_members;
      for (std::size_t i = 0; i < members.size(); ++i) {
        if (assignment[i] == static_cast<int>(c)) {
          cluster_members.push_back(members[i]);
        }
      }
      if (!cluster_members.empty()) {
        centers->push_back(seeds[c]);
        clusters->push_back(std::move(cluster_members));
      }
    }
  }

  void updateCenters(
    const std::vector<int> & members, const std::vector<int> & assignment,
    std::vector<std::vector<uint8_t>> * centers) const
  {
    const int columns = tree_.columns_;
    for (std::size_t c = 0; c < centers->size(); ++c) {
      std::vector<double> sums(
        tree_.type_ == DescriptorType::kBinary ? static_cast<std::size_t>(columns) * 8 : columns,
        0.0);
      int count = 0;
      for (std::size_t i = 0; i < members.size(); ++i) {
        if (assignment[i] != static_cast<int>(c)) {
          continue;
        }
        ++count;
        const uint8_t * r = row(members[i]);
        if (tree_.type_ == DescriptorType::kBinary) {
          for (int b = 0; b < columns * 8; ++b) {
            sums[b] += (r[b / 8] >> (7 - b % 8)) & 1;
          }
        } else {
          for (int j = 0; j < columns; ++j) {
            float v = 0.0F;
            std::memcpy(&v, r + sizeof(float) * static_cast<std::size_t>(j), sizeof(float));
            sums[j] += v;
          }
        }
      }
      if (count == 0) {
        continue;  // keep the old center; dropped later if it stays empty
      }
      auto & center = (*centers)[c];
      if (tree_.type_ == DescriptorType::kBinary) {
        // k-majority: a bit is set when more than half of the members have it.
        std::fill(center.begin(), center.end(), 0);
        for (int b = 0; b < columns * 8; ++b) {
          if (sums[b] * 2 > count) {
            center[b / 8] = static_cast<uint8_t>(center[b / 8] | (1U << (7 - b % 8)));
          }
        }
      } else {
        for (int j = 0; j < columns; ++j) {
          const auto mean = static_cast<float>(sums[j] / count);
          std::memcpy(
            center.data() + sizeof(float) * static_cast<std::size_t>(j), &mean, sizeof(float));
        }
      }
    }
  }

  VocabularyTree & tree_;
  const cv::Mat & data_;
  Random random_;
};

std::size_t VocabularyTree::rowBytes() const
{
  return type_ == DescriptorType::kBinary ? static_cast<std::size_t>(columns_)
                                          : static_cast<std::size_t>(columns_) * 4;
}

double VocabularyTree::distance(const uint8_t * a, const uint8_t * b) const
{
  return type_ == DescriptorType::kBinary ? hamming(a, b, columns_) : squaredL2(a, b, columns_);
}

VocabularyTree VocabularyTree::train(const cv::Mat & descriptors, const VocabularyParams & params)
{
  if (descriptors.empty() || (descriptors.type() != CV_8UC1 && descriptors.type() != CV_32FC1)) {
    throw std::invalid_argument("vocabulary training needs CV_8U or CV_32F descriptor rows");
  }
  if (params.branching < 2 || params.depth < 1 || params.iterations < 1) {
    throw std::invalid_argument("invalid vocabulary parameters");
  }
  VocabularyTree tree;
  tree.type_ = descriptors.type() == CV_8UC1 ? DescriptorType::kBinary : DescriptorType::kFloat32;
  tree.columns_ = descriptors.cols;
  tree.params_ = params;

  const cv::Mat data = descriptors.isContinuous() ? descriptors : descriptors.clone();
  std::vector<int> members(data.rows);
  for (int i = 0; i < data.rows; ++i) {
    members[i] = i;
  }
  if (members.size() > params.max_training_descriptors) {
    Random random(params.seed ^ 0x9E3779B97F4A7C15ULL);
    for (std::size_t i = 0; i < params.max_training_descriptors; ++i) {
      std::swap(members[i], members[i + random.index(members.size() - i)]);
    }
    members.resize(params.max_training_descriptors);
    std::sort(members.begin(), members.end());
  }
  VocabularyBuilder(tree, data).run(members);
  tree.idf_.assign(tree.num_words_, 1.0F);
  return tree;
}

uint32_t VocabularyTree::word(const cv::Mat & descriptors, int row) const
{
  if (empty()) {
    throw std::logic_error("vocabulary is empty");
  }
  if (
    descriptors.cols != columns_ ||
    descriptors.type() != (type_ == DescriptorType::kBinary ? CV_8UC1 : CV_32FC1)) {
    throw std::invalid_argument("descriptor format doesn't match the vocabulary");
  }
  const auto * descriptor = descriptors.ptr<uint8_t>(row);
  uint32_t node = 0;
  while (nodes_[node].child_count > 0) {
    uint32_t best = nodes_[node].first_child;
    double best_distance = std::numeric_limits<double>::max();
    for (uint32_t c = 0; c < nodes_[node].child_count; ++c) {
      const uint32_t child = nodes_[node].first_child + c;
      const double d = distance(descriptor, center(child));
      if (d < best_distance) {
        best_distance = d;
        best = child;
      }
    }
    node = best;
  }
  return static_cast<uint32_t>(nodes_[node].word);
}

std::vector<uint32_t> VocabularyTree::words(const cv::Mat & descriptors) const
{
  std::vector<uint32_t> result(descriptors.rows);
  for (int i = 0; i < descriptors.rows; ++i) {
    result[i] = word(descriptors, i);
  }
  return result;
}

void VocabularyTree::computeIdf(const std::vector<std::vector<uint32_t>> & image_words)
{
  std::vector<uint32_t> images_with_word(num_words_, 0);
  for (const auto & words : image_words) {
    std::vector<uint32_t> unique = words;
    std::sort(unique.begin(), unique.end());
    unique.erase(std::unique(unique.begin(), unique.end()), unique.end());
    for (uint32_t w : unique) {
      if (w < num_words_) {
        ++images_with_word[w];
      }
    }
  }
  const auto n = static_cast<double>(image_words.size());
  idf_.assign(num_words_, 0.0F);
  for (uint32_t w = 0; w < num_words_; ++w) {
    if (images_with_word[w] > 0) {
      idf_[w] = static_cast<float>(std::log(n / images_with_word[w]));
    }
  }
}

BowVector VocabularyTree::bow(const cv::Mat & descriptors) const
{
  if (descriptors.rows == 0) {
    return {};
  }
  std::map<uint32_t, float> counts;
  for (uint32_t w : words(descriptors)) {
    counts[w] += 1.0F;
  }
  BowVector vector;
  double total = 0.0;
  for (const auto & [w, count] : counts) {
    const float weight = count / static_cast<float>(descriptors.rows) * idf_[w];
    if (weight > 0.0F) {
      vector.emplace_back(w, weight);
      total += weight;
    }
  }
  for (auto & entry : vector) {
    entry.second = static_cast<float>(entry.second / total);
  }
  return vector;
}

std::vector<uint8_t> VocabularyTree::serialize() const
{
  std::vector<uint8_t> out;
  put(out, kMagic);
  put(out, kFormatVersion);
  put(out, static_cast<uint8_t>(type_));
  put(out, static_cast<int32_t>(columns_));
  put(out, static_cast<int32_t>(params_.branching));
  put(out, static_cast<int32_t>(params_.depth));
  put(out, static_cast<int32_t>(params_.iterations));
  put(out, params_.seed);
  put(out, static_cast<uint64_t>(params_.max_training_descriptors));
  put(out, static_cast<uint32_t>(nodes_.size()));
  put(out, num_words_);
  for (const auto & node : nodes_) {
    put(out, node.first_child);
    put(out, node.child_count);
    put(out, node.word);
  }
  out.insert(out.end(), centers_.begin(), centers_.end());
  for (float w : idf_) {
    put(out, w);
  }
  return out;
}

VocabularyTree VocabularyTree::deserialize(const std::vector<uint8_t> & data)
{
  std::size_t offset = 0;
  const auto magic = take<uint32_t>(data, offset);
  const auto format_version = take<uint32_t>(data, offset);
  if (magic != kMagic || format_version != kFormatVersion) {
    throw std::runtime_error("not a Landfall vocabulary (or an unsupported format version)");
  }
  VocabularyTree tree;
  const auto type = take<uint8_t>(data, offset);
  if (type > static_cast<uint8_t>(DescriptorType::kFloat32)) {
    throw std::runtime_error("vocabulary has an unknown descriptor type");
  }
  tree.type_ = static_cast<DescriptorType>(type);
  tree.columns_ = take<int32_t>(data, offset);
  tree.params_.branching = take<int32_t>(data, offset);
  tree.params_.depth = take<int32_t>(data, offset);
  tree.params_.iterations = take<int32_t>(data, offset);
  tree.params_.seed = take<uint64_t>(data, offset);
  tree.params_.max_training_descriptors = take<uint64_t>(data, offset);
  const auto node_count = take<uint32_t>(data, offset);
  tree.num_words_ = take<uint32_t>(data, offset);
  if (tree.columns_ <= 0 || node_count == 0 || tree.num_words_ > node_count) {
    throw std::runtime_error("vocabulary header is inconsistent");
  }
  tree.nodes_.resize(node_count);
  for (auto & node : tree.nodes_) {
    node.first_child = take<uint32_t>(data, offset);
    node.child_count = take<uint32_t>(data, offset);
    node.word = take<int32_t>(data, offset);
    const bool leaf = node.child_count == 0;
    if (
      (leaf && (node.word < 0 || static_cast<uint32_t>(node.word) >= tree.num_words_)) ||
      (!leaf && static_cast<uint64_t>(node.first_child) + node.child_count > node_count)) {
      throw std::runtime_error("vocabulary tree structure is corrupt");
    }
  }
  const std::size_t center_bytes = tree.rowBytes() * node_count;
  if (offset + center_bytes + 4ULL * tree.num_words_ != data.size()) {
    throw std::runtime_error("vocabulary data has the wrong size");
  }
  const auto centers_begin = data.begin() + static_cast<std::ptrdiff_t>(offset);
  tree.centers_.assign(centers_begin, centers_begin + static_cast<std::ptrdiff_t>(center_bytes));
  offset += center_bytes;
  tree.idf_.resize(tree.num_words_);
  for (auto & w : tree.idf_) {
    w = take<float>(data, offset);
  }
  return tree;
}

double bowScore(const BowVector & a, const BowVector & b)
{
  double score = 0.0;
  auto i = a.begin();
  auto j = b.begin();
  while (i != a.end() && j != b.end()) {
    if (i->first < j->first) {
      ++i;
    } else if (j->first < i->first) {
      ++j;
    } else {
      score += std::abs(i->second) + std::abs(j->second) - std::abs(i->second - j->second);
      ++i;
      ++j;
    }
  }
  return 0.5 * score;
}

}  // namespace landfall
