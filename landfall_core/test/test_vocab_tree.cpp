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

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <random>
#include <vector>

#include "landfall/features/orb.hpp"
#include "landfall/features/retrieval.hpp"
#include "landfall/features/vocab_tree.hpp"

using landfall::BowVector;
using landfall::Features;
using landfall::GlobalDescriptor;
using landfall::VocabularyTree;

namespace
{

// A "place": a cluttered scene that differs per seed.
cv::Mat placeImage(int seed)
{
  cv::RNG rng(seed);
  cv::Mat image(480, 640, CV_8UC1);
  rng.fill(image, cv::RNG::UNIFORM, 60, 200);
  cv::GaussianBlur(image, image, cv::Size(0, 0), 3.0);
  for (int i = 0; i < 300; ++i) {
    const cv::Point corner(rng.uniform(0, 640), rng.uniform(0, 480));
    const cv::Scalar color(rng.uniform(0, 256));
    if (i % 3 == 0) {
      cv::circle(image, corner, rng.uniform(3, 25), color, -1);
    } else {
      cv::rectangle(
        image, corner, corner + cv::Point(rng.uniform(4, 50), rng.uniform(4, 50)), color, -1);
    }
  }
  return image;
}

// The same place seen again: rotated, scaled, shifted, brighter or darker, with sensor noise.
cv::Mat revisit(const cv::Mat & image, int seed)
{
  cv::RNG rng(seed);
  const double angle = rng.uniform(-10.0, 10.0);
  const double scale = rng.uniform(0.9, 1.1);
  cv::Mat warp = cv::getRotationMatrix2D(cv::Point2f(320, 240), angle, scale);
  warp.at<double>(0, 2) += rng.uniform(-30.0, 30.0);
  warp.at<double>(1, 2) += rng.uniform(-30.0, 30.0);
  cv::Mat result;
  cv::warpAffine(image, result, warp, image.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
  cv::Mat noise(result.size(), CV_16SC1);
  rng.fill(noise, cv::RNG::NORMAL, 0, 4);
  cv::Mat noisy;
  result.convertTo(noisy, CV_16SC1, 1.0, rng.uniform(-20, 20));
  noisy += noise;
  noisy.convertTo(result, CV_8UC1);
  return result;
}

}  // namespace

TEST(VocabularyTree, BowRecallOnSyntheticPlaces)
{
  constexpr int kPlaces = 100;
  const landfall::OrbFeatures orb;
  std::vector<Features> database;
  cv::Mat all;
  for (int p = 0; p < kPlaces; ++p) {
    database.push_back(orb.extract(placeImage(1000 + p)));
    all.push_back(database.back().descriptors);
  }
  landfall::VocabularyParams params;
  params.depth = 4;  // ~190k training descriptors: about 20 per word
  auto vocabulary = std::make_shared<VocabularyTree>(VocabularyTree::train(all, params));
  std::vector<std::vector<uint32_t>> image_words;
  image_words.reserve(database.size());
  for (const auto & f : database) {
    image_words.push_back(vocabulary->words(f.descriptors));
  }
  vocabulary->computeIdf(image_words);
  const landfall::BowGlobalDescriptor bow(vocabulary);
  landfall::BowIndex index;
  for (int p = 0; p < kPlaces; ++p) {
    index.add(p, bow.compute(cv::Mat(), &database[p]));
  }

  int correct = 0;
  for (int p = 0; p < kPlaces; ++p) {
    const Features query = orb.extract(revisit(placeImage(1000 + p), 77 + p));
    const auto top = index.search(bow.compute(cv::Mat(), &query), 1);
    correct += !top.empty() && top[0].id == p ? 1 : 0;
  }
  const double recall = static_cast<double>(correct) / kPlaces;
  RecordProperty("recall_at_1", std::to_string(recall));
  EXPECT_GE(recall, 0.95);
}

TEST(VocabularyTree, TrainingIsDeterministicAndSerializes)
{
  cv::Mat descriptors(5000, 32, CV_8UC1);
  cv::RNG(9).fill(descriptors, cv::RNG::UNIFORM, 0, 256);
  landfall::VocabularyParams params;
  params.depth = 3;
  const VocabularyTree a = VocabularyTree::train(descriptors, params);
  const VocabularyTree b = VocabularyTree::train(descriptors, params);
  EXPECT_EQ(a.serialize(), b.serialize());
  EXPECT_GT(a.numWords(), 500U);
  EXPECT_LE(a.numWords(), 1000U);

  const VocabularyTree restored = VocabularyTree::deserialize(a.serialize());
  EXPECT_EQ(restored.serialize(), a.serialize());
  EXPECT_EQ(restored.words(descriptors), a.words(descriptors));

  auto corrupt = a.serialize();
  corrupt.resize(corrupt.size() - 3);
  EXPECT_THROW(VocabularyTree::deserialize(corrupt), std::runtime_error);
  EXPECT_THROW(VocabularyTree::deserialize({1, 2, 3}), std::runtime_error);
}

TEST(VocabularyTree, FloatDescriptorsUseKMeans)
{
  // Ten well-separated clusters of 16-D points: each cluster ends up under one top-level node.
  std::mt19937 rng(4);
  std::normal_distribution<float> noise(0.0F, 0.05F);
  cv::Mat descriptors(1000, 16, CV_32FC1);
  for (int i = 0; i < descriptors.rows; ++i) {
    for (int j = 0; j < 16; ++j) {
      descriptors.at<float>(i, j) = (j == i % 10 ? 1.0F : 0.0F) + noise(rng);
    }
  }
  landfall::VocabularyParams params;
  params.depth = 2;
  const VocabularyTree tree = VocabularyTree::train(descriptors, params);
  EXPECT_EQ(tree.type(), landfall::DescriptorType::kFloat32);
  const auto words = tree.words(descriptors);
  // Words of different clusters never coincide.
  for (int i = 0; i < descriptors.rows; ++i) {
    for (int j = i + 1; j < 40; ++j) {
      if (i % 10 != j % 10) {
        EXPECT_NE(words[i], words[j]);
      }
    }
  }
  EXPECT_EQ(VocabularyTree::deserialize(tree.serialize()).words(descriptors), words);
}

TEST(VocabularyTree, IdfAndBowScore)
{
  cv::Mat descriptors(2000, 32, CV_8UC1);
  cv::RNG(11).fill(descriptors, cv::RNG::UNIFORM, 0, 256);
  landfall::VocabularyParams params;
  params.depth = 2;
  VocabularyTree tree = VocabularyTree::train(descriptors, params);
  const auto words = tree.words(descriptors);
  // Word words[0] occurs in both images, so it carries no information (idf = 0).
  tree.computeIdf({{words[0], words[1]}, {words[0], words[2]}});
  const BowVector v = tree.bow(descriptors.rowRange(0, 3));
  float total = 0.0F;
  for (const auto & [word, weight] : v) {
    EXPECT_NE(word, words[0]);
    total += weight;
  }
  EXPECT_NEAR(total, 1.0F, 1e-6F);
  EXPECT_NEAR(landfall::bowScore(v, v), 1.0, 1e-6);
  EXPECT_NEAR(landfall::bowScore(v, BowVector{}), 0.0, 1e-12);
  EXPECT_NEAR(landfall::bowScore({{1, 0.5F}, {2, 0.5F}}, {{2, 0.5F}, {3, 0.5F}}), 0.5, 1e-6);
}

TEST(RetrievalIndex, BowIndexMatchesBruteForceScores)
{
  std::mt19937 rng(8);
  std::uniform_int_distribution<uint32_t> word(0, 300);
  std::vector<BowVector> vectors;
  landfall::BowIndex index;
  for (int i = 0; i < 50; ++i) {
    std::map<uint32_t, float> entries;
    for (int j = 0; j < 40; ++j) {
      entries[word(rng)] += 1.0F;
    }
    BowVector v(entries.begin(), entries.end());
    for (auto & e : v) {
      e.second /= 40.0F;
    }
    vectors.push_back(v);
    index.add(100 + i, GlobalDescriptor{{}, v});
  }
  const auto results = index.search(GlobalDescriptor{{}, vectors[7]}, 5);
  ASSERT_EQ(results.size(), 5U);
  EXPECT_EQ(results[0].id, 107);
  for (const auto & r : results) {
    EXPECT_NEAR(r.score, landfall::bowScore(vectors[7], vectors[r.id - 100]), 1e-6);
  }
  for (std::size_t i = 1; i < results.size(); ++i) {
    EXPECT_GE(results[i - 1].score, results[i].score);
  }
}

TEST(RetrievalIndex, DenseIndexFindsTheNearestVector)
{
  std::mt19937 rng(2);
  std::normal_distribution<float> gaussian;
  const auto unit = [&](std::vector<float> v) {
    float n = 0.0F;
    for (float x : v) {
      n += x * x;
    }
    for (float & x : v) {
      x /= std::sqrt(n);
    }
    return v;
  };
  landfall::DenseIndex index;
  std::vector<std::vector<float>> rows;
  for (int i = 0; i < 200; ++i) {
    std::vector<float> v(64);
    for (float & x : v) {
      x = gaussian(rng);
    }
    rows.push_back(unit(v));
    index.add(i, GlobalDescriptor{rows.back(), {}});
  }
  for (int i = 0; i < 200; i += 17) {
    std::vector<float> q = rows[i];
    for (float & x : q) {
      x += 0.05F * gaussian(rng);
    }
    const auto top = index.search(GlobalDescriptor{unit(q), {}}, 3);
    ASSERT_EQ(top.size(), 3U);
    EXPECT_EQ(top[0].id, i);
  }
  EXPECT_EQ(index.search(GlobalDescriptor{rows[0], {}}, 1000).size(), 200U);
  EXPECT_THROW(index.add(1, GlobalDescriptor{{1.0F, 0.0F}, {}}), std::invalid_argument);
}
