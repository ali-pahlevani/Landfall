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

#include "landfall/features/matchers.hpp"

#include <cmath>
#include <opencv2/features2d.hpp>
#include <stdexcept>
#include <vector>

#include "../config_text.hpp"

namespace landfall
{

RatioMatcher::RatioMatcher(const RatioMatcherParams & params) : params_(params)
{
  if (std::isnan(params_.ratio) || params_.ratio <= 0.0 || params_.ratio > 1.0) {
    throw std::invalid_argument("matcher ratio must be in (0, 1]");
  }
}

Matches RatioMatcher::match(const Features & query, const Features & database) const
{
  if (query.descriptors.empty() || database.descriptors.empty()) {
    return {};
  }
  if (query.type != database.type) {
    throw std::invalid_argument("cannot match descriptors of different types");
  }
  const int norm = query.type == DescriptorType::kBinary ? cv::NORM_HAMMING : cv::NORM_L2;
  cv::BFMatcher matcher(norm, false);
  std::vector<std::vector<cv::DMatch>> knn;
  matcher.knnMatch(query.descriptors, database.descriptors, knn, 2);
  std::vector<cv::DMatch> reverse;
  if (params_.mutual) {
    matcher.match(database.descriptors, query.descriptors, reverse);
  }
  Matches matches;
  for (const auto & candidates : knn) {
    if (candidates.empty()) {
      continue;
    }
    const cv::DMatch & best = candidates[0];
    if (candidates.size() > 1 && !(best.distance < params_.ratio * candidates[1].distance)) {
      continue;
    }
    if (params_.mutual && reverse[best.trainIdx].trainIdx != best.queryIdx) {
      continue;
    }
    matches.push_back({best.queryIdx, best.trainIdx, best.distance});
  }
  return matches;
}

BackendId RatioMatcher::id() const
{
  detail::ConfigText config;
  config.add("ratio", params_.ratio).add("mutual", params_.mutual);
  return {"ratio", "1", config.str()};
}

MutualNearestMatcher::MutualNearestMatcher(const MutualNearestParams & params) : params_(params) {}

Matches MutualNearestMatcher::match(const Features & query, const Features & database) const
{
  if (query.descriptors.empty() || database.descriptors.empty()) {
    return {};
  }
  if (query.type != DescriptorType::kFloat32 || database.type != DescriptorType::kFloat32) {
    throw std::invalid_argument("mutual nearest matching needs float descriptors");
  }
  cv::Mat similarity;
  cv::gemm(query.descriptors, database.descriptors, 1.0, cv::Mat(), 0.0, similarity, cv::GEMM_2_T);
  std::vector<int> best_database(similarity.rows, -1);
  std::vector<int> best_query(similarity.cols, -1);
  std::vector<float> best_query_score(similarity.cols, -2.0F);
  for (int i = 0; i < similarity.rows; ++i) {
    const auto * row = similarity.ptr<float>(i);
    float best = -2.0F;
    for (int j = 0; j < similarity.cols; ++j) {
      if (row[j] > best) {
        best = row[j];
        best_database[i] = j;
      }
      if (row[j] > best_query_score[j]) {
        best_query_score[j] = row[j];
        best_query[j] = i;
      }
    }
  }
  Matches matches;
  for (int i = 0; i < similarity.rows; ++i) {
    const int j = best_database[i];
    if (j < 0 || best_query[j] != i) {
      continue;
    }
    const float cosine = similarity.at<float>(i, j);
    if (cosine >= params_.min_cosine) {
      matches.push_back({i, j, 2.0F - 2.0F * cosine});
    }
  }
  return matches;
}

BackendId MutualNearestMatcher::id() const
{
  detail::ConfigText config;
  config.add("min_cosine", params_.min_cosine);
  return {"mnn", "1", config.str()};
}

}  // namespace landfall
