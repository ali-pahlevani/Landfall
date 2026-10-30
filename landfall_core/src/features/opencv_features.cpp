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

#include <algorithm>
#include <cmath>
#include <opencv2/features2d.hpp>
#include <set>
#include <stdexcept>

#include "../config_text.hpp"
#include "landfall/features/bucketing.hpp"
#include "landfall/features/orb.hpp"
#include "landfall/features/sift.hpp"
#include "landfall/util/sha256.hpp"

namespace landfall
{
namespace
{

bool stronger(const cv::KeyPoint & a, const cv::KeyPoint & b)
{
  if (a.response != b.response) {
    return a.response > b.response;
  }
  if (a.pt.y != b.pt.y) {
    return a.pt.y < b.pt.y;
  }
  if (a.pt.x != b.pt.x) {
    return a.pt.x < b.pt.x;
  }
  return a.octave < b.octave;
}

void requireGray(const cv::Mat & gray)
{
  if (gray.empty() || gray.type() != CV_8UC1) {
    throw std::invalid_argument("feature extraction needs a non-empty 8-bit grayscale image");
  }
}

Features toFeatures(
  const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & descriptors, DescriptorType type)
{
  Features features;
  features.type = type;
  features.descriptors = descriptors;
  features.keypoints.reserve(keypoints.size());
  for (const auto & k : keypoints) {
    features.keypoints.push_back({k.pt.x, k.pt.y, k.size, k.angle, k.response});
  }
  return features;
}

std::string gridText(const GridSize & grid)
{
  return std::to_string(grid.cols) + "x" + std::to_string(grid.rows);
}

}  // namespace

std::string configSha256(const BackendId & id)
{
  return sha256Hex(id.name + "\n" + id.version + "\n" + id.config);
}

int gridCell(float x, float y, cv::Size image_size, const GridSize & grid)
{
  const int col = std::clamp(
    static_cast<int>(
      std::floor(x * static_cast<float>(grid.cols) / static_cast<float>(image_size.width))),
    0, grid.cols - 1);
  const int row = std::clamp(
    static_cast<int>(
      std::floor(y * static_cast<float>(grid.rows) / static_cast<float>(image_size.height))),
    0, grid.rows - 1);
  return row * grid.cols + col;
}

std::vector<cv::KeyPoint> bucketKeypoints(
  std::vector<cv::KeyPoint> keypoints, cv::Size image_size, const GridSize & grid,
  int max_keypoints)
{
  std::sort(keypoints.begin(), keypoints.end(), stronger);
  if (max_keypoints <= 0) {
    return {};
  }
  if (keypoints.size() <= static_cast<std::size_t>(max_keypoints)) {
    return keypoints;
  }
  const int cells = grid.cols * grid.rows;
  const int share = std::max(1, max_keypoints / cells);
  std::vector<int> per_cell(cells, 0);
  std::vector<char> taken(keypoints.size(), 0);
  std::vector<cv::KeyPoint> result;
  result.reserve(max_keypoints);
  for (std::size_t i = 0;
       i < keypoints.size() && result.size() < static_cast<std::size_t>(max_keypoints); ++i) {
    const int cell = gridCell(keypoints[i].pt.x, keypoints[i].pt.y, image_size, grid);
    if (per_cell[cell] < share) {
      ++per_cell[cell];
      taken[i] = 1;
      result.push_back(keypoints[i]);
    }
  }
  for (std::size_t i = 0;
       i < keypoints.size() && result.size() < static_cast<std::size_t>(max_keypoints); ++i) {
    if (taken[i] == 0) {
      result.push_back(keypoints[i]);
    }
  }
  std::sort(result.begin(), result.end(), stronger);
  return result;
}

double gridCoverage(const std::vector<Vec2> & pixels, cv::Size image_size, const GridSize & grid)
{
  std::set<int> cells;
  for (const auto & p : pixels) {
    cells.insert(gridCell(static_cast<float>(p.x()), static_cast<float>(p.y()), image_size, grid));
  }
  return static_cast<double>(cells.size()) / (grid.cols * grid.rows);
}

OrbFeatures::OrbFeatures(const OrbParams & params) : params_(params)
{
  if (
    params_.max_keypoints <= 0 || params_.levels <= 0 || params_.grid.cols <= 0 ||
    params_.grid.rows <= 0 || params_.scale_factor <= 1.0 || params_.oversample < 1.0) {
    throw std::invalid_argument("invalid ORB parameters");
  }
}

Features OrbFeatures::extract(const cv::Mat & gray) const
{
  requireGray(gray);
  const int detect_count =
    static_cast<int>(std::lround(params_.max_keypoints * params_.oversample));
  const auto make_orb = [&](int fast_threshold) {
    return cv::ORB::create(
      detect_count, static_cast<float>(params_.scale_factor), params_.levels, 31, 0, 2,
      cv::ORB::HARRIS_SCORE, 31, fast_threshold);
  };
  const auto orb = make_orb(params_.fast_threshold);
  std::vector<cv::KeyPoint> keypoints;
  orb->detect(gray, keypoints);

  // Low-texture cells: detect again there with the lower FAST threshold, as ORB-SLAM does.
  const int cells = params_.grid.cols * params_.grid.rows;
  const int share = std::max(1, params_.max_keypoints / cells);
  std::vector<int> per_cell(cells, 0);
  for (const auto & k : keypoints) {
    ++per_cell[gridCell(k.pt.x, k.pt.y, gray.size(), params_.grid)];
  }
  cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
  std::vector<char> weak(cells, 0);
  for (int cell = 0; cell < cells; ++cell) {
    if (per_cell[cell] * 2 >= share) {
      continue;
    }
    weak[cell] = 1;
    const int col = cell % params_.grid.cols;
    const int row = cell / params_.grid.cols;
    const int x0 = col * gray.cols / params_.grid.cols;
    const int x1 = (col + 1) * gray.cols / params_.grid.cols;
    const int y0 = row * gray.rows / params_.grid.rows;
    const int y1 = (row + 1) * gray.rows / params_.grid.rows;
    mask(cv::Rect(x0, y0, x1 - x0, y1 - y0)).setTo(255);
  }
  if (params_.min_fast_threshold < params_.fast_threshold && cv::countNonZero(mask) > 0) {
    std::vector<cv::KeyPoint> extra;
    make_orb(params_.min_fast_threshold)->detect(gray, extra, mask);
    keypoints.erase(
      std::remove_if(
        keypoints.begin(), keypoints.end(),
        [&](const cv::KeyPoint & k) {
          return weak[gridCell(k.pt.x, k.pt.y, gray.size(), params_.grid)] != 0;
        }),
      keypoints.end());
    keypoints.insert(keypoints.end(), extra.begin(), extra.end());
  }

  keypoints =
    bucketKeypoints(std::move(keypoints), gray.size(), params_.grid, params_.max_keypoints);
  cv::Mat descriptors;
  orb->compute(gray, keypoints, descriptors);
  return toFeatures(keypoints, descriptors, DescriptorType::kBinary);
}

BackendId OrbFeatures::id() const
{
  detail::ConfigText config;
  config.add("max_keypoints", params_.max_keypoints)
    .add("oversample", params_.oversample)
    .add("scale_factor", params_.scale_factor)
    .add("levels", params_.levels)
    .add("fast_threshold", params_.fast_threshold)
    .add("min_fast_threshold", params_.min_fast_threshold)
    .add("grid", gridText(params_.grid));
  return {"orb", "1", config.str()};
}

SiftFeatures::SiftFeatures(const SiftParams & params) : params_(params)
{
  if (params_.max_keypoints <= 0 || params_.grid.cols <= 0 || params_.grid.rows <= 0) {
    throw std::invalid_argument("invalid SIFT parameters");
  }
}

Features SiftFeatures::extract(const cv::Mat & gray) const
{
  requireGray(gray);
  const auto sift = cv::SIFT::create(
    0, params_.octave_layers, params_.contrast_threshold, params_.edge_threshold, params_.sigma);
  std::vector<cv::KeyPoint> keypoints;
  sift->detect(gray, keypoints);
  keypoints =
    bucketKeypoints(std::move(keypoints), gray.size(), params_.grid, params_.max_keypoints);
  cv::Mat descriptors;
  sift->compute(gray, keypoints, descriptors);
  // RootSIFT: L1-normalize, then square root; the rows end up with unit L2 norm.
  for (int i = 0; i < descriptors.rows; ++i) {
    auto * row = descriptors.ptr<float>(i);
    float l1 = 0.0F;
    for (int j = 0; j < descriptors.cols; ++j) {
      l1 += std::abs(row[j]);
    }
    if (l1 > 0.0F) {
      for (int j = 0; j < descriptors.cols; ++j) {
        row[j] = std::sqrt(std::abs(row[j]) / l1);
      }
    }
  }
  return toFeatures(keypoints, descriptors, DescriptorType::kFloat32);
}

BackendId SiftFeatures::id() const
{
  detail::ConfigText config;
  config.add("max_keypoints", params_.max_keypoints)
    .add("contrast_threshold", params_.contrast_threshold)
    .add("edge_threshold", params_.edge_threshold)
    .add("sigma", params_.sigma)
    .add("octave_layers", params_.octave_layers)
    .add("grid", gridText(params_.grid))
    .add("normalization", std::string("rootsift"));
  return {"sift", "1", config.str()};
}

}  // namespace landfall
