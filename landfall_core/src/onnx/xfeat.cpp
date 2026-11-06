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

#include "landfall/onnx/xfeat.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <numeric>
#include <stdexcept>

#include "landfall/util/sha256.hpp"
#include "opencv2/imgproc.hpp"

namespace landfall::onnx
{

namespace
{

constexpr int kDim = 64;
constexpr int kCell = 8;

// XFeat's InterpolateSparse2d normalizes a pixel position by (size - 1) of the full-resolution
// image; grid_sample (align_corners=False) maps it into a map of `map_size` cells. float, as
// PyTorch computes it.
float sourceIndex(float pos, int full_size, int map_size)
{
  const float g = 2.0F * (pos / static_cast<float>(full_size - 1)) - 1.0F;
  return ((g + 1.0F) * static_cast<float>(map_size) - 1.0F) / 2.0F;
}

// A value of a single-channel map, zero outside (grid_sample's zero padding).
float at(const float * map, int width, int height, int x, int y)
{
  return x >= 0 && x < width && y >= 0 && y < height
           ? map
               [static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
                static_cast<std::size_t>(x)]
           : 0.0F;
}

float nearest(const float * map, int width, int height, float ix, float iy)
{
  // grid_sample's nearest: round half to even.
  return at(
    map, width, height, static_cast<int>(std::nearbyint(ix)), static_cast<int>(std::nearbyint(iy)));
}

float bilinear(const float * map, int width, int height, float ix, float iy)
{
  // As grid_sample's CPU kernel: the corner weights first, then the weighted sum.
  const float x0 = std::floor(ix);
  const float y0 = std::floor(iy);
  const float west = ix - x0;
  const float north = iy - y0;
  const float east = 1.0F - west;
  const float south = 1.0F - north;
  const int x = static_cast<int>(x0);
  const int y = static_cast<int>(y0);
  return at(map, width, height, x, y) * (south * east) +
         at(map, width, height, x + 1, y) * (south * west) +
         at(map, width, height, x, y + 1) * (north * east) +
         at(map, width, height, x + 1, y + 1) * (north * west);
}

// PyTorch's cubic convolution weights (A = -0.75) for a fractional offset t
// (get_cubic_upsample_coefficients).
std::array<float, 4> cubicWeights(float t)
{
  constexpr float kA = -0.75F;
  const auto inner = [](float x) { return ((kA + 2.0F) * x - (kA + 3.0F)) * x * x + 1.0F; };
  const auto outer = [](float x) { return ((kA * x - 5.0F * kA) * x + 8.0F * kA) * x - 4.0F * kA; };
  const float t2 = 1.0F - t;
  return {outer(t + 1.0F), inner(t), inner(t2), outer(t2 + 1.0F)};
}

}  // namespace

Features detectXFeat(const XFeatMaps & maps, int max_keypoints, double detection_threshold)
{
  const int h = maps.height;
  const int w = maps.width;
  const int h8 = h / kCell;
  const int w8 = w / kCell;
  const auto pixels = static_cast<std::size_t>(h) * static_cast<std::size_t>(w);
  const auto cells = static_cast<std::size_t>(h8) * static_cast<std::size_t>(w8);
  if (
    h <= 0 || w <= 0 || h % 32 != 0 || w % 32 != 0 || maps.heatmap.size() != pixels ||
    maps.is_max.size() != pixels || maps.reliability.size() != cells ||
    maps.descriptors.size() != cells * kDim) {
    throw std::invalid_argument("XFeat maps of inconsistent sizes");
  }

  // Non-maximum suppression and scores, in row-major order like torch.nonzero.
  struct Candidate
  {
    int x;
    int y;
    float score;
  };
  std::vector<Candidate> candidates;
  const auto threshold = static_cast<float>(detection_threshold);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const std::size_t i =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      if (maps.is_max[i] == 0 || !(maps.heatmap[i] > threshold) || (x == 0 && y == 0)) {
        continue;
      }
      const auto px = static_cast<float>(x);
      const auto py = static_cast<float>(y);
      const float score =
        nearest(maps.heatmap.data(), w, h, sourceIndex(px, w, w), sourceIndex(py, h, h)) *
        bilinear(maps.reliability.data(), w8, h8, sourceIndex(px, w, w8), sourceIndex(py, h, h8));
      candidates.push_back({x, y, score});
    }
  }
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) { return a.score > b.score; });
  if (max_keypoints >= 0 && candidates.size() > static_cast<std::size_t>(max_keypoints)) {
    candidates.resize(static_cast<std::size_t>(max_keypoints));
  }
  while (!candidates.empty() && !(candidates.back().score > 0.0F)) {
    candidates.pop_back();
  }

  // Bicubic descriptors, L2-normalized.
  Features features;
  features.type = DescriptorType::kFloat32;
  features.descriptors = cv::Mat(static_cast<int>(candidates.size()), kDim, CV_32F);
  features.keypoints.reserve(candidates.size());
  for (std::size_t k = 0; k < candidates.size(); ++k) {
    const Candidate & c = candidates[k];
    const float ix = sourceIndex(static_cast<float>(c.x), w, w8);
    const float iy = sourceIndex(static_cast<float>(c.y), h, h8);
    const float x0 = std::floor(ix);
    const float y0 = std::floor(iy);
    const auto wx = cubicWeights(ix - x0);
    const auto wy = cubicWeights(iy - y0);
    const int bx = static_cast<int>(x0) - 1;
    const int by = static_cast<int>(y0) - 1;
    auto * row = features.descriptors.ptr<float>(static_cast<int>(k));
    double norm = 0.0;
    for (int d = 0; d < kDim; ++d) {
      const float * map = maps.descriptors.data() + static_cast<std::size_t>(d) * cells;
      float value = 0.0F;
      for (int j = 0; j < 4; ++j) {
        float line = 0.0F;
        for (int i = 0; i < 4; ++i) {
          line += at(map, w8, h8, bx + i, by + j) * wx[static_cast<std::size_t>(i)];
        }
        value += line * wy[static_cast<std::size_t>(j)];
      }
      row[d] = value;
      norm += static_cast<double>(value) * static_cast<double>(value);
    }
    // F.normalize: divide by max(norm, 1e-12).
    const auto scale = static_cast<float>(1.0 / std::max(std::sqrt(norm), 1e-12));
    for (int d = 0; d < kDim; ++d) {
      row[d] *= scale;
    }
    features.keypoints.push_back(
      {static_cast<float>(c.x), static_cast<float>(c.y), static_cast<float>(kCell), -1.0F,
       c.score});
  }
  return features;
}

XFeatFeatures::XFeatFeatures(const XFeatParams & params) : params_(params)
{
  if (params_.max_keypoints < 1) {
    throw std::invalid_argument("xfeat: max_keypoints must be at least 1");
  }
  const auto sha = sha256File(params_.model);
  if (!sha) {
    throw std::runtime_error("xfeat: cannot read model " + params_.model);
  }
  model_sha256_ = *sha;
  session_ = std::make_unique<Session>(params_.model, params_.session);
  const auto & names = session_->outputNames();
  for (const char * wanted : {"descriptors", "heatmap", "reliability", "is_max"}) {
    const auto it = std::find(names.begin(), names.end(), wanted);
    if (it == names.end()) {
      throw std::runtime_error(
        "xfeat: model " + params_.model + " has no output '" + wanted +
        "' (export it with landfall_tools/export/xfeat.py)");
    }
    output_index_.push_back(static_cast<int>(it - names.begin()));
  }
  if (session_->inputNames().size() != 1) {
    throw std::runtime_error("xfeat: model " + params_.model + " must take one input (image)");
  }
}

Features XFeatFeatures::extract(const cv::Mat & gray) const
{
  if (gray.empty() || gray.type() != CV_8UC1) {
    throw std::invalid_argument("xfeat: expects an 8-bit grayscale image");
  }
  // XFeat.preprocess_tensor: [0, 1] floats, resized (bilinear) to a multiple of 32.
  const int h = gray.rows / 32 * 32;
  const int w = gray.cols / 32 * 32;
  if (h == 0 || w == 0) {
    throw std::invalid_argument("xfeat: the image is smaller than 32 x 32 pixels");
  }
  cv::Mat input;
  gray.convertTo(input, CV_32F, 1.0 / 255.0);
  if (h != gray.rows || w != gray.cols) {
    cv::resize(input, input, cv::Size(w, h), 0.0, 0.0, cv::INTER_LINEAR);
  }
  Tensor image;
  image.shape = {1, 1, h, w};
  image.values.assign(input.ptr<float>(0), input.ptr<float>(0) + input.total());
  std::vector<Tensor> outputs = session_->run({image});

  XFeatMaps maps;
  maps.height = h;
  maps.width = w;
  maps.descriptors = std::move(outputs[static_cast<std::size_t>(output_index_[0])].values);
  maps.heatmap = std::move(outputs[static_cast<std::size_t>(output_index_[1])].values);
  maps.reliability = std::move(outputs[static_cast<std::size_t>(output_index_[2])].values);
  maps.is_max = std::move(outputs[static_cast<std::size_t>(output_index_[3])].flags);
  Features features = detectXFeat(maps, params_.max_keypoints, params_.detection_threshold);
  const float rx = static_cast<float>(gray.cols) / static_cast<float>(w);
  const float ry = static_cast<float>(gray.rows) / static_cast<float>(h);
  for (auto & k : features.keypoints) {
    k.x *= rx;
    k.y *= ry;
  }
  return features;
}

BackendId XFeatFeatures::id() const
{
  // The model by file name: the relocalizer finds it in its models directory and compares the
  // SHA-256 with the database's.
  BackendId id;
  id.name = "xfeat";
  id.version = "1";
  char threshold[32];
  std::snprintf(threshold, sizeof(threshold), "%.6g", params_.detection_threshold);
  id.config = std::string("detection_threshold: ") + threshold +
              "\nmax_keypoints: " + std::to_string(params_.max_keypoints) +
              "\nmodel: " + std::filesystem::path(params_.model).filename().string() + "\n";
  id.model_sha256 = model_sha256_;
  return id;
}

}  // namespace landfall::onnx
