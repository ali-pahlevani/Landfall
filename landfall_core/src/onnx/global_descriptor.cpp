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

#include "landfall/onnx/global_descriptor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "landfall/util/sha256.hpp"
#include "opencv2/imgproc.hpp"

namespace landfall::onnx
{

namespace
{

std::string number(float value)
{
  char text[32];
  std::snprintf(text, sizeof(text), "%.6g", static_cast<double>(value));
  return text;
}

}  // namespace

std::string canonicalTriple(const std::array<float, 3> & values)
{
  return number(values[0]) + " " + number(values[1]) + " " + number(values[2]);
}

OnnxGlobalDescriptor::OnnxGlobalDescriptor(const OnnxGlobalParams & params) : params_(params)
{
  if (params_.width < 0 || params_.height < 0 || (params_.width == 0) != (params_.height == 0)) {
    throw std::invalid_argument(params_.name + ": width and height must both be 0 or both be set");
  }
  for (const float s : params_.std) {
    if (!(s > 0.0F)) {
      throw std::invalid_argument(params_.name + ": std must be positive");
    }
  }
  const auto sha = sha256File(params_.model);
  if (!sha) {
    throw std::runtime_error(params_.name + ": cannot read model " + params_.model);
  }
  model_sha256_ = *sha;
  session_ = std::make_unique<Session>(params_.model, params_.session);
  if (session_->inputNames().size() != 1 || session_->outputNames().size() != 1) {
    throw std::runtime_error(
      params_.name + ": model " + params_.model + " must map one input to one output");
  }
}

GlobalDescriptor OnnxGlobalDescriptor::compute(const cv::Mat & image, const Features *) const
{
  if (image.empty() || image.depth() != CV_8U || image.channels() == 2 || image.channels() > 4) {
    throw std::invalid_argument(params_.name + ": expects an 8-bit gray, BGR or BGRA image");
  }
  cv::Mat bgr = image;
  if (image.channels() == 1) {
    cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, bgr, cv::COLOR_BGRA2BGR);
  }
  if (params_.width > 0 && (bgr.cols != params_.width || bgr.rows != params_.height)) {
    const bool shrinking = params_.width < bgr.cols || params_.height < bgr.rows;
    cv::Mat resized;
    cv::resize(
      bgr, resized, cv::Size(params_.width, params_.height), 0.0, 0.0,
      shrinking ? cv::INTER_AREA : cv::INTER_LINEAR);
    bgr = resized;
  }
  const int h = bgr.rows;
  const int w = bgr.cols;
  const auto plane = static_cast<std::size_t>(h) * static_cast<std::size_t>(w);
  Tensor input;
  input.shape = {1, 3, h, w};
  input.values.resize(3 * plane);
  for (int y = 0; y < h; ++y) {
    const auto * row = bgr.ptr<uint8_t>(y);
    for (int x = 0; x < w; ++x) {
      const std::size_t i =
        static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x);
      for (std::size_t c = 0; c < 3; ++c) {
        // Model channel c: R, G, B from BGR when rgb, else B, G, R.
        const std::size_t source = params_.rgb ? 2 - c : c;
        const float value = static_cast<float>(row[3 * x + static_cast<int>(source)]) / 255.0F;
        input.values[c * plane + i] = (value - params_.mean[c]) / params_.std[c];
      }
    }
  }
  const std::vector<Tensor> outputs = session_->run({input});
  GlobalDescriptor descriptor;
  descriptor.dense = outputs.front().values;
  double norm = 0.0;
  for (const float v : descriptor.dense) {
    norm += static_cast<double>(v) * static_cast<double>(v);
  }
  const auto scale = static_cast<float>(1.0 / std::max(std::sqrt(norm), 1e-12));
  for (float & v : descriptor.dense) {
    v *= scale;
  }
  return descriptor;
}

BackendId OnnxGlobalDescriptor::id() const
{
  BackendId id;
  id.name = params_.name;
  id.version = "1";
  id.config =
    "height: " + std::to_string(params_.height) + "\nmean: " + canonicalTriple(params_.mean) +
    "\nmodel: " + std::filesystem::path(params_.model).filename().string() +
    "\nrgb: " + (params_.rgb ? "true" : "false") + "\nstd: " + canonicalTriple(params_.std) +
    "\nwidth: " + std::to_string(params_.width) + "\n";
  id.model_sha256 = model_sha256_;
  return id;
}

}  // namespace landfall::onnx
