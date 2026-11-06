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

#ifndef LANDFALL_ONNX_GLOBAL_DESCRIPTOR_HPP_
#define LANDFALL_ONNX_GLOBAL_DESCRIPTOR_HPP_

#include <array>
#include <memory>
#include <string>

#include "landfall/interfaces/global_descriptor.hpp"
#include "landfall/onnx/session.hpp"

namespace landfall::onnx
{

/// A learned global descriptor: one ONNX model from an image to an L2-normalized vector, with its
/// preprocessing. The presets (eigenplaces, megaloc) are in onnx/backends.hpp.
struct OnnxGlobalParams
{
  std::string name;  ///< backend name
  std::string model;
  int width = 0;  ///< input size; 0: the image's own
  int height = 0;
  bool rgb = true;  ///< channel order the model takes (images arrive BGR)
  /// Per channel, in the model's order, on values in [0, 1]: (value - mean) / std.
  std::array<float, 3> mean{0.485F, 0.456F, 0.406F};
  std::array<float, 3> std{0.229F, 0.224F, 0.225F};
  SessionOptions session;
};

/// "0.485 0.456 0.406": how a mean or std appears in the canonical configuration.
std::string canonicalTriple(const std::array<float, 3> & values);

/// Preprocessing: grayscale images are repeated over three channels (BGRA loses alpha); the image
/// is resized
/// (cv::INTER_AREA when shrinking, cv::INTER_LINEAR when enlarging), converted to the model's
/// channel order and normalized, then fed as float32 NCHW. The output is L2-normalized again.
class OnnxGlobalDescriptor : public IGlobalDescriptor
{
public:
  /// Throws std::runtime_error if the model can't be loaded or doesn't map one input to one
  /// output.
  explicit OnnxGlobalDescriptor(const OnnxGlobalParams & params);
  GlobalDescriptor compute(const cv::Mat & image, const Features * features) const override;
  BackendId id() const override;

private:
  OnnxGlobalParams params_;
  std::string model_sha256_;
  std::unique_ptr<Session> session_;
};

}  // namespace landfall::onnx

#endif  // LANDFALL_ONNX_GLOBAL_DESCRIPTOR_HPP_
