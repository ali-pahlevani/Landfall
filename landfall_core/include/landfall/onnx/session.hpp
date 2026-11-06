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

#ifndef LANDFALL_ONNX_SESSION_HPP_
#define LANDFALL_ONNX_SESSION_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace landfall::onnx
{

/// Where a model runs. Only the CPU is part of the release guarantees (docs/PLAN.md D11); CUDA and
/// TensorRT need an ONNX Runtime built with them (ONNXRUNTIME_ROOT), not the rosdep one.
enum class ExecutionProvider : uint8_t {
  kCpu,
  kCuda,
  kTensorRt,
};

/// "cpu", "cuda" or "tensorrt".
std::optional<ExecutionProvider> parseExecutionProvider(const std::string & name);
std::string toString(ExecutionProvider provider);

struct SessionOptions
{
  ExecutionProvider provider = ExecutionProvider::kCpu;
  int threads = 0;  ///< intra-op threads; 0: ONNX Runtime's default (one per physical core)
};

/// A model input or output, row-major: float32 in `values`, or bool in `flags`.
struct Tensor
{
  std::vector<int64_t> shape;
  std::vector<float> values;
  std::vector<uint8_t> flags;
};

/// One ONNX model, loaded once. run() may be called from several threads at once, as ONNX
/// Runtime allows. ONNX Runtime stays an implementation detail: no header of it leaks here.
class Session
{
public:
  /// Throws std::runtime_error if the model can't be loaded or the provider isn't available.
  Session(const std::string & model_path, const SessionOptions & options);
  ~Session();
  Session(const Session &) = delete;
  Session & operator=(const Session &) = delete;
  Session(Session &&) = delete;
  Session & operator=(Session &&) = delete;

  const std::vector<std::string> & inputNames() const;
  const std::vector<std::string> & outputNames() const;

  /// Runs the model on float32 inputs, given in inputNames() order. Returns every output, in
  /// outputNames() order. Throws std::runtime_error on failure (wrong shapes, ...).
  std::vector<Tensor> run(const std::vector<Tensor> & inputs) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/// The version of the ONNX Runtime in use, e.g. "1.24.3".
std::string runtimeVersion();

}  // namespace landfall::onnx

#endif  // LANDFALL_ONNX_SESSION_HPP_
