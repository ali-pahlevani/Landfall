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

#include "landfall/onnx/session.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace landfall::onnx
{

namespace
{

// One environment per process, as ONNX Runtime recommends; created on first use.
Ort::Env & environment()
{
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "landfall");
  return env;
}

void appendProvider(Ort::SessionOptions & options, ExecutionProvider provider)
{
  switch (provider) {
    case ExecutionProvider::kCpu:
      return;
    case ExecutionProvider::kCuda: {
      const OrtCUDAProviderOptions cuda;  // ONNX Runtime's defaults (device 0)
      options.AppendExecutionProvider_CUDA(cuda);
      return;
    }
    case ExecutionProvider::kTensorRt: {
      // The V2 options start from ONNX Runtime's defaults; the V1 struct has none.
      const OrtApi & api = Ort::GetApi();
      OrtTensorRTProviderOptionsV2 * tensorrt = nullptr;
      Ort::ThrowOnError(api.CreateTensorRTProviderOptions(&tensorrt));
      const std::unique_ptr<OrtTensorRTProviderOptionsV2, void (*)(OrtTensorRTProviderOptionsV2 *)>
        owned(tensorrt, api.ReleaseTensorRTProviderOptions);
      options.AppendExecutionProvider_TensorRT_V2(*owned);
      return;
    }
  }
}

}  // namespace

std::optional<ExecutionProvider> parseExecutionProvider(const std::string & name)
{
  if (name == "cpu") {
    return ExecutionProvider::kCpu;
  }
  if (name == "cuda") {
    return ExecutionProvider::kCuda;
  }
  if (name == "tensorrt") {
    return ExecutionProvider::kTensorRt;
  }
  return std::nullopt;
}

std::string toString(ExecutionProvider provider)
{
  switch (provider) {
    case ExecutionProvider::kCpu:
      return "cpu";
    case ExecutionProvider::kCuda:
      return "cuda";
    case ExecutionProvider::kTensorRt:
      return "tensorrt";
  }
  return "unknown";
}

struct Session::Impl
{
  // Ort::Session::Run is not const; concurrent Run calls are safe.
  mutable Ort::Session session{nullptr};
  std::vector<std::string> input_names;
  std::vector<std::string> output_names;
  std::vector<const char *> input_name_ptrs;
  std::vector<const char *> output_name_ptrs;
};

Session::Session(const std::string & model_path, const SessionOptions & options)
: impl_(std::make_unique<Impl>())
{
  try {
    Ort::SessionOptions session_options;
    if (options.threads > 0) {
      session_options.SetIntraOpNumThreads(options.threads);
    }
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    appendProvider(session_options, options.provider);
    impl_->session = Ort::Session(environment(), model_path.c_str(), session_options);
    Ort::AllocatorWithDefaultOptions allocator;
    for (std::size_t i = 0; i < impl_->session.GetInputCount(); ++i) {
      impl_->input_names.emplace_back(impl_->session.GetInputNameAllocated(i, allocator).get());
    }
    for (std::size_t i = 0; i < impl_->session.GetOutputCount(); ++i) {
      impl_->output_names.emplace_back(impl_->session.GetOutputNameAllocated(i, allocator).get());
    }
  } catch (const Ort::Exception & e) {
    throw std::runtime_error(
      "cannot load ONNX model " + model_path + " (" + toString(options.provider) +
      "): " + e.what());
  }
  for (const auto & name : impl_->input_names) {
    impl_->input_name_ptrs.push_back(name.c_str());
  }
  for (const auto & name : impl_->output_names) {
    impl_->output_name_ptrs.push_back(name.c_str());
  }
}

Session::~Session() = default;

const std::vector<std::string> & Session::inputNames() const { return impl_->input_names; }

const std::vector<std::string> & Session::outputNames() const { return impl_->output_names; }

std::vector<Tensor> Session::run(const std::vector<Tensor> & inputs) const
{
  if (inputs.size() != impl_->input_names.size()) {
    throw std::runtime_error(
      "the model takes " + std::to_string(impl_->input_names.size()) + " inputs, got " +
      std::to_string(inputs.size()));
  }
  try {
    const Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<Ort::Value> values;
    values.reserve(inputs.size());
    for (const Tensor & t : inputs) {
      // ONNX Runtime reads the input buffers; it never writes them.
      values.push_back(Ort::Value::CreateTensor<float>(
        memory, const_cast<float *>(t.values.data()), t.values.size(),  // NOLINT
        t.shape.data(), t.shape.size()));
    }
    std::vector<Ort::Value> outputs = impl_->session.Run(
      Ort::RunOptions{nullptr}, impl_->input_name_ptrs.data(), values.data(), values.size(),
      impl_->output_name_ptrs.data(), impl_->output_name_ptrs.size());
    std::vector<Tensor> result(outputs.size());
    for (std::size_t i = 0; i < outputs.size(); ++i) {
      const auto info = outputs[i].GetTensorTypeAndShapeInfo();
      result[i].shape = info.GetShape();
      const std::size_t count = info.GetElementCount();
      switch (info.GetElementType()) {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT: {
          const auto * data = outputs[i].GetTensorData<float>();
          result[i].values.assign(data, data + count);
          break;
        }
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_BOOL: {
          const auto * data = outputs[i].GetTensorData<bool>();
          result[i].flags.resize(count);
          std::transform(data, data + count, result[i].flags.begin(), [](bool b) {
            return static_cast<uint8_t>(b ? 1U : 0U);
          });
          break;
        }
        default:
          throw std::runtime_error(
            "output " + impl_->output_names[i] + " is neither float32 nor bool");
      }
    }
    return result;
  } catch (const Ort::Exception & e) {
    throw std::runtime_error(std::string("ONNX Runtime: ") + e.what());
  }
}

std::string runtimeVersion() { return Ort::GetVersionString(); }

}  // namespace landfall::onnx
