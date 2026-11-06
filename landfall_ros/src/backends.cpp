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

#include "landfall/ros/backends.hpp"

#ifdef LANDFALL_WITH_ONNX
#include "landfall/onnx/backends.hpp"
#include "landfall/onnx/session.hpp"
#endif

namespace landfall::ros
{

const BackendRegistry & backendRegistry()
{
  static const BackendRegistry registry = [] {
    BackendRegistry r;
    registerBuiltinBackends(r);
#ifdef LANDFALL_WITH_ONNX
    onnx::registerBackends(r);
#endif
    return r;
  }();
  return registry;
}

std::string learnedBackendsRuntime()
{
#ifdef LANDFALL_WITH_ONNX
  return onnx::runtimeVersion();
#else
  return "";
#endif
}

}  // namespace landfall::ros
