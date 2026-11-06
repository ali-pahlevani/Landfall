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

#ifndef LANDFALL_ROS_BACKENDS_HPP_
#define LANDFALL_ROS_BACKENDS_HPP_

#include <string>

#include "landfall/registry.hpp"

namespace landfall::ros
{

/// The backends of this build: the built-in ones, and the learned ones (onnx::registerBackends)
/// when landfall_core was built with ONNX Runtime. Created on first use; read-only after.
const BackendRegistry & backendRegistry();

/// The ONNX Runtime version the learned backends use, or "" when the build has none.
std::string learnedBackendsRuntime();

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_BACKENDS_HPP_
