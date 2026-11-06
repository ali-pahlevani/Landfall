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

#ifndef LANDFALL_ONNX_BACKENDS_HPP_
#define LANDFALL_ONNX_BACKENDS_HPP_

#include "landfall/registry.hpp"

namespace landfall::onnx
{

/// Registers the learned backends (docs/models.md):
/// - local features `xfeat`: options model (xfeat.onnx), max_keypoints (2048),
///   detection_threshold (0.05);
/// - global descriptors `eigenplaces` (eigenplaces_resnet18_512.onnx, the image's own size) and
///   `megaloc` (megaloc.onnx, 322 x 322): options model, width, height.
/// `model` names a file in the models directory (runtime.models_dir, see modelsDir()); the
/// runtime options runtime.provider and runtime.threads choose where and how it runs.
void registerBackends(BackendRegistry & registry);

}  // namespace landfall::onnx

#endif  // LANDFALL_ONNX_BACKENDS_HPP_
