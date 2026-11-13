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

#include "landfall/onnx/backends.hpp"

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include "landfall/features/floor_mask.hpp"
#include "landfall/onnx/global_descriptor.hpp"
#include "landfall/onnx/xfeat.hpp"

namespace landfall::onnx
{

namespace
{

SessionOptions sessionOptions(BackendOptionReader & reader, const std::string & backend)
{
  std::string provider = "cpu";
  int threads = 0;
  reader.read(kProviderOption, &provider);
  reader.read(kThreadsOption, &threads);
  const auto parsed = parseExecutionProvider(provider);
  if (!parsed) {
    throw std::invalid_argument(
      "backend '" + backend + "': unknown execution provider '" + provider +
      "' (cpu, cuda, tensorrt)");
  }
  return {*parsed, threads};
}

// The model file named by the `model` option, in the models directory.
std::string modelPath(BackendOptionReader & reader, const std::string & default_file)
{
  std::string file = default_file;
  std::string dir;
  reader.read("model", &file);
  reader.read(kModelsDirOption, &dir);
  return (std::filesystem::path(modelsDir(dir)) / file).string();
}

void addGlobal(BackendRegistry & registry, const OnnxGlobalParams & preset)
{
  registry.addGlobalDescriptor(preset.name, [preset](const BackendOptions & options) {
    OnnxGlobalParams p = preset;
    BackendOptionReader reader(p.name, options);
    p.model = modelPath(reader, p.model);
    reader.read("width", &p.width);
    reader.read("height", &p.height);
    // Fixed by the model's training: accepted when they match, never changed.
    reader.expect("rgb", p.rgb ? "true" : "false");
    reader.expect("mean", canonicalTriple(p.mean));
    reader.expect("std", canonicalTriple(p.std));
    p.session = sessionOptions(reader, p.name);
    reader.finish();
    return std::make_unique<OnnxGlobalDescriptor>(p);
  });
}

}  // namespace

void registerBackends(BackendRegistry & registry)
{
  registry.addLocalFeatures("xfeat", [](const BackendOptions & options) {
    XFeatParams p;
    BackendOptionReader reader("xfeat", options);
    p.model = modelPath(reader, "xfeat.onnx");
    reader.read("max_keypoints", &p.max_keypoints);
    reader.read("detection_threshold", &p.detection_threshold);
    reader.read(kFloorMaskOption, &p.floor_mask_distance);
    p.session = sessionOptions(reader, "xfeat");
    reader.finish();
    return std::make_unique<XFeatFeatures>(p);
  });

  OnnxGlobalParams eigenplaces;  // ImageNet normalization, RGB
  eigenplaces.name = "eigenplaces";
  eigenplaces.model = "eigenplaces_resnet18_512.onnx";
  addGlobal(registry, eigenplaces);

  OnnxGlobalParams megaloc;  // ImageNet normalization, RGB, evaluated at 322 x 322
  megaloc.name = "megaloc";
  megaloc.model = "megaloc.onnx";
  megaloc.width = 322;
  megaloc.height = 322;
  addGlobal(registry, megaloc);
}

}  // namespace landfall::onnx
