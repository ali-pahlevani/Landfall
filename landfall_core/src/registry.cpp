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

#include "landfall/registry.hpp"

#include <sstream>
#include <stdexcept>
#include <utility>

#include "landfall/features/floor_mask.hpp"
#include "landfall/features/matchers.hpp"
#include "landfall/features/orb.hpp"
#include "landfall/features/sift.hpp"

namespace landfall
{

namespace
{

template <typename Factory>
std::vector<std::string> keys(const std::map<std::string, Factory> & factories)
{
  std::vector<std::string> names;
  names.reserve(factories.size());
  for (const auto & entry : factories) {
    names.push_back(entry.first);
  }
  return names;
}

}  // namespace

BackendOptions parseBackendConfig(const std::string & config_text)
{
  BackendOptions options;
  std::istringstream lines(config_text);
  std::string line;
  while (std::getline(lines, line)) {
    if (line.empty()) {
      continue;
    }
    const auto colon = line.find(": ");
    if (colon == std::string::npos || colon == 0) {
      throw std::invalid_argument("malformed backend config line: '" + line + "'");
    }
    options[line.substr(0, colon)] = line.substr(colon + 2);
  }
  return options;
}

void BackendRegistry::addLocalFeatures(const std::string & name, LocalFeaturesFactory factory)
{
  local_features_[name] = std::move(factory);
}

void BackendRegistry::addMatcher(const std::string & name, MatcherFactory factory)
{
  matchers_[name] = std::move(factory);
}

std::unique_ptr<ILocalFeatures> BackendRegistry::createLocalFeatures(
  const std::string & name, const BackendOptions & options) const
{
  const auto it = local_features_.find(name);
  if (it == local_features_.end()) {
    throw std::invalid_argument("unknown local feature backend '" + name + "'");
  }
  return it->second(options);
}

std::unique_ptr<IMatcher> BackendRegistry::createMatcher(
  const std::string & name, const BackendOptions & options) const
{
  const auto it = matchers_.find(name);
  if (it == matchers_.end()) {
    throw std::invalid_argument("unknown matcher '" + name + "'");
  }
  return it->second(options);
}

void BackendRegistry::addGlobalDescriptor(const std::string & name, GlobalDescriptorFactory factory)
{
  global_descriptors_[name] = std::move(factory);
}

std::unique_ptr<IGlobalDescriptor> BackendRegistry::createGlobalDescriptor(
  const std::string & name, const BackendOptions & options) const
{
  const auto it = global_descriptors_.find(name);
  if (it == global_descriptors_.end()) {
    throw std::invalid_argument("unknown global descriptor backend '" + name + "'");
  }
  return it->second(options);
}

std::vector<std::string> BackendRegistry::globalDescriptorNames() const
{
  return keys(global_descriptors_);
}

bool BackendRegistry::hasGlobalDescriptor(const std::string & name) const
{
  return global_descriptors_.count(name) > 0;
}

std::vector<std::string> BackendRegistry::localFeatureNames() const
{
  return keys(local_features_);
}

std::vector<std::string> BackendRegistry::matcherNames() const { return keys(matchers_); }

void registerBuiltinBackends(BackendRegistry & registry)
{
  registry.addLocalFeatures("orb", [](const BackendOptions & options) {
    OrbParams p;
    BackendOptionReader reader("orb", options);
    reader.read("max_keypoints", &p.max_keypoints);
    reader.read("oversample", &p.oversample);
    reader.read("scale_factor", &p.scale_factor);
    reader.read("levels", &p.levels);
    reader.read("fast_threshold", &p.fast_threshold);
    reader.read("min_fast_threshold", &p.min_fast_threshold);
    reader.readGrid("grid", &p.grid);
    reader.read(kFloorMaskOption, &p.floor_mask_distance);
    reader.ignoreRuntimeOptions();
    reader.finish();
    return std::make_unique<OrbFeatures>(p);
  });
  registry.addLocalFeatures("sift", [](const BackendOptions & options) {
    SiftParams p;
    BackendOptionReader reader("sift", options);
    reader.read("max_keypoints", &p.max_keypoints);
    reader.read("contrast_threshold", &p.contrast_threshold);
    reader.read("edge_threshold", &p.edge_threshold);
    reader.read("sigma", &p.sigma);
    reader.read("octave_layers", &p.octave_layers);
    reader.readGrid("grid", &p.grid);
    reader.read(kFloorMaskOption, &p.floor_mask_distance);
    reader.expect("normalization", "rootsift");
    reader.ignoreRuntimeOptions();
    reader.finish();
    return std::make_unique<SiftFeatures>(p);
  });
  registry.addMatcher("ratio", [](const BackendOptions & options) {
    RatioMatcherParams p;
    BackendOptionReader reader("ratio", options);
    reader.read("ratio", &p.ratio);
    reader.read("mutual", &p.mutual);
    reader.ignoreRuntimeOptions();
    reader.finish();
    return std::make_unique<RatioMatcher>(p);
  });
  registry.addMatcher("mnn", [](const BackendOptions & options) {
    MutualNearestParams p;
    BackendOptionReader reader("mnn", options);
    reader.read("min_cosine", &p.min_cosine);
    reader.ignoreRuntimeOptions();
    reader.finish();
    return std::make_unique<MutualNearestMatcher>(p);
  });
}

std::string defaultMatcherFor(DescriptorType type)
{
  return type == DescriptorType::kBinary ? "ratio" : "mnn";
}

}  // namespace landfall
