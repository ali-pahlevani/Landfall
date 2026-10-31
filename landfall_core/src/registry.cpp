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

#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "landfall/features/matchers.hpp"
#include "landfall/features/orb.hpp"
#include "landfall/features/sift.hpp"

namespace landfall
{

namespace
{

// Reads typed values out of BackendOptions and rejects keys nobody asked for.
class OptionReader
{
public:
  OptionReader(std::string backend, const BackendOptions & options)
  : backend_(std::move(backend)), options_(options)
  {
  }

  void read(const std::string & key, int * value)
  {
    if (const std::string * text = find(key)) {
      std::size_t used = 0;
      try {
        *value = std::stoi(*text, &used);
      } catch (const std::exception &) {
        used = 0;
      }
      check(used != 0 && used == text->size(), key, *text);
    }
  }

  void read(const std::string & key, double * value)
  {
    if (const std::string * text = find(key)) {
      std::size_t used = 0;
      try {
        *value = std::stod(*text, &used);
      } catch (const std::exception &) {
        used = 0;
      }
      check(used != 0 && used == text->size(), key, *text);
    }
  }

  void read(const std::string & key, bool * value)
  {
    if (const std::string * text = find(key)) {
      check(*text == "true" || *text == "false", key, *text);
      *value = *text == "true";
    }
  }

  void readGrid(const std::string & key, GridSize * grid)
  {
    if (const std::string * text = find(key)) {
      const auto x = text->find('x');
      bool ok = x != std::string::npos;
      if (ok) {
        try {
          grid->cols = std::stoi(text->substr(0, x));
          grid->rows = std::stoi(text->substr(x + 1));
        } catch (const std::exception &) {
          ok = false;
        }
      }
      check(ok && grid->cols > 0 && grid->rows > 0, key, *text);
    }
  }

  /// Accepts a key whose value must equal `expected` (fixed properties such as a normalization).
  void expect(const std::string & key, const std::string & expected)
  {
    if (const std::string * text = find(key)) {
      check(*text == expected, key, *text);
    }
  }

  void finish() const
  {
    for (const auto & [key, value] : options_) {
      if (used_.count(key) == 0) {
        throw std::invalid_argument("backend '" + backend_ + "' has no option '" + key + "'");
      }
    }
  }

private:
  const std::string * find(const std::string & key)
  {
    used_.insert(key);
    const auto it = options_.find(key);
    return it == options_.end() ? nullptr : &it->second;
  }

  void check(bool ok, const std::string & key, const std::string & text) const
  {
    if (!ok) {
      throw std::invalid_argument(
        "backend '" + backend_ + "': invalid value '" + text + "' for option '" + key + "'");
    }
  }

  std::string backend_;
  const BackendOptions & options_;
  std::set<std::string> used_;
};

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

std::vector<std::string> BackendRegistry::localFeatureNames() const
{
  return keys(local_features_);
}

std::vector<std::string> BackendRegistry::matcherNames() const { return keys(matchers_); }

void registerBuiltinBackends(BackendRegistry & registry)
{
  registry.addLocalFeatures("orb", [](const BackendOptions & options) {
    OrbParams p;
    OptionReader reader("orb", options);
    reader.read("max_keypoints", &p.max_keypoints);
    reader.read("oversample", &p.oversample);
    reader.read("scale_factor", &p.scale_factor);
    reader.read("levels", &p.levels);
    reader.read("fast_threshold", &p.fast_threshold);
    reader.read("min_fast_threshold", &p.min_fast_threshold);
    reader.readGrid("grid", &p.grid);
    reader.finish();
    return std::make_unique<OrbFeatures>(p);
  });
  registry.addLocalFeatures("sift", [](const BackendOptions & options) {
    SiftParams p;
    OptionReader reader("sift", options);
    reader.read("max_keypoints", &p.max_keypoints);
    reader.read("contrast_threshold", &p.contrast_threshold);
    reader.read("edge_threshold", &p.edge_threshold);
    reader.read("sigma", &p.sigma);
    reader.read("octave_layers", &p.octave_layers);
    reader.readGrid("grid", &p.grid);
    reader.expect("normalization", "rootsift");
    reader.finish();
    return std::make_unique<SiftFeatures>(p);
  });
  registry.addMatcher("ratio", [](const BackendOptions & options) {
    RatioMatcherParams p;
    OptionReader reader("ratio", options);
    reader.read("ratio", &p.ratio);
    reader.read("mutual", &p.mutual);
    reader.finish();
    return std::make_unique<RatioMatcher>(p);
  });
  registry.addMatcher("mnn", [](const BackendOptions & options) {
    MutualNearestParams p;
    OptionReader reader("mnn", options);
    reader.read("min_cosine", &p.min_cosine);
    reader.finish();
    return std::make_unique<MutualNearestMatcher>(p);
  });
}

std::string defaultMatcherFor(DescriptorType type)
{
  return type == DescriptorType::kBinary ? "ratio" : "mnn";
}

}  // namespace landfall
