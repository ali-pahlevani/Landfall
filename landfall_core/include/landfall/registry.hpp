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

#ifndef LANDFALL_REGISTRY_HPP_
#define LANDFALL_REGISTRY_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "landfall/backend_options.hpp"
#include "landfall/interfaces/global_descriptor.hpp"
#include "landfall/interfaces/local_features.hpp"
#include "landfall/interfaces/matcher.hpp"

namespace landfall
{

/// Parses canonical config text ("key: value" lines) into options.
BackendOptions parseBackendConfig(const std::string & config_text);

/// Creates backends by name. Registration is explicit (registerBuiltinBackends(), and the ONNX
/// backends' own function when built with ONNX Runtime): no static-initialization tricks.
class BackendRegistry
{
public:
  using LocalFeaturesFactory =
    std::function<std::unique_ptr<ILocalFeatures>(const BackendOptions &)>;
  using MatcherFactory = std::function<std::unique_ptr<IMatcher>(const BackendOptions &)>;
  using GlobalDescriptorFactory =
    std::function<std::unique_ptr<IGlobalDescriptor>(const BackendOptions &)>;

  void addLocalFeatures(const std::string & name, LocalFeaturesFactory factory);
  void addMatcher(const std::string & name, MatcherFactory factory);
  /// Learned global descriptors. The bag of words (`bow`) is not here: it needs the vocabulary
  /// the builder trains, and the builder and the localization map create it themselves.
  void addGlobalDescriptor(const std::string & name, GlobalDescriptorFactory factory);

  /// Throw std::invalid_argument for an unknown name, an unknown option or an invalid value.
  std::unique_ptr<ILocalFeatures> createLocalFeatures(
    const std::string & name, const BackendOptions & options) const;
  std::unique_ptr<IMatcher> createMatcher(
    const std::string & name, const BackendOptions & options) const;
  std::unique_ptr<IGlobalDescriptor> createGlobalDescriptor(
    const std::string & name, const BackendOptions & options) const;

  std::vector<std::string> localFeatureNames() const;
  std::vector<std::string> matcherNames() const;
  std::vector<std::string> globalDescriptorNames() const;
  bool hasGlobalDescriptor(const std::string & name) const;

private:
  std::map<std::string, LocalFeaturesFactory> local_features_;
  std::map<std::string, MatcherFactory> matchers_;
  std::map<std::string, GlobalDescriptorFactory> global_descriptors_;
};

/// Local features `orb` and `sift`, matchers `ratio` and `mnn`.
void registerBuiltinBackends(BackendRegistry & registry);

/// Matcher that suits a descriptor type: `ratio` for binary, `mnn` for float descriptors.
std::string defaultMatcherFor(DescriptorType type);

}  // namespace landfall

#endif  // LANDFALL_REGISTRY_HPP_
