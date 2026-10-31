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

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "landfall/registry.hpp"

using landfall::BackendOptions;
using landfall::BackendRegistry;

TEST(BackendRegistry, BuiltinBackendsRecreateFromTheirStoredConfig)
{
  BackendRegistry registry;
  landfall::registerBuiltinBackends(registry);
  EXPECT_EQ(registry.localFeatureNames(), (std::vector<std::string>{"orb", "sift"}));
  EXPECT_EQ(registry.matcherNames(), (std::vector<std::string>{"mnn", "ratio"}));

  // What the builder stores (BackendId::config) re-creates an identical backend.
  for (const std::string name : {"orb", "sift"}) {
    const auto original = registry.createLocalFeatures(name, {{"max_keypoints", "777"}});
    const auto id = original->id();
    EXPECT_NE(id.config.find("max_keypoints: 777"), std::string::npos);
    const auto again =
      registry.createLocalFeatures(id.name, landfall::parseBackendConfig(id.config));
    EXPECT_EQ(again->id().config, id.config);
    EXPECT_EQ(landfall::configSha256(again->id()), landfall::configSha256(id));
  }
  for (const std::string name : {"ratio", "mnn"}) {
    const auto id = registry.createMatcher(name, {})->id();
    EXPECT_EQ(
      registry.createMatcher(id.name, landfall::parseBackendConfig(id.config))->id().config,
      id.config);
  }
  const auto orb = registry.createLocalFeatures("orb", {{"grid", "4x3"}, {"scale_factor", "1.3"}});
  EXPECT_NE(orb->id().config.find("grid: 4x3"), std::string::npos);
  EXPECT_EQ(landfall::defaultMatcherFor(landfall::DescriptorType::kBinary), "ratio");
  EXPECT_EQ(landfall::defaultMatcherFor(landfall::DescriptorType::kFloat32), "mnn");
}

TEST(BackendRegistry, RejectsUnknownNamesOptionsAndValues)
{
  BackendRegistry registry;
  landfall::registerBuiltinBackends(registry);
  EXPECT_THROW(registry.createLocalFeatures("superpoint", {}), std::invalid_argument);
  EXPECT_THROW(registry.createMatcher("lightglue", {}), std::invalid_argument);
  EXPECT_THROW(
    registry.createLocalFeatures("orb", {{"max_keypionts", "10"}}), std::invalid_argument);
  EXPECT_THROW(
    registry.createLocalFeatures("orb", {{"max_keypoints", "ten"}}), std::invalid_argument);
  EXPECT_THROW(
    registry.createLocalFeatures("orb", {{"max_keypoints", "10.5"}}), std::invalid_argument);
  EXPECT_THROW(registry.createLocalFeatures("orb", {{"grid", "8by6"}}), std::invalid_argument);
  EXPECT_THROW(
    registry.createLocalFeatures("sift", {{"normalization", "l2"}}), std::invalid_argument);
  EXPECT_THROW(registry.createMatcher("ratio", {{"mutual", "yes"}}), std::invalid_argument);
  EXPECT_THROW(landfall::parseBackendConfig("no separator\n"), std::invalid_argument);
  EXPECT_EQ(
    landfall::parseBackendConfig("a: 1\nb: x y\n"), (BackendOptions{{"a", "1"}, {"b", "x y"}}));
}
