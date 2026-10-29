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

#include <regex>
#include <string>

#include "landfall/version.hpp"

TEST(Version, LibraryMatchesHeaders) { EXPECT_STREQ(landfall::version(), LANDFALL_VERSION_STRING); }

TEST(Version, IsSemanticVersion)
{
  const std::string version = landfall::version();
  EXPECT_TRUE(std::regex_match(version, std::regex(R"(\d+\.\d+\.\d+)"))) << version;

  const std::string from_parts = std::to_string(LANDFALL_VERSION_MAJOR) + "." +
                                 std::to_string(LANDFALL_VERSION_MINOR) + "." +
                                 std::to_string(LANDFALL_VERSION_PATCH);
  EXPECT_EQ(version, from_parts);
}
