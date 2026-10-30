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

// Shared helpers of the landfall_core tests.

#ifndef LANDFALL_CORE_TEST_UTILS_HPP_
#define LANDFALL_CORE_TEST_UTILS_HPP_

#include <stdlib.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace landfall::test
{

/// A fresh directory under the system temp dir, removed with its content on destruction.
class TempDir
{
public:
  TempDir()
  {
    std::string pattern =
      (std::filesystem::temp_directory_path() / "landfall_test_XXXXXX").string();
    if (mkdtemp(pattern.data()) == nullptr) {
      throw std::runtime_error("mkdtemp failed");
    }
    path_ = pattern;
  }
  TempDir(const TempDir &) = delete;
  TempDir & operator=(const TempDir &) = delete;
  ~TempDir()
  {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }

  std::string file(const std::string & name) const { return (path_ / name).string(); }

private:
  std::filesystem::path path_;
};

inline void writeText(const std::string & path, const std::string & text)
{
  std::ofstream file(path, std::ios::binary);
  file << text;
}

}  // namespace landfall::test

#endif  // LANDFALL_CORE_TEST_UTILS_HPP_
