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

#ifndef LANDFALL_BACKEND_OPTIONS_HPP_
#define LANDFALL_BACKEND_OPTIONS_HPP_

#include <map>
#include <set>
#include <string>

#include "landfall/features/bucketing.hpp"

namespace landfall
{

/// Backend options as text, keyed like the backend's canonical config (BackendId::config), so a
/// backend can be re-created from what the database stores.
using BackendOptions = std::map<std::string, std::string>;

/// Options that say how a backend runs, not what it computes: callers add them, and backends
/// keep them out of BackendId::config.
inline constexpr const char * kModelsDirOption = "runtime.models_dir";  ///< "": see modelsDir()
inline constexpr const char * kProviderOption = "runtime.provider";     ///< cpu, cuda, tensorrt
inline constexpr const char * kThreadsOption = "runtime.threads";       ///< 0: the runtime's

/// Where learned backends find their models: `dir` if set, else $LANDFALL_MODELS_DIR, else
/// ~/.local/share/landfall/models.
std::string modelsDir(const std::string & dir);

/// Reads typed values out of BackendOptions. finish() rejects the keys nobody read, so a typo in
/// a configuration fails loudly. Throws std::invalid_argument naming the backend and the key.
class BackendOptionReader
{
public:
  BackendOptionReader(std::string backend, const BackendOptions & options);

  void read(const std::string & key, int * value);
  void read(const std::string & key, double * value);
  void read(const std::string & key, bool * value);
  void read(const std::string & key, std::string * value);
  void readGrid(const std::string & key, GridSize * grid);
  /// Accepts a key whose value must equal `expected` (fixed properties such as a normalization).
  void expect(const std::string & key, const std::string & expected);
  /// Accepts the runtime options without using them (backends that load no model).
  void ignoreRuntimeOptions();
  void finish() const;

private:
  const std::string * find(const std::string & key);
  void check(bool ok, const std::string & key, const std::string & text) const;

  std::string backend_;
  const BackendOptions & options_;
  std::set<std::string> used_;
};

}  // namespace landfall

#endif  // LANDFALL_BACKEND_OPTIONS_HPP_
