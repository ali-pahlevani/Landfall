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

#include "landfall/backend_options.hpp"

#include <cstdlib>
#include <stdexcept>
#include <utility>

namespace landfall
{

std::string modelsDir(const std::string & dir)
{
  if (!dir.empty()) {
    return dir;
  }
  if (const char * env = std::getenv("LANDFALL_MODELS_DIR"); env != nullptr && *env != '\0') {
    return env;
  }
  const char * home = std::getenv("HOME");
  return std::string(home != nullptr ? home : "") + "/.local/share/landfall/models";
}

BackendOptionReader::BackendOptionReader(std::string backend, const BackendOptions & options)
: backend_(std::move(backend)), options_(options)
{
}

void BackendOptionReader::read(const std::string & key, int * value)
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

void BackendOptionReader::read(const std::string & key, double * value)
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

void BackendOptionReader::read(const std::string & key, bool * value)
{
  if (const std::string * text = find(key)) {
    check(*text == "true" || *text == "false", key, *text);
    *value = *text == "true";
  }
}

void BackendOptionReader::read(const std::string & key, std::string * value)
{
  if (const std::string * text = find(key)) {
    *value = *text;
  }
}

void BackendOptionReader::readGrid(const std::string & key, GridSize * grid)
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

void BackendOptionReader::expect(const std::string & key, const std::string & expected)
{
  if (const std::string * text = find(key)) {
    check(*text == expected, key, *text);
  }
}

void BackendOptionReader::ignoreRuntimeOptions()
{
  for (const char * key : {kModelsDirOption, kProviderOption, kThreadsOption}) {
    find(key);
  }
}

void BackendOptionReader::finish() const
{
  for (const auto & [key, value] : options_) {
    if (used_.count(key) == 0) {
      throw std::invalid_argument("backend '" + backend_ + "' has no option '" + key + "'");
    }
  }
}

const std::string * BackendOptionReader::find(const std::string & key)
{
  used_.insert(key);
  const auto it = options_.find(key);
  return it == options_.end() ? nullptr : &it->second;
}

void BackendOptionReader::check(bool ok, const std::string & key, const std::string & text) const
{
  if (!ok) {
    throw std::invalid_argument(
      "backend '" + backend_ + "': invalid value '" + text + "' for option '" + key + "'");
  }
}

}  // namespace landfall
