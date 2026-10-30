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

// Private helper: canonical "key: value" configuration text of a backend (sorted keys, numbers
// printed with round-trip precision), the input of BackendId::config.

#ifndef LANDFALL_CONFIG_TEXT_HPP_
#define LANDFALL_CONFIG_TEXT_HPP_

#include <cstdio>
#include <map>
#include <string>

namespace landfall::detail
{

class ConfigText
{
public:
  ConfigText & add(const std::string & key, const std::string & value)
  {
    entries_[key] = value;
    return *this;
  }
  ConfigText & add(const std::string & key, double value)
  {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.17g", value);
    return add(key, std::string(buffer));
  }
  ConfigText & add(const std::string & key, int value) { return add(key, std::to_string(value)); }
  ConfigText & add(const std::string & key, bool value)
  {
    return add(key, std::string(value ? "true" : "false"));
  }

  std::string str() const
  {
    std::string text;
    for (const auto & [key, value] : entries_) {
      text.append(key).append(": ").append(value).append("\n");
    }
    return text;
  }

private:
  std::map<std::string, std::string> entries_;
};

}  // namespace landfall::detail

#endif  // LANDFALL_CONFIG_TEXT_HPP_
