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

#ifndef LANDFALL_UTIL_JSON_HPP_
#define LANDFALL_UTIL_JSON_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace landfall
{

/// Minimal streaming JSON writer for reports (no parsing; yaml-cpp reads JSON back).
class JsonWriter
{
public:
  JsonWriter & beginObject();
  JsonWriter & endObject();
  JsonWriter & beginArray();
  JsonWriter & endArray();
  JsonWriter & key(const std::string & name);
  JsonWriter & value(const std::string & text);
  JsonWriter & value(const char * text) { return value(std::string(text)); }
  JsonWriter & value(double number);  ///< non-finite numbers become null
  JsonWriter & value(int64_t number);
  JsonWriter & value(int number) { return value(static_cast<int64_t>(number)); }
  JsonWriter & value(bool flag);
  /// Shorthand for key(name).value(v).
  template <typename T>
  JsonWriter & field(const std::string & name, const T & v)
  {
    return key(name).value(v);
  }

  const std::string & str() const { return out_; }

private:
  void separate();

  std::string out_;
  std::vector<bool> first_;  ///< per open container: nothing written yet
  bool after_key_ = false;
};

}  // namespace landfall

#endif  // LANDFALL_UTIL_JSON_HPP_
