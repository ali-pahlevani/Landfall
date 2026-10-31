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

#include "landfall/util/json.hpp"

#include <cmath>
#include <cstdio>

namespace landfall
{

void JsonWriter::separate()
{
  if (after_key_) {
    after_key_ = false;
    return;
  }
  if (!first_.empty()) {
    if (!first_.back()) {
      out_ += ',';
    }
    first_.back() = false;
  }
}

JsonWriter & JsonWriter::beginObject()
{
  separate();
  out_ += '{';
  first_.push_back(true);
  return *this;
}

JsonWriter & JsonWriter::endObject()
{
  first_.pop_back();
  out_ += '}';
  return *this;
}

JsonWriter & JsonWriter::beginArray()
{
  separate();
  out_ += '[';
  first_.push_back(true);
  return *this;
}

JsonWriter & JsonWriter::endArray()
{
  first_.pop_back();
  out_ += ']';
  return *this;
}

JsonWriter & JsonWriter::key(const std::string & name)
{
  value(name);
  out_ += ':';
  after_key_ = true;
  return *this;
}

JsonWriter & JsonWriter::value(const std::string & text)
{
  separate();
  out_ += '"';
  for (const char c : text) {
    switch (c) {
      case '"':
        out_ += "\\\"";
        break;
      case '\\':
        out_ += "\\\\";
        break;
      case '\n':
        out_ += "\\n";
        break;
      case '\t':
        out_ += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char escaped[8];
          std::snprintf(escaped, sizeof(escaped), "\\u%04x", static_cast<unsigned>(c));
          out_ += escaped;
        } else {
          out_ += c;
        }
    }
  }
  out_ += '"';
  return *this;
}

JsonWriter & JsonWriter::value(double number)
{
  separate();
  if (!std::isfinite(number)) {
    out_ += "null";
    return *this;
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.10g", number);
  out_ += buffer;
  return *this;
}

JsonWriter & JsonWriter::value(int64_t number)
{
  separate();
  out_ += std::to_string(number);
  return *this;
}

JsonWriter & JsonWriter::value(bool flag)
{
  separate();
  out_ += flag ? "true" : "false";
  return *this;
}

}  // namespace landfall
