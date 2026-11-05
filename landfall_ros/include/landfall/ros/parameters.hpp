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

#ifndef LANDFALL_ROS_PARAMETERS_HPP_
#define LANDFALL_ROS_PARAMETERS_HPP_

#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <stdexcept>
#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace landfall::ros
{

/// Parameter descriptor; read-only parameters are fixed for the node's lifetime (set them at
/// startup), the others can change at runtime.
inline rcl_interfaces::msg::ParameterDescriptor describe(
  const std::string & description, bool read_only = true)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  d.read_only = read_only;
  return d;
}

inline rcl_interfaces::msg::ParameterDescriptor describe(
  const std::string & description, double from, double to, bool read_only = true)
{
  auto d = describe(description, read_only);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  d.floating_point_range.push_back(range);
  return d;
}

inline rcl_interfaces::msg::ParameterDescriptor describeInt(
  const std::string & description, int64_t from, int64_t to, bool read_only = true)
{
  auto d = describe(description, read_only);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = 1;
  d.integer_range.push_back(range);
  return d;
}

/// Replaces a leading `~` with $HOME.
inline std::string expandHome(const std::string & path)
{
  if (path.rfind('~', 0) == 0) {
    const char * home = std::getenv("HOME");
    return std::string(home != nullptr ? home : "") + path.substr(1);
  }
  return path;
}

/// Throws std::invalid_argument unless `value` is one of `allowed`.
inline void requireOneOf(
  const std::string & name, const std::string & value, std::initializer_list<const char *> allowed)
{
  std::string list;
  for (const char * option : allowed) {
    if (value == option) {
      return;
    }
    list += list.empty() ? option : std::string(", ") + option;
  }
  throw std::invalid_argument("parameter " + name + " must be one of: " + list);
}

}  // namespace landfall::ros

#endif  // LANDFALL_ROS_PARAMETERS_HPP_
