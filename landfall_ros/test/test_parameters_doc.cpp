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

// docs/parameters.md is generated from the nodes' parameter declarations. This test fails when the
// committed file differs; `test_parameters_doc --print` writes the current one to stdout
// (scripts/update_parameters_doc.sh).

#include <gtest/gtest.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "landfall/ros/recorder_node.hpp"
#include "landfall/ros/relocalizer_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

std::string valueText(const rclcpp::Parameter & p)
{
  std::ostringstream out;
  switch (p.get_type()) {
    case rclcpp::ParameterType::PARAMETER_STRING:
      out << "`\"" << p.as_string() << "\"`";
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL:
      out << "`" << (p.as_bool() ? "true" : "false") << "`";
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      out << "`" << p.as_int() << "`";
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      out.precision(10);
      out << "`" << p.as_double() << "`";
      break;
    default:
      out << "`" << p.value_to_string() << "`";
  }
  return out.str();
}

std::string rangeText(const rcl_interfaces::msg::ParameterDescriptor & d)
{
  std::ostringstream out;
  out.precision(10);
  if (!d.floating_point_range.empty()) {
    out << "[" << d.floating_point_range.front().from_value << ", "
        << d.floating_point_range.front().to_value << "]";
  } else if (!d.integer_range.empty()) {
    out << "[" << d.integer_range.front().from_value << ", " << d.integer_range.front().to_value
        << "]";
  }
  return out.str();
}

std::string table(rclcpp_lifecycle::LifecycleNode & node)
{
  auto names = node.list_parameters({}, 0).names;
  std::sort(names.begin(), names.end());
  std::ostringstream out;
  out << "| Name | Type | Default | Range | Changes | Description |\n"
      << "|---|---|---|---|---|---|\n";
  for (const auto & name : names) {
    // rclcpp's own parameters (start_type_description_service exists from Jazzy on).
    if (
      name == "use_sim_time" || name == "start_type_description_service" ||
      name.rfind("qos_overrides.", 0) == 0) {
      continue;
    }
    const auto p = node.get_parameter(name);
    const auto d = node.describe_parameter(name);
    std::string description = d.description;
    for (std::size_t i = 0; (i = description.find('|', i)) != std::string::npos; i += 2) {
      description.replace(i, 1, "\\|");
    }
    out << "| `" << name << "` | " << p.get_type_name() << " | " << valueText(p) << " | "
        << rangeText(d) << " | " << (d.read_only ? "at startup" : "at runtime") << " | "
        << description << " |\n";
  }
  return out.str();
}

std::string render()
{
  auto relocalizer = std::make_shared<landfall::ros::RelocalizerNode>();
  auto recorder = std::make_shared<landfall::ros::RecorderNode>();
  std::ostringstream doc;
  doc << "# Parameters\n\n"
      << "<!-- Generated from the nodes' parameter declarations by "
         "scripts/update_parameters_doc.sh. Do not edit: test_parameters_doc fails when this file "
         "is stale. -->\n\n"
      << "Parameters marked *at startup* are read-only: set them in the launch file or YAML. The "
         "others can change with `ros2 param set` and apply to the next request (relocalizer) "
         "or frame (recorder). Every value is checked against its range on startup and on every "
         "change.\n\n"
      << "## landfall_relocalizer\n\n"
      << "Relocalizer node (`relocalizer_node`, component `landfall::ros::RelocalizerNode`); "
         "see [relocalization.md](relocalization.md).\n\n"
      << table(*relocalizer) << "\n"
      << "## landfall_recorder\n\n"
      << "Keyframe recorder (`recorder_node`, component `landfall::ros::RecorderNode`); see "
         "[recording.md](recording.md).\n\n"
      << table(*recorder);
  relocalizer->shutdown();
  recorder->shutdown();
  return doc.str();
}

std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

}  // namespace

TEST(ParametersDoc, IsUpToDate)
{
  const std::string path = std::string(LANDFALL_REPO_DIR) + "/docs/parameters.md";
  EXPECT_EQ(readFile(path), render())
    << path << " is stale: regenerate it with scripts/update_parameters_doc.sh";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  if (argc > 1 && std::strcmp(argv[1], "--print") == 0) {
    std::cout << render();
  } else {
    ::testing::InitGoogleTest(&argc, argv);
    result = RUN_ALL_TESTS();
  }
  rclcpp::shutdown();
  return result;
}
