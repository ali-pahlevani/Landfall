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

// The relocalizer and recorder load as components, the way a component container does it. This
// executable doesn't link the node libraries: class_loader only finds factories in libraries it
// loads itself.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

namespace
{

void expectComponent(
  const std::string & library, const std::string & node_class, const std::string & node_name)
{
  class_loader::ClassLoader loader(library);
  const auto classes = loader.getAvailableClasses<rclcpp_components::NodeFactory>();
  const std::string factory_name = "rclcpp_components::NodeFactoryTemplate<" + node_class + ">";
  ASSERT_NE(std::find(classes.begin(), classes.end(), factory_name), classes.end());
  auto factory = loader.createInstance<rclcpp_components::NodeFactory>(factory_name);
  auto wrapper = factory->create_node_instance(rclcpp::NodeOptions());
  EXPECT_EQ(std::string(wrapper.get_node_base_interface()->get_name()), node_name);
}

}  // namespace

TEST(Components, RelocalizerAndRecorderLoad)
{
  rclcpp::init(0, nullptr);
  expectComponent(
    LANDFALL_RELOCALIZER_LIBRARY, "landfall::ros::RelocalizerNode", "landfall_relocalizer");
  expectComponent(LANDFALL_RECORDER_LIBRARY, "landfall::ros::RecorderNode", "landfall_recorder");
  rclcpp::shutdown();
}
