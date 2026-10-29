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

// Probes that a nav2_behavior_tree::BtServiceNode subclass for the Relocalize service compiles
// and registers with the BehaviorTree.CPP version Nav2 uses on this distro. The real ReLocalize
// BT node arrives in M6; this test only exercises the API surface it will rely on.

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "landfall/nav2/compat/bt.hpp"
#include "landfall_msgs/msg/relocalization_result.hpp"
#include "landfall_msgs/srv/relocalize.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

namespace
{

class ProbeRelocalize : public nav2_behavior_tree::BtServiceNode<landfall_msgs::srv::Relocalize>
{
public:
  ProbeRelocalize(const std::string & name, const landfall::nav2::bt::NodeConfig & config)
  : nav2_behavior_tree::BtServiceNode<landfall_msgs::srv::Relocalize>(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<bool>("dry_run", false, "Run checks without publishing")});
  }

  void on_tick() override
  {
    bool dry_run = false;
    if (!getInput("dry_run", dry_run)) {
      dry_run = false;
    }
    request_->dry_run = dry_run;
  }

  BT::NodeStatus on_completion(
    std::shared_ptr<landfall_msgs::srv::Relocalize::Response> response) override
  {
    return response->result.status == landfall_msgs::msg::RelocalizationResult::SUCCESS
             ? BT::NodeStatus::SUCCESS
             : BT::NodeStatus::FAILURE;
  }
};

}  // namespace

TEST(CompatBt, BtServiceNodeSubclassRegistersWithPorts)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<ProbeRelocalize>("ProbeRelocalize");

  const auto & manifests = factory.manifests();
  const auto it = manifests.find("ProbeRelocalize");
  ASSERT_NE(it, manifests.end());
  const BT::PortsList & ports = it->second.ports;
  EXPECT_EQ(ports.count("dry_run"), 1u);
  EXPECT_EQ(ports.count("service_name"), 1u);    // from BtServiceNode
  EXPECT_EQ(ports.count("server_timeout"), 1u);  // from BtServiceNode
}

TEST(CompatBt, ReportsBehaviorTreeVersion)
{
  RecordProperty("btcpp_major", LANDFALL_BTCPP_V4 ? 4 : 3);
  SUCCEED() << "BehaviorTree.CPP v" << (LANDFALL_BTCPP_V4 ? 4 : 3);
}
