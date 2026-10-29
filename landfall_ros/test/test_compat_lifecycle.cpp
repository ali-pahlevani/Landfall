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

// Probes that bondcpp and diagnostic_updater accept a plain rclcpp_lifecycle::LifecycleNode on
// every supported distro (Landfall nodes don't use nav2_util::LifecycleNode).

#include <gtest/gtest.h>

#include <memory>

#include "bondcpp/bond.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class CompatLifecycle : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(CompatLifecycle, DiagnosticUpdaterAcceptsLifecycleNode)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("landfall_probe_diagnostics");
  diagnostic_updater::Updater updater(node);
  updater.setHardwareID("none");
  bool called = false;
  updater.add("probe", [&called](diagnostic_updater::DiagnosticStatusWrapper & status) {
    called = true;
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "ok");
  });
  updater.force_update();
  EXPECT_TRUE(called);

  // Lyrical warns when a lifecycle node is destroyed without being shut down.
  EXPECT_EQ(node->shutdown().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST_F(CompatLifecycle, BondAcceptsLifecycleNode)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("landfall_probe_bond");
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Same topic and id convention as nav2_lifecycle_manager (bond id = node name). The bond must be
  // owned by a shared_ptr: bondcpp 4.4 (Lyrical) calls shared_from_this() inside start().
  auto bond = std::make_shared<bond::Bond>("bond", node->get_name(), node);
  bond->setHeartbeatPeriod(0.10);
  bond->setHeartbeatTimeout(4.0);
  bond->start();
  EXPECT_FALSE(bond->isBroken());
  bond.reset();

  EXPECT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->shutdown().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}
